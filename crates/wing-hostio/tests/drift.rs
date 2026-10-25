//! Drift simulation: the engine (virtual-device clock) pushes 64-frame blocks, a simulated
//! hardware device with a clock off by +-200 ppm pulls callbacks through the bridge, both with
//! scheduling jitter. Over a simulated hour the ring must never under/overflow, the fill must
//! stay near the target and the ratio correction must be smooth and converge to the drift.

use std::time::Duration;

use wing_hostio::{bridge, BridgeConfig, BridgeState, ResamplerConfig};

/// Small deterministic PRNG (xorshift64*).
struct Rng(u64);
impl Rng {
    fn next_f64(&mut self) -> f64 {
        self.0 ^= self.0 >> 12;
        self.0 ^= self.0 << 25;
        self.0 ^= self.0 >> 27;
        (self.0.wrapping_mul(0x2545_F491_4F6C_DD1D) >> 11) as f64 / (1u64 << 53) as f64
    }
}

struct Report {
    underruns: u64,
    resyncs: u64,
    overflow: u64,
    max_fill_dev: f64,
    max_corr_jump_ppm: f64,
    mean_corr_ppm: f64,
    expected_corr_ppm: f64,
    corr_min_ppm: f64,
    corr_max_ppm: f64,
    target: f64,
}

/// `drift_ppm`: hardware clock error (+ = hardware faster). Returns stats after `seconds`.
/// `burst`: engine blocks delivered back to back per producer wake-up (e.g. 8 when the driver
/// IO cycle is 512 frames).
/// `virt_ppm`: error of the virtual device (engine) clock against host time.
fn simulate(
    drift_ppm: f64,
    virt_ppm: f64,
    device_rate: f64,
    buffer: usize,
    burst: usize,
    seconds: f64,
) -> Report {
    let source_rate = 48_000.0;
    let block = 64usize;
    let cfg = BridgeConfig {
        channels: 1,
        source_rate,
        max_block_frames: block * burst,
        target_latency: Duration::from_millis(5),
        // Short kernel: the loop does not depend on kernel quality, and this keeps an hour of
        // simulated audio quick.
        resampler: ResamplerConfig {
            taps: 16,
            phases: 64,
            ..Default::default()
        },
        ..Default::default()
    };
    let (mut prod, mut cons) = bridge(cfg);
    cons.configure(device_rate, buffer);
    cons.start();
    let stats = cons.stats().clone();

    let mut rng = Rng(0x9E37_79B9_7F4A_7C15);
    let hw_rate = device_rate * (1.0 + drift_ppm * 1e-6);
    let t0 = 1.0; // start host time at 1 s
    let prod_period = block as f64 / (source_rate * (1.0 + virt_ppm * 1e-6));
    let cons_period = buffer as f64 / hw_rate;
    let mut k = 0u64; // producer blocks
    let mut j = 0u64; // consumer callbacks
                      // Producer pushes after its cycle: up to 300 us of wake-up/processing jitter.
    let next_push = |k: u64, rng: &mut Rng| {
        let wake = k / burst as u64 + 1;
        t0 + (wake * burst as u64) as f64 * prod_period + 300e-6 * rng.next_f64()
    };
    let mut t_push = next_push(k, &mut rng);
    let mut t_cb = t0 + 0.0123;

    let mut blk = vec![0.0f32; block];
    let mut phase = 0.0f64;
    let settle = 120.0;
    let mut prev_corr: Option<f64> = None;
    let (mut corr_sum, mut corr_n) = (0.0f64, 0u64);
    let mut max_jump = 0.0f64;
    let mut max_fill_dev = 0.0f64;
    let (mut cmin, mut cmax) = (f64::MAX, f64::MIN);
    let end = t0 + seconds;
    let mut sink = 0.0f32;

    while t_cb < end {
        if t_push <= t_cb {
            for s in blk.iter_mut() {
                *s = (phase.sin() * 0.1) as f32;
                phase += 2.0 * std::f64::consts::PI * 440.0 / source_rate;
            }
            prod.push(&[&blk], (t_push * 1e9) as u64);
            k += 1;
            if k.is_multiple_of(burst as u64) {
                t_push = next_push(k, &mut rng);
            } else {
                t_push += 5e-6; // next block of the burst, right after
            }
        } else {
            // HAL "now" stamp: the ideal callback time with +-20 us of jitter.
            let stamp = t_cb + 40e-6 * (rng.next_f64() - 0.5);
            cons.render(buffer, (stamp * 1e9) as u64, |_, s| sink += s[0]);
            j += 1;
            t_cb = t0 + 0.0123 + j as f64 * cons_period;
            if t_cb - t0 > settle {
                let s = stats.snapshot();
                assert_eq!(s.state, BridgeState::Running);
                max_fill_dev = max_fill_dev.max((s.fill_frames - s.target_frames).abs());
                if let Some(p) = prev_corr {
                    max_jump = max_jump.max((s.correction_ppm - p).abs());
                }
                prev_corr = Some(s.correction_ppm);
                corr_sum += s.correction_ppm;
                corr_n += 1;
                cmin = cmin.min(s.correction_ppm);
                cmax = cmax.max(s.correction_ppm);
            }
        }
    }
    assert!(sink.is_finite());
    let s = stats.snapshot();
    Report {
        underruns: s.underruns,
        resyncs: s.resyncs,
        overflow: s.overflow_frames,
        max_fill_dev,
        max_corr_jump_ppm: max_jump,
        mean_corr_ppm: corr_sum / corr_n.max(1) as f64,
        // consumption = hw_rate * step * (1 + c) must equal source_rate.
        expected_corr_ppm: ((1.0 + virt_ppm * 1e-6) / (1.0 + drift_ppm * 1e-6) - 1.0) * 1e6,
        corr_min_ppm: cmin,
        corr_max_ppm: cmax,
        target: s.target_frames,
    }
}

fn check(r: &Report, label: &str) {
    eprintln!(
        "{label}: target {:.0} fr, max |fill-target| {:.1} fr, mean corr {:.2} ppm (want {:.2}, \
         range {:.2}..{:.2}), max jump {:.3} ppm/cb, underruns {}, resyncs {}, overflow {}",
        r.target,
        r.max_fill_dev,
        r.mean_corr_ppm,
        r.expected_corr_ppm,
        r.corr_min_ppm,
        r.corr_max_ppm,
        r.max_corr_jump_ppm,
        r.underruns,
        r.resyncs,
        r.overflow
    );
    assert_eq!(r.underruns, 0, "{label}: underruns");
    assert_eq!(r.resyncs, 0, "{label}: resyncs");
    assert_eq!(r.overflow, 0, "{label}: overflow");
    // Fill stays within ~1 ms of the target (target itself is buffer + 5 ms).
    assert!(
        r.max_fill_dev < 48.0,
        "{label}: fill deviation {}",
        r.max_fill_dev
    );
    // Converged on the true drift.
    assert!(
        (r.mean_corr_ppm - r.expected_corr_ppm).abs() < 0.5,
        "{label}: mean correction {} vs {}",
        r.mean_corr_ppm,
        r.expected_corr_ppm
    );
    // Smooth: steady-state wander small, no per-callback jumps.
    assert!(
        r.corr_max_ppm - r.corr_min_ppm < 6.0,
        "{label}: correction wander {}..{}",
        r.corr_min_ppm,
        r.corr_max_ppm
    );
    assert!(
        r.max_corr_jump_ppm < 0.5,
        "{label}: jump {}",
        r.max_corr_jump_ppm
    );
}

#[test]
fn one_hour_plus_200ppm() {
    check(
        &simulate(200.0, 0.0, 48_000.0, 256, 1, 3600.0),
        "+200 ppm, 48k, 256",
    );
}

#[test]
fn one_hour_minus_200ppm() {
    check(
        &simulate(-200.0, 0.0, 48_000.0, 128, 1, 3600.0),
        "-200 ppm, 48k, 128",
    );
}

#[test]
fn ten_minutes_44k1_device() {
    check(
        &simulate(150.0, 0.0, 44_100.0, 512, 1, 600.0),
        "+150 ppm, 44.1k, 512",
    );
}

#[test]
fn ten_minutes_bursty_producer() {
    // Engine runs 8 cycles back to back per 512-frame driver IO cycle.
    check(
        &simulate(-180.0, 0.0, 48_000.0, 128, 8, 600.0),
        "-180 ppm, 48k, 128, bursts of 8",
    );
}

#[test]
fn ten_minutes_both_clocks_drift() {
    // Virtual device clock itself 80 ppm fast against host time, hardware 120 ppm slow.
    check(
        &simulate(-120.0, 80.0, 48_000.0, 256, 1, 600.0),
        "-120 ppm hw, +80 ppm virt",
    );
}
