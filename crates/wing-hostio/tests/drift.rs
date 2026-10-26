//! Drift simulation of the whole bridge loop with simulated clocks.
//!
//! Producer side, modelled on the live system (T027): the virtual device (driver) runs IO
//! cycles of `driver_buf` frames on host time (optionally off by `virt_ppm`). Each cycle the
//! DAW does its own work inside the IO proc before the driver's WriteMix (variable, with steps
//! when e.g. playback starts), then the daemon wakes (os_sync latency), and runs all available
//! 64-frame engine blocks back to back, pushing each block into the bridge as it finishes
//! (~160 us per block). Occasionally the daemon wakes a whole cycle late and catches up.
//!
//! Consumer side: a hardware device with a clock off by `drift_ppm` pulls `buffer`-frame
//! callbacks; the HAL "now" stamp has +-20 us of jitter and the IO thread itself runs up to
//! 300 us late (so the ring snapshot is taken later than the stamp says).
//!
//! Each run reports underruns, fill deviation (estimate vs target), the true mean ring content,
//! the correction (mean/range vs the true drift), its largest per-callback jump, and the time
//! after which fill and correction stayed converged.

use std::collections::VecDeque;
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

#[derive(Clone, Copy)]
struct Scenario {
    /// Hardware clock error (+ = hardware faster).
    drift_ppm: f64,
    /// Virtual device clock error against host time.
    virt_ppm: f64,
    device_rate: f64,
    /// Device callback size.
    buffer: usize,
    /// Driver (DAW) IO buffer = producer burst.
    driver_buf: usize,
    /// DAW work inside its IO proc before WriteMix: base (s) and uniform jitter (s).
    daw_work: f64,
    daw_jitter: f64,
    /// From `daw_step_at` seconds on, the DAW's work grows by `daw_step` (s): playback start,
    /// plug-ins switched on. A phase step of the producer.
    daw_step: f64,
    daw_step_at: f64,
    /// At this time (s) the DAW skips a cycle: the driver delivers two buffers at once (the
    /// HAL's sample time jumped; the gap is zero-filled), a step of one whole buffer in the
    /// ring. The live v0.2 run's fill went 26 -> 36 ms this way and the old PI controller
    /// answered with a +-500 ppm limit cycle.
    extra_buffer_at: f64,
    /// Late daemon wake-ups: probability per cycle and lateness (s).
    late_wake: f64,
    late_by: f64,
    /// Engine block processing time (s).
    block_time: f64,
    seconds: f64,
    /// Seconds after which steady-state statistics are collected.
    settle: f64,
}

impl Scenario {
    /// Idealised producer as in the pre-T027 tests (tiny bursts, little jitter).
    fn ideal(drift_ppm: f64, device_rate: f64, buffer: usize, driver_buf: usize) -> Self {
        Self {
            drift_ppm,
            virt_ppm: 0.0,
            device_rate,
            buffer,
            driver_buf,
            daw_work: 0.0,
            daw_jitter: 300e-6,
            daw_step: 0.0,
            daw_step_at: f64::MAX,
            extra_buffer_at: f64::MAX,
            block_time: 5e-6,
            late_wake: 0.0,
            late_by: 0.0,
            seconds: 600.0,
            settle: 120.0,
        }
    }

    /// The live v0.2 pattern: DAW at `driver_buf`, USB interface at `buffer`.
    fn live(drift_ppm: f64, buffer: usize, driver_buf: usize) -> Self {
        Self {
            drift_ppm,
            virt_ppm: 0.0,
            device_rate: 48_000.0,
            buffer,
            driver_buf,
            daw_work: 0.5e-3,
            daw_jitter: 1.5e-3,
            daw_step: 0.0,
            daw_step_at: f64::MAX,
            extra_buffer_at: f64::MAX,
            block_time: 160e-6,
            late_wake: 1.0 / 3000.0,
            late_by: 3.2e-3,
            seconds: 600.0,
            settle: 30.0,
        }
    }
}

#[derive(Debug)]
struct Report {
    underruns: u64,
    resyncs: u64,
    overflow: u64,
    target: f64,
    max_fill_dev: f64,
    mean_ring: f64,
    mean_corr_ppm: f64,
    expected_corr_ppm: f64,
    corr_min_ppm: f64,
    corr_max_ppm: f64,
    /// Range of the controller's drift estimate (the correction without the phase part).
    drift_min_ppm: f64,
    drift_max_ppm: f64,
    max_corr_jump_ppm: f64,
    /// Last time (s) the estimate was > 1 ms off target or the correction > 20 ppm off.
    converged_after: f64,
    /// Over the last 60 s: max |fill - target| and max |correction - drift|.
    end_fill_dev: f64,
    end_corr_dev: f64,
    /// Callbacks after `settle` not in the Running state.
    not_running: u64,
    latency_ms: f64,
}

fn simulate(sc: Scenario) -> Report {
    let source_rate = 48_000.0;
    let block = 64usize;
    let cfg = BridgeConfig {
        channels: 1,
        source_rate,
        max_block_frames: sc.driver_buf,
        target_latency: Duration::from_millis(5),
        // Short kernel: the loop does not depend on kernel quality, and this keeps long runs
        // quick.
        resampler: ResamplerConfig {
            taps: 16,
            phases: 64,
            ..Default::default()
        },
        ..Default::default()
    };
    let (mut prod, mut cons) = bridge(cfg);
    cons.configure(sc.device_rate, sc.buffer);
    cons.start();
    let stats = cons.stats().clone();

    let mut rng = Rng(0x9E37_79B9_7F4A_7C15);
    let hw_rate = sc.device_rate * (1.0 + sc.drift_ppm * 1e-6);
    let t0 = 1.0; // host time starts at 1 s
    let cycle = sc.driver_buf as f64 / (source_rate * (1.0 + sc.virt_ppm * 1e-6));
    let cons_period = sc.buffer as f64 / hw_rate;
    let expected = ((1.0 + sc.virt_ppm * 1e-6) / (1.0 + sc.drift_ppm * 1e-6) - 1.0) * 1e6;

    // Pending pushes (host time, frames), generated one driver cycle at a time.
    let mut pushes: VecDeque<(f64, bool)> = VecDeque::new();
    let mut k = 0u64; // driver cycles generated
    let mut daemon_free = 0.0f64;
    let mut skipped = false;
    let mut gen_cycle = |pushes: &mut VecDeque<(f64, bool)>, rng: &mut Rng| {
        // Cycle k's frames are written at the end of its period plus the DAW's work.
        let t_cycle = t0 + (k + 1) as f64 * cycle;
        let mut base = sc.daw_work;
        if t_cycle - t0 >= sc.daw_step_at {
            base += sc.daw_step;
        }
        let mut blocks = sc.driver_buf / block;
        if t_cycle - t0 >= sc.extra_buffer_at && !skipped {
            skipped = true;
            blocks *= 2;
        }
        let written = t_cycle + base + sc.daw_jitter * rng.next_f64();
        let mut wake = written + 20e-6 + 150e-6 * rng.next_f64();
        if rng.next_f64() < sc.late_wake {
            wake += sc.late_by;
        }
        let mut t = wake.max(daemon_free);
        let n = blocks;
        for i in 0..n {
            t += sc.block_time * (0.8 + 0.4 * rng.next_f64());
            pushes.push_back((t, i + 1 == n));
        }
        daemon_free = t;
        k += 1;
    };

    let mut blk = vec![0.0f32; block];
    let mut phase = 0.0f64;
    let mut prev_corr: Option<f64> = None;
    let (mut corr_sum, mut ring_sum, mut n) = (0.0f64, 0.0f64, 0u64);
    let mut max_jump = 0.0f64;
    let mut max_fill_dev = 0.0f64;
    let (mut cmin, mut cmax) = (f64::MAX, f64::MIN);
    let (mut dmin, mut dmax) = (f64::MAX, f64::MIN);
    let mut converged_after = 0.0f64;
    let mut not_running = 0u64;
    let (mut end_fill_dev, mut end_corr_dev) = (0.0f64, 0.0f64);
    let mut sink = 0.0f32;
    let t_c0 = t0 + 0.0123;
    let mut j = 0u64;
    loop {
        let ideal = t_c0 + j as f64 * cons_period;
        if ideal - t0 > sc.seconds {
            break;
        }
        let exec = ideal + 300e-6 * rng.next_f64();
        // Everything the producer pushed before the callback actually runs.
        loop {
            if pushes.is_empty() {
                gen_cycle(&mut pushes, &mut rng);
            }
            let (t, last) = pushes[0];
            if t > exec {
                break;
            }
            pushes.pop_front();
            for s in blk.iter_mut() {
                *s = (phase.sin() * 0.1) as f32;
                phase += 2.0 * std::f64::consts::PI * 440.0 / source_rate;
            }
            // Like the daemon: blocks unstamped, one stamp per driver cycle.
            prod.push(&[&blk], 0);
            if last {
                prod.end_burst((t * 1e9) as u64, sc.driver_buf);
            }
        }
        let stamp = ideal + 40e-6 * (rng.next_f64() - 0.5);
        cons.render(sc.buffer, (stamp * 1e9) as u64, |_, s| sink += s[0]);
        j += 1;
        let s = stats.snapshot();
        let el = ideal - t0;
        if s.state == BridgeState::Running
            && ((s.fill_frames - s.target_frames).abs() > 48.0
                || (s.correction_ppm - expected).abs() > 20.0)
        {
            converged_after = el;
        }
        if el > sc.seconds - 60.0 {
            end_fill_dev = end_fill_dev.max((s.fill_frames - s.target_frames).abs());
            end_corr_dev = end_corr_dev.max((s.correction_ppm - expected).abs());
        }
        if el > sc.settle {
            if s.state != BridgeState::Running {
                not_running += 1;
            }
            max_fill_dev = max_fill_dev.max((s.fill_frames - s.target_frames).abs());
            if let Some(p) = prev_corr {
                max_jump = max_jump.max((s.correction_ppm - p).abs());
            }
            prev_corr = Some(s.correction_ppm);
            corr_sum += s.correction_ppm;
            ring_sum += s.ring_frames;
            n += 1;
            cmin = cmin.min(s.correction_ppm);
            cmax = cmax.max(s.correction_ppm);
            dmin = dmin.min(s.drift_ppm);
            dmax = dmax.max(s.drift_ppm);
        }
    }
    assert!(sink.is_finite());
    let s = stats.snapshot();
    Report {
        underruns: s.underruns,
        resyncs: s.resyncs,
        overflow: s.overflow_frames,
        target: s.target_frames,
        max_fill_dev,
        mean_ring: ring_sum / n.max(1) as f64,
        mean_corr_ppm: corr_sum / n.max(1) as f64,
        expected_corr_ppm: expected,
        corr_min_ppm: cmin,
        corr_max_ppm: cmax,
        drift_min_ppm: dmin,
        drift_max_ppm: dmax,
        max_corr_jump_ppm: max_jump,
        converged_after,
        end_fill_dev,
        end_corr_dev,
        not_running,
        latency_ms: cons.latency().as_secs_f64() * 1e3,
    }
}

fn print(r: &Report, label: &str) {
    eprintln!(
        "{label}: target {:.0} fr ({:.1} ms latency), max |fill-target| {:.1} fr, mean ring {:.0} \
         fr, corr mean {:.2} ppm (want {:.2}, range {:.2}..{:.2}, drift est {:.2}..{:.2}), max jump {:.3} ppm/cb, \
         converged after {:.1} s, underruns {}, resyncs {}, overflow {}, not running {}",
        r.target,
        r.latency_ms,
        r.max_fill_dev,
        r.mean_ring,
        r.mean_corr_ppm,
        r.expected_corr_ppm,
        r.corr_min_ppm,
        r.corr_max_ppm,
        r.drift_min_ppm,
        r.drift_max_ppm,
        r.max_corr_jump_ppm,
        r.converged_after,
        r.underruns,
        r.resyncs,
        r.overflow,
        r.not_running
    );
}

/// Common assertions; `wander_ppm`: allowed steady-state correction range.
fn check(r: &Report, label: &str, wander_ppm: f64) {
    print(r, label);
    assert_eq!(r.underruns, 0, "{label}: underruns");
    assert_eq!(r.resyncs, 0, "{label}: resyncs");
    assert_eq!(r.overflow, 0, "{label}: overflow");
    assert_eq!(r.not_running, 0, "{label}: not running");
    // Fill stays within 1 ms of the target.
    assert!(
        r.max_fill_dev < 48.0,
        "{label}: fill deviation {}",
        r.max_fill_dev
    );
    // Converged on the true drift.
    assert!(
        (r.mean_corr_ppm - r.expected_corr_ppm).abs() < 1.0,
        "{label}: mean correction {} vs {}",
        r.mean_corr_ppm,
        r.expected_corr_ppm
    );
    assert!(
        r.corr_max_ppm - r.corr_min_ppm < wander_ppm,
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

/// T027 acceptance: settled (fill within 1 ms, correction within 20 ppm) after 30 s.
fn check_live(r: &Report, label: &str) {
    check(r, label, 25.0);
    assert!(
        r.converged_after < 30.0,
        "{label}: converged only after {:.1} s",
        r.converged_after
    );
}

#[test]
fn one_hour_plus_200ppm() {
    let mut sc = Scenario::ideal(200.0, 48_000.0, 256, 64);
    sc.seconds = 3600.0;
    check(&simulate(sc), "+200 ppm, 48k, 256", 10.0);
}

#[test]
fn one_hour_minus_200ppm() {
    let mut sc = Scenario::ideal(-200.0, 48_000.0, 128, 64);
    sc.seconds = 3600.0;
    check(&simulate(sc), "-200 ppm, 48k, 128", 10.0);
}

#[test]
fn ten_minutes_44k1_device() {
    check(
        &simulate(Scenario::ideal(150.0, 44_100.0, 512, 64)),
        "+150 ppm, 44.1k, 512",
        10.0,
    );
}

#[test]
fn ten_minutes_both_clocks_drift() {
    // Virtual device clock itself 80 ppm fast against host time, hardware 120 ppm slow.
    let mut sc = Scenario::ideal(-120.0, 48_000.0, 256, 64);
    sc.virt_ppm = 80.0;
    check(&simulate(sc), "-120 ppm hw, +80 ppm virt", 10.0);
}

// ---- T027: the live v0.2 pattern (DAW at 512, USB interface at 128/256) ----------------------

#[test]
fn live_reaper512_device128_plus200() {
    check_live(
        &simulate(Scenario::live(200.0, 128, 512)),
        "live: REAPER 512 / dev 128, +200 ppm",
    );
}

#[test]
fn live_reaper512_device128_minus200() {
    check_live(
        &simulate(Scenario::live(-200.0, 128, 512)),
        "live: REAPER 512 / dev 128, -200 ppm",
    );
}

#[test]
fn live_reaper512_device256_plus200() {
    check_live(
        &simulate(Scenario::live(200.0, 256, 512)),
        "live: REAPER 512 / dev 256, +200 ppm",
    );
}

#[test]
fn live_reaper512_device256_minus200() {
    check_live(
        &simulate(Scenario::live(-200.0, 256, 512)),
        "live: REAPER 512 / dev 256, -200 ppm",
    );
}

#[test]
fn live_reaper512_device512_hw_locked() {
    // The live run: both clocks practically locked (|drift| < 1 ppm), DAW and interface 512.
    check_live(
        &simulate(Scenario::live(0.3, 512, 512)),
        "live: REAPER 512 / dev 512, +0.3 ppm",
    );
}

#[test]
fn live_reaper128_device128_minus150_virt_plus50() {
    let mut sc = Scenario::live(-150.0, 128, 128);
    sc.virt_ppm = 50.0;
    check_live(
        &simulate(sc),
        "live: REAPER 128 / dev 128, -150 ppm hw, +50 virt",
    );
}

/// Phase steps (T027 regression for the live +-500 ppm limit cycle): a whole extra driver
/// buffer lands at 120 s (+10.7 ms in the ring), the DAW's work grows by 2 ms at 300 s. Each is
/// drained at the bounded phase rate without overshoot, and without underruns.
#[test]
fn live_steps_drain_without_oscillation() {
    let mut sc = Scenario::live(-150.0, 128, 512);
    sc.extra_buffer_at = 120.0;
    sc.daw_step = 2e-3;
    sc.daw_step_at = 300.0;
    let r = simulate(sc);
    let label = "live steps: REAPER 512 / dev 128, -150 ppm, +1 buffer @120 s, +2 ms @300 s";
    print(&r, label);
    eprintln!(
        "{label}: last 60 s: max |fill-target| {:.1} fr, max |corr-drift| {:.2} ppm",
        r.end_fill_dev, r.end_corr_dev
    );
    assert_eq!(r.underruns, 0);
    assert_eq!(r.resyncs, 0);
    assert_eq!(r.not_running, 0);
    // Bounded: drift +- (phase-correction limit 150 ppm + estimate noise), no limit cycle.
    assert!(
        r.corr_max_ppm - r.expected_corr_ppm < 175.0
            && r.expected_corr_ppm - r.corr_min_ppm < 175.0,
        "{r:?}"
    );
    assert!(r.end_fill_dev < 48.0, "{r:?}");
    assert!(r.end_corr_dev < 20.0, "{r:?}");
}
