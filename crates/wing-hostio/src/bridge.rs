//! One bridge = ring + resampler + drift controller + stats, independent of CoreAudio.
//!
//! * [`BridgeProducer`] (engine thread): pushes a block of the mapped simulator channels per
//!   engine cycle, and marks the end of each burst of blocks (one driver IO cycle) with
//!   [`BridgeProducer::end_burst`].
//! * [`BridgeConsumer`] (device IO thread): [`BridgeConsumer::render`] pulls exactly what one
//!   device callback needs through the resampler, steering the ratio so the ring fill stays at
//!   the target latency.
//! * [`BridgeStats`]: lock-free counters/gauges readable from any thread.
//!
//! # Fill estimate and target (T027)
//!
//! The producer delivers bursts (a whole driver buffer, e.g. 512 frames, processed back to
//! back), so the raw ring count is a sawtooth of burst height that, sampled by device callbacks
//! of another size and clock, aliases into slow wander the controller would chase. Instead the
//! producer stamps `(host time, write index)` at the end of each burst; the consumer
//! extrapolates the producer's position to its own callback time along the nominal rate
//! (`end + (now - t) * fs`), i.e. the line through the burst ends. Sampled anywhere, that is
//! the same value up to the stamp jitter, which the controller averages out.
//!
//! In the worst callback phase (just before a burst lands) the raw content is the estimate
//! minus one burst, and that must still cover one device buffer, so
//! `target = burst + device buffer + margin` (`margin` = [`BridgeConfig::target_latency`]).
//! The burst follows the driver buffer at run time ([`BridgeProducer::end_burst`]).
//!
//! # Startup
//!
//! After [`BridgeConsumer::start`] the stale ring content is flushed, then the consumer primes:
//! silence until the producer has delivered a few fresh bursts and the estimate reaches the
//! target, then it drops the excess and plays. The drift learned before is kept across
//! restarts at the same rate. A ring that runs dry because the producer stopped (DAW stopped
//! its IO) re-primes without counting an underrun.
//!
//! The consumer is portable so the whole loop is testable with simulated clocks (see the tests
//! and `tests/drift.rs`).

use std::sync::atomic::{AtomicBool, AtomicU32, AtomicU64, Ordering};
use std::sync::Arc;
use std::time::Duration;

use crate::control::{DriftConfig, DriftController};
use crate::resampler::{Resampler, ResamplerConfig};
use crate::ring::{ring, RingConsumer, RingProducer};

/// Bridge parameters (rates in Hz).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BridgeConfig {
    /// Number of bridged channels.
    pub channels: usize,
    /// Simulator (virtual device) sample rate.
    pub source_rate: f64,
    /// Producer burst assumed until the producer reports one with
    /// [`BridgeProducer::end_burst`] (one engine block, or the driver IO buffer when the
    /// engine runs cycles back to back per driver IO cycle). Part of the latency budget.
    pub max_block_frames: usize,
    /// Safety margin held in the ring on top of one device buffer and one producer burst.
    pub target_latency: Duration,
    /// Ring capacity (at least this much audio).
    pub ring_duration: Duration,
    /// Drift controller tuning.
    pub drift: DriftConfig,
    /// Resampler kernel.
    pub resampler: ResamplerConfig,
}

impl Default for BridgeConfig {
    fn default() -> Self {
        Self {
            channels: 2,
            source_rate: 48_000.0,
            max_block_frames: 64,
            target_latency: Duration::from_millis(5),
            ring_duration: Duration::from_millis(250),
            drift: DriftConfig::default(),
            resampler: ResamplerConfig::default(),
        }
    }
}

/// Consumer state as reported in [`StatsSnapshot::state`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u32)]
pub enum BridgeState {
    /// Not attached to a running device.
    Stopped = 0,
    /// Waiting for the ring to fill to the target (outputs silence).
    Priming = 1,
    /// Playing.
    Running = 2,
}

impl BridgeState {
    fn from_u32(v: u32) -> Self {
        match v {
            1 => Self::Priming,
            2 => Self::Running,
            _ => Self::Stopped,
        }
    }
}

/// Shared, lock-free bridge statistics.
#[derive(Debug, Default)]
pub struct BridgeStats {
    active: AtomicBool,
    state: AtomicU32,
    fill_frames: AtomicU64,   // f64 bits
    ring_frames: AtomicU64,   // f64 bits
    target_frames: AtomicU64, // f64 bits
    correction: AtomicU64,    // f64 bits
    drift: AtomicU64,         // f64 bits
    underruns: AtomicU64,
    resyncs: AtomicU64,
    callbacks: AtomicU64,
    frames_out: AtomicU64,
    device_rate: AtomicU64, // f64 bits
    device_buffer: AtomicU32,
    reconnects: AtomicU64,
    overflow_frames: AtomicU64,
    /// Producer burst (frames) reported by the producer; 0 = not reported.
    burst_frames: AtomicU32,
    /// Burst the consumer's target currently uses.
    target_burst: AtomicU32,
    resampler_frames: AtomicU64, // f64 bits
    output_latency: AtomicU64,   // f64 bits, seconds
}

/// Point-in-time copy of [`BridgeStats`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct StatsSnapshot {
    /// Consumer state.
    pub state: BridgeState,
    /// Estimated ring fill (input frames): the producer position extrapolated from its recent
    /// burst stamps minus the consumer position; low-passed (the controller's view) while
    /// running.
    pub fill_frames: f64,
    /// Raw ring content (input frames, minus the resampler phase) at the last callback: the
    /// producer-burst sawtooth that `fill_frames` smooths out.
    pub ring_frames: f64,
    /// Fill target (input frames).
    pub target_frames: f64,
    /// Ratio correction in ppm (positive = consuming faster than nominal).
    pub correction_ppm: f64,
    /// The controller's drift estimate in ppm (the correction without the fill-error part).
    pub drift_ppm: f64,
    /// Drift ratio applied on top of the nominal device/source rate ratio
    /// (`1 / (1 + correction)`; < 1 means the device clock runs fast).
    pub ratio: f64,
    /// Callbacks that found too little data while the producer was running (output silence
    /// and re-primed).
    pub underruns: u64,
    /// Times the consumer discarded excess data to get back to the target.
    pub resyncs: u64,
    /// Device callbacks served.
    pub callbacks: u64,
    /// Output frames produced.
    pub frames_out: u64,
    /// Device sample rate (0 when stopped).
    pub device_rate: f64,
    /// Device buffer size in frames.
    pub device_buffer: u32,
    /// Device (re)connections.
    pub reconnects: u64,
    /// Frames the producer dropped because the ring was full.
    pub overflow_frames: u64,
    /// Producer burst the target is based on (input frames).
    pub burst_frames: u32,
    /// Resampler group delay (input frames).
    pub resampler_frames: f64,
    /// Device output latency (device latency + safety offset + one buffer), seconds.
    pub output_latency_s: f64,
}

impl StatsSnapshot {
    /// Fill expressed in milliseconds of source audio.
    pub fn fill_ms(&self, source_rate: f64) -> f64 {
        self.fill_frames / source_rate * 1e3
    }

    /// End-to-end latency (ms) from the producer finishing a burst (the daemon's push of a
    /// driver IO cycle, right after the DAW's write) until the burst's first frame leaves the
    /// device: the ring time of that frame (`fill - burst`, see the module docs), the resampler
    /// delay and the device's output latency. `None` unless running.
    pub fn latency_ms(&self, source_rate: f64) -> Option<f64> {
        (self.state == BridgeState::Running && source_rate > 0.0).then(|| {
            let ring = (self.fill_frames - self.burst_frames as f64).max(0.0);
            ((ring + self.resampler_frames) / source_rate + self.output_latency_s) * 1e3
        })
    }
}

fn ld(a: &AtomicU64) -> f64 {
    f64::from_bits(a.load(Ordering::Relaxed))
}
fn st(a: &AtomicU64, v: f64) {
    a.store(v.to_bits(), Ordering::Relaxed)
}

impl BridgeStats {
    /// Snapshot all counters.
    pub fn snapshot(&self) -> StatsSnapshot {
        let corr = ld(&self.correction);
        let dev = ld(&self.device_rate);
        StatsSnapshot {
            state: BridgeState::from_u32(self.state.load(Ordering::Relaxed)),
            fill_frames: ld(&self.fill_frames),
            ring_frames: ld(&self.ring_frames),
            target_frames: ld(&self.target_frames),
            correction_ppm: corr * 1e6,
            drift_ppm: ld(&self.drift) * 1e6,
            ratio: 1.0 / (1.0 + corr),
            underruns: self.underruns.load(Ordering::Relaxed),
            resyncs: self.resyncs.load(Ordering::Relaxed),
            callbacks: self.callbacks.load(Ordering::Relaxed),
            frames_out: self.frames_out.load(Ordering::Relaxed),
            device_rate: dev,
            device_buffer: self.device_buffer.load(Ordering::Relaxed),
            reconnects: self.reconnects.load(Ordering::Relaxed),
            overflow_frames: self.overflow_frames.load(Ordering::Relaxed),
            burst_frames: self.target_burst.load(Ordering::Relaxed),
            resampler_frames: ld(&self.resampler_frames),
            output_latency_s: ld(&self.output_latency),
        }
    }

    /// Count a device (re)connection (device layer).
    pub fn note_reconnect(&self) {
        self.reconnects.fetch_add(1, Ordering::Relaxed);
    }

    /// Record the device's output latency (device layer, at (re)configuration).
    pub fn set_output_latency(&self, latency: Duration) {
        st(&self.output_latency, latency.as_secs_f64());
    }

    /// Whether a consumer is attached to a running device (producers skip pushing otherwise).
    pub fn is_active(&self) -> bool {
        self.active.load(Ordering::Relaxed)
    }
}

/// Create a bridge: the producer goes to the engine, the consumer to the device layer.
pub fn bridge(cfg: BridgeConfig) -> (BridgeProducer, BridgeConsumer) {
    let ring_frames = (cfg.ring_duration.as_secs_f64() * cfg.source_rate).ceil() as usize;
    let (p, c) = ring(cfg.channels, ring_frames.max(8 * cfg.max_block_frames));
    let stats = Arc::new(BridgeStats::default());
    (
        BridgeProducer {
            ring: p,
            stats: stats.clone(),
        },
        BridgeConsumer::new(c, stats, cfg),
    )
}

/// Engine-side handle.
pub struct BridgeProducer {
    ring: RingProducer,
    stats: Arc<BridgeStats>,
}

impl BridgeProducer {
    /// Number of channels.
    pub fn channels(&self) -> usize {
        self.ring.channels()
    }

    /// Push one engine block; `source(i)` returns bridged channel `i` (at least `frames` long).
    /// `now_ns` is the host time (see [`crate::host_time_ns`]) to stamp the push with, or 0 to
    /// leave the stamp to [`end_burst`](Self::end_burst). While no device is running the block
    /// is discarded. RT-safe.
    pub fn push_with<'a>(
        &mut self,
        frames: usize,
        now_ns: u64,
        source: impl FnMut(usize) -> &'a [f32],
    ) {
        if !self.stats.is_active() {
            return;
        }
        if !self.ring.push_with(frames, now_ns, source) {
            self.stats
                .overflow_frames
                .store(self.ring.overflow_frames(), Ordering::Relaxed);
        }
    }

    /// Push a planar block (one slice per channel).
    pub fn push(&mut self, block: &[&[f32]], now_ns: u64) {
        let frames = block.iter().map(|c| c.len()).min().unwrap_or(0);
        self.push_with(frames, now_ns, |i| block[i]);
    }

    /// Mark the end of a burst of blocks (one driver IO cycle): stamps the producer position
    /// with the host time `now_ns`, and reports the nominal burst size `burst_frames` (the
    /// driver IO buffer; 0 = unchanged) from which the consumer derives its target. RT-safe.
    pub fn end_burst(&mut self, now_ns: u64, burst_frames: usize) {
        if burst_frames != 0 {
            self.stats
                .burst_frames
                .store(burst_frames as u32, Ordering::Relaxed);
        }
        if self.stats.is_active() {
            self.ring.stamp(now_ns);
        }
    }

    /// Shared statistics.
    pub fn stats(&self) -> &Arc<BridgeStats> {
        &self.stats
    }
}

/// Fresh producer bursts required before priming completes.
const PRIME_BURSTS: u32 = 3;
/// Producer bursts the fill estimate takes the median over.
const STAMPS: usize = 5;
/// A producer silent for longer than this after the ring ran dry had stopped, not stalled.
const STALL_NS: u64 = 100_000_000;

/// Device-side handle.
pub struct BridgeConsumer {
    ring: RingConsumer,
    stats: Arc<BridgeStats>,
    cfg: BridgeConfig,
    resampler: Resampler,
    control: DriftController,
    device_rate: f64,
    nominal_step: f64,
    buffer_frames: usize,
    burst: usize,
    target: f64,
    state: BridgeState,
    flush: bool,
    last_ns: u64,
    /// Stamp time of the last producer burst seen, and bursts seen while priming.
    last_stamp: u64,
    bursts_seen: u32,
    /// The last [`STAMPS`] burst stamps `(host time ns, write index)`, newest at
    /// `stamp_pos - 1`.
    stamps: [(u64, u64); STAMPS],
    n_stamps: usize,
    stamp_pos: usize,
    /// Host time the ring ran dry while the producer had gone quiet (0 = not pending): an
    /// underrun if the producer resumes soon after (a stall), not if it stopped.
    dry_ns: u64,
}

impl BridgeConsumer {
    fn new(ring: RingConsumer, stats: Arc<BridgeStats>, cfg: BridgeConfig) -> Self {
        let resampler = Resampler::new(cfg.channels, 1.0, cfg.resampler);
        let control = DriftController::new(cfg.source_rate, cfg.drift);
        let mut s = Self {
            ring,
            stats,
            cfg,
            resampler,
            control,
            device_rate: cfg.source_rate,
            nominal_step: 1.0,
            buffer_frames: cfg.max_block_frames,
            burst: cfg.max_block_frames.max(1),
            target: 0.0,
            state: BridgeState::Stopped,
            flush: true,
            last_ns: 0,
            last_stamp: 0,
            bursts_seen: 0,
            stamps: [(0, 0); STAMPS],
            n_stamps: 0,
            stamp_pos: 0,
            dry_ns: 0,
        };
        s.configure(cfg.source_rate, cfg.max_block_frames);
        s
    }

    /// Number of channels.
    pub fn channels(&self) -> usize {
        self.cfg.channels
    }

    /// Shared statistics.
    pub fn stats(&self) -> &Arc<BridgeStats> {
        &self.stats
    }

    /// Bridge configuration.
    pub fn config(&self) -> &BridgeConfig {
        &self.cfg
    }

    /// Fill target in input frames.
    pub fn target_frames(&self) -> f64 {
        self.target
    }

    /// Total added latency in seconds (ring target + resampler delay), excluding the device's
    /// own output latency.
    pub fn latency(&self) -> Duration {
        Duration::from_secs_f64(
            (self.target + self.resampler.latency_frames()) / self.cfg.source_rate,
        )
    }

    fn margin_frames(&self) -> f64 {
        self.cfg.target_latency.as_secs_f64() * self.cfg.source_rate
    }

    fn update_target(&mut self) {
        self.target = self.burst as f64
            + self.buffer_frames as f64 * self.nominal_step
            + self.margin_frames();
        st(&self.stats.target_frames, self.target);
        self.stats
            .target_burst
            .store(self.burst as u32, Ordering::Relaxed);
    }

    /// (Re)configure for a device running at `device_rate` with `buffer_frames` per callback.
    /// Allocates; call while the device is stopped. Keeps the learned drift if the rate is
    /// unchanged.
    pub fn configure(&mut self, device_rate: f64, buffer_frames: usize) {
        let step = self.cfg.source_rate / device_rate;
        if (device_rate - self.device_rate).abs() > 0.5 || self.target == 0.0 {
            self.resampler = Resampler::new(self.cfg.channels, step, self.cfg.resampler);
            self.control.reset();
        }
        self.device_rate = device_rate;
        self.nominal_step = step;
        self.buffer_frames = buffer_frames;
        self.update_target();
        st(
            &self.stats.resampler_frames,
            self.resampler.latency_frames(),
        );
        st(&self.stats.device_rate, device_rate);
        self.stats
            .device_buffer
            .store(buffer_frames as u32, Ordering::Relaxed);
    }

    fn set_state(&mut self, s: BridgeState) {
        self.state = s;
        self.stats.state.store(s as u32, Ordering::Relaxed);
    }

    fn reprime(&mut self) {
        self.set_state(BridgeState::Priming);
        self.bursts_seen = 0;
        self.control.reset_error();
    }

    /// Mark the device as started: the stale ring content is flushed and the bridge primes.
    pub fn start(&mut self) {
        self.flush = true;
        self.resampler.reset();
        self.last_ns = 0;
        self.last_stamp = 0;
        self.n_stamps = 0;
        self.dry_ns = 0;
        self.reprime();
        self.stats.active.store(true, Ordering::Release);
    }

    /// Mark the device as stopped (producers stop pushing).
    pub fn stop(&mut self) {
        self.stats.active.store(false, Ordering::Release);
        self.set_state(BridgeState::Stopped);
    }

    /// Render one device callback of `frames` frames. `now_ns` is the callback's host time.
    /// `write(frame, samples)` receives each produced frame (one sample per bridged channel);
    /// frames not written must be treated as silence by the caller (who pre-zeroes the
    /// buffers). RT-safe: no allocation, locks or syscalls.
    pub fn render(&mut self, frames: usize, now_ns: u64, mut write: impl FnMut(usize, &[f32])) {
        self.stats.callbacks.fetch_add(1, Ordering::Relaxed);
        if self.state == BridgeState::Stopped {
            return;
        }
        let info = self.ring.begin();
        if self.flush {
            self.flush = false;
            self.ring.skip(info.available);
            self.ring.commit();
            return;
        }
        let dt = if self.last_ns == 0 || now_ns <= self.last_ns {
            frames as f64 / self.device_rate
        } else {
            (now_ns - self.last_ns) as f64 * 1e-9
        };
        self.last_ns = now_ns;

        // Producer burst changed (driver buffer size): new target; growing it re-primes, as
        // the slow loop could not add a whole buffer of headroom in time.
        let burst = self.stats.burst_frames.load(Ordering::Relaxed) as usize;
        if burst != 0 && burst != self.burst {
            let old = self.target;
            self.burst = burst;
            self.update_target();
            if self.state == BridgeState::Running && self.target > old + 0.5 * self.margin_frames()
            {
                self.reprime();
            }
        }

        let fs = self.cfg.source_rate;
        let phase = self.resampler.phase();
        let raw = info.available as f64 - phase;
        st(&self.stats.ring_frames, raw);
        // Producer position extrapolated from its latest burst stamp (module docs). Clamped
        // to within two bursts of the raw count so a bad stamp cannot run away.
        let mut fill = raw;
        let mut alive = false;
        if let (Some((t, end)), true) = (info.stamp, now_ns != 0) {
            if t != self.last_stamp {
                self.last_stamp = t;
                self.stamps[self.stamp_pos] = (t, end);
                self.stamp_pos = (self.stamp_pos + 1) % STAMPS;
                self.n_stamps = (self.n_stamps + 1).min(STAMPS);
                self.bursts_seen = self.bursts_seen.saturating_add(1);
                if self.dry_ns != 0 {
                    if t.saturating_sub(self.dry_ns) < STALL_NS {
                        self.stats.underruns.fetch_add(1, Ordering::Relaxed);
                    }
                    self.dry_ns = 0;
                }
            }
            let age = (now_ns as f64 - t as f64) * 1e-9;
            // Stale: no burst for 4 burst periods and 50 ms (DAW stopped its IO).
            alive = age < 4.0 * self.burst as f64 / fs + 0.05;
            if alive {
                // Median over the last few bursts: one late burst (a late daemon wake-up)
                // does not move the estimate; DAW/daemon jitter is reduced.
                let mut pos = [0.0f64; STAMPS];
                let n = self.n_stamps.max(1);
                for (k, p) in pos.iter_mut().enumerate().take(n) {
                    let (ts, e) = if self.n_stamps == 0 {
                        (t, end)
                    } else {
                        self.stamps[k]
                    };
                    *p = e as f64 + (now_ns as f64 - ts as f64) * 1e-9 * fs;
                }
                let pos = &mut pos[..n];
                pos.sort_unstable_by(|a, b| a.total_cmp(b));
                let produced = if n % 2 == 1 {
                    pos[n / 2]
                } else {
                    0.5 * (pos[n / 2 - 1] + pos[n / 2])
                };
                let b = 2.0 * self.burst as f64;
                let model = produced - (info.read_index as f64 + phase);
                fill = model.clamp(raw - b, raw + b);
            }
        }
        st(&self.stats.fill_frames, fill);

        if self.state == BridgeState::Priming {
            if !alive || self.bursts_seen < PRIME_BURSTS || fill < self.target {
                return;
            }
            // Start exactly at the target and play this callback already.
            let excess = (fill - self.target).floor().max(0.0) as u64;
            fill -= self.ring.skip(excess) as f64;
            self.set_state(BridgeState::Running);
        }

        // Way too much queued (e.g. the device stalled): drop back to the target.
        let max_excess =
            self.target.max(2.0 * self.burst as f64) + 2.0 * self.burst as f64 + frames as f64;
        if fill - self.target > max_excess {
            self.ring.skip((fill - self.target).floor() as u64);
            self.stats.resyncs.fetch_add(1, Ordering::Relaxed);
            self.control.reset_error();
            self.ring.commit();
            return;
        }

        let corr = self.control.update(fill - self.target, dt);
        // Report the controller's low-passed fill (the per-callback estimate still carries
        // the producer's timing jitter).
        st(
            &self.stats.fill_frames,
            self.target + self.control.filtered_error(),
        );
        st(&self.stats.correction, corr);
        st(&self.stats.drift, self.control.drift());
        let step = self.nominal_step * (1.0 + corr);
        let need = self.resampler.input_needed(frames, step) + 1;
        if need > self.ring.remaining() {
            // Ran dry. An underrun if the producer is on time (a late burst) or resumes within
            // STALL_NS (a stall); not if it stopped (the DAW stopped its IO).
            let period_ns = self.burst as f64 / fs * 1e9;
            let on_time = info
                .stamp
                .is_some_and(|(t, _)| (now_ns as f64 - t as f64) < 2.0 * period_ns + 2e6);
            if on_time {
                self.stats.underruns.fetch_add(1, Ordering::Relaxed);
            } else {
                self.dry_ns = now_ns.max(1);
            }
            self.reprime();
            return;
        }
        let ring = &mut self.ring;
        for i in 0..frames {
            let out = self.resampler.next_frame(step, |f| {
                ring.frame(f);
            });
            write(i, out);
        }
        self.ring.commit();
        self.stats
            .frames_out
            .fetch_add(frames as u64, Ordering::Relaxed);
        self.stats
            .overflow_frames
            .store(self.ring.overflow_frames(), Ordering::Relaxed);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn primes_then_plays_signal() {
        let (mut p, mut c) = bridge(BridgeConfig {
            channels: 1,
            ..Default::default()
        });
        c.configure(48_000.0, 128);
        c.start();
        assert!(p.stats().is_active());
        let blk = [0.5f32; 64];
        let mut t = 1_000_000u64;
        let mut got = Vec::new();
        for _ in 0..400 {
            // two producer blocks per consumer callback (same nominal rate)
            for _ in 0..2 {
                t += 1_333_333;
                p.push(&[&blk], t);
            }
            c.render(128, t, |_, s| got.push(s[0]));
        }
        let s = p.stats().snapshot();
        assert_eq!(s.state, BridgeState::Running);
        assert_eq!(s.underruns, 0);
        assert!(got.len() > 10_000);
        assert!((got.last().unwrap() - 0.5).abs() < 1e-5);
        assert!((s.fill_frames - s.target_frames).abs() < 64.0, "{s:?}");
    }

    #[test]
    fn inactive_bridge_discards() {
        let (mut p, c) = bridge(BridgeConfig::default());
        let blk = [0.0f32; 64];
        p.push(&[&blk, &blk], 1);
        assert_eq!(p.ring.len(), 0);
        drop(c);
    }

    #[test]
    fn underrun_reprimes() {
        let (mut p, mut c) = bridge(BridgeConfig {
            channels: 1,
            ..Default::default()
        });
        c.configure(48_000.0, 64);
        c.start();
        let blk = [0.1f32; 64];
        let mut t = 1_000_000u64;
        c.render(64, t, |_, _| {}); // flush
        let mut run = |p: &mut BridgeProducer, c: &mut BridgeConsumer, n, every: Option<usize>| {
            for i in 0..n {
                t += 1_333_333;
                if every.is_some_and(|e| i % e == 0) {
                    p.push(&[&blk], t);
                }
                c.render(64, t, |_, _| {});
            }
        };
        run(&mut p, &mut c, 100, Some(1));
        assert_eq!(c.stats().snapshot().state, BridgeState::Running);
        // The producer keeps delivering, but at half speed: a real underrun.
        run(&mut p, &mut c, 400, Some(2));
        assert!(c.stats().snapshot().underruns >= 1);
        // Back to full speed: primes and plays again.
        run(&mut p, &mut c, 100, Some(1));
        assert_eq!(c.stats().snapshot().state, BridgeState::Running);
        let u = c.stats().snapshot().underruns;
        // The producer stops (DAW stopped its IO): drains and re-primes, no underrun counted.
        run(&mut p, &mut c, 300, None);
        let s = c.stats().snapshot();
        assert_eq!(s.underruns, u, "{s:?}");
        assert_eq!(s.state, BridgeState::Priming);
    }

    /// The producer reports a bigger burst (driver buffer 64 -> 512): the target grows and the
    /// bridge re-primes to it instead of underrunning; the latency report follows.
    #[test]
    fn burst_change_retargets() {
        let (mut p, mut c) = bridge(BridgeConfig {
            channels: 1,
            ..Default::default()
        });
        c.configure(48_000.0, 128);
        c.start();
        c.stats().set_output_latency(Duration::from_millis(3));
        let blk = [0.2f32; 64];
        let mut t = 1_000_000_000u64;
        let mut run = |p: &mut BridgeProducer, c: &mut BridgeConsumer, burst: usize, secs: f64| {
            let cycles = (secs * 48_000.0 / burst as f64) as usize;
            for _ in 0..cycles {
                // One burst of `burst` frames, then the callbacks covering that period.
                for _ in 0..burst / 64 {
                    p.push(&[&blk], 0);
                }
                p.end_burst(t, burst);
                for _ in 0..burst / 128 {
                    c.render(128, t, |_, _| {});
                    t += 2_666_667;
                }
            }
        };
        run(&mut p, &mut c, 128, 3.0);
        let s = c.stats().snapshot();
        assert_eq!(s.state, BridgeState::Running);
        assert_eq!(s.target_frames, 128.0 + 128.0 + 240.0);
        run(&mut p, &mut c, 512, 5.0);
        let s = c.stats().snapshot();
        assert_eq!(s.state, BridgeState::Running);
        assert_eq!(s.burst_frames, 512);
        assert_eq!(s.target_frames, 512.0 + 128.0 + 240.0);
        assert_eq!(s.underruns, 0);
        assert!((s.fill_frames - s.target_frames).abs() < 24.0, "{s:?}");
        // (fill - burst + resampler) / fs + 3 ms
        let want = (s.fill_frames - 512.0 + s.resampler_frames) / 48.0 + 3.0;
        assert!((s.latency_ms(48_000.0).unwrap() - want).abs() < 1e-9);
    }

    /// Simulated disconnect / reconnect at a different device rate.
    #[test]
    fn restart_after_disconnect_and_rate_change() {
        let (mut p, mut c) = bridge(BridgeConfig {
            channels: 2,
            ..Default::default()
        });
        let blk = [0.25f32; 64];
        let mut t = 1_000_000_000u64;
        let run =
            |p: &mut BridgeProducer, c: &mut BridgeConsumer, t: &mut u64, rate: f64, secs: f64| {
                let buf = 256usize;
                let mut acc = 0.0f64;
                let mut last = 0.0f32;
                for _ in 0..(secs * rate / buf as f64) as usize {
                    acc += buf as f64 / rate;
                    while acc > 64.0 / 48_000.0 {
                        acc -= 64.0 / 48_000.0;
                        *t += 1_333_333;
                        p.push(&[&blk, &blk], *t);
                    }
                    c.render(buf, *t, |_, s| last = s[1]);
                }
                last
            };
        c.configure(48_000.0, 256);
        c.start();
        assert!((run(&mut p, &mut c, &mut t, 48_000.0, 2.0) - 0.25).abs() < 1e-5);
        // Device vanishes: producer pushes are discarded, nothing accumulates.
        c.stop();
        let before = p.ring.len();
        p.push(&[&blk, &blk], t);
        assert_eq!(p.ring.len(), before);
        // Comes back at 44.1 kHz.
        c.configure(44_100.0, 512);
        c.start();
        assert!((run(&mut p, &mut c, &mut t, 44_100.0, 2.0) - 0.25).abs() < 1e-5);
        let s = c.stats().snapshot();
        assert_eq!(s.state, BridgeState::Running);
        assert_eq!(s.underruns, 0);
        assert_eq!(s.device_rate, 44_100.0);
        assert!((s.fill_frames - s.target_frames).abs() < 64.0, "{s:?}");
    }
}
