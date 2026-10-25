//! One bridge = ring + resampler + drift controller + stats, independent of CoreAudio.
//!
//! * [`BridgeProducer`] (engine thread): pushes a block of the mapped simulator channels per
//!   engine cycle.
//! * [`BridgeConsumer`] (device IO thread): [`BridgeConsumer::render`] pulls exactly what one
//!   device callback needs through the resampler, steering the ratio so the ring fill stays at
//!   the target latency.
//! * [`BridgeStats`]: lock-free counters/gauges readable from any thread.
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
    /// Largest burst the producer delivers at once (one engine block, or several when the
    /// engine runs cycles back to back per driver IO cycle). Part of the latency budget.
    pub max_block_frames: usize,
    /// Extra latency held in the ring on top of one device buffer and one producer burst.
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
    target_frames: AtomicU64, // f64 bits
    correction: AtomicU64,    // f64 bits
    underruns: AtomicU64,
    resyncs: AtomicU64,
    callbacks: AtomicU64,
    frames_out: AtomicU64,
    device_rate: AtomicU64, // f64 bits
    device_buffer: AtomicU32,
    reconnects: AtomicU64,
    overflow_frames: AtomicU64,
}

/// Point-in-time copy of [`BridgeStats`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct StatsSnapshot {
    /// Consumer state.
    pub state: BridgeState,
    /// Estimated ring fill (input frames), measured at the last callback.
    pub fill_frames: f64,
    /// Fill target (input frames).
    pub target_frames: f64,
    /// Ratio correction in ppm (positive = consuming faster than nominal).
    pub correction_ppm: f64,
    /// Drift ratio applied on top of the nominal device/source rate ratio
    /// (`1 / (1 + correction)`; < 1 means the device clock runs fast).
    pub ratio: f64,
    /// Callbacks that found too little data (output silence and re-primed).
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
}

impl StatsSnapshot {
    /// Fill expressed in milliseconds of source audio.
    pub fn fill_ms(&self, source_rate: f64) -> f64 {
        self.fill_frames / source_rate * 1e3
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
            target_frames: ld(&self.target_frames),
            correction_ppm: corr * 1e6,
            ratio: 1.0 / (1.0 + corr),
            underruns: self.underruns.load(Ordering::Relaxed),
            resyncs: self.resyncs.load(Ordering::Relaxed),
            callbacks: self.callbacks.load(Ordering::Relaxed),
            frames_out: self.frames_out.load(Ordering::Relaxed),
            device_rate: dev,
            device_buffer: self.device_buffer.load(Ordering::Relaxed),
            reconnects: self.reconnects.load(Ordering::Relaxed),
            overflow_frames: self.overflow_frames.load(Ordering::Relaxed),
        }
    }

    /// Count a device (re)connection (device layer).
    pub fn note_reconnect(&self) {
        self.reconnects.fetch_add(1, Ordering::Relaxed);
    }

    /// Whether a consumer is attached to a running device (producers skip pushing otherwise).
    pub fn is_active(&self) -> bool {
        self.active.load(Ordering::Relaxed)
    }
}

/// Create a bridge: the producer goes to the engine, the consumer to the device layer.
pub fn bridge(cfg: BridgeConfig) -> (BridgeProducer, BridgeConsumer) {
    let ring_frames = (cfg.ring_duration.as_secs_f64() * cfg.source_rate).ceil() as usize;
    let (p, c) = ring(cfg.channels, ring_frames.max(4 * cfg.max_block_frames));
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
    /// `now_ns` is the host time (see [`crate::host_time_ns`]). While no device is running the
    /// block is discarded. RT-safe.
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

    /// Shared statistics.
    pub fn stats(&self) -> &Arc<BridgeStats> {
        &self.stats
    }
}

/// Device-side handle.
pub struct BridgeConsumer {
    ring: RingConsumer,
    stats: Arc<BridgeStats>,
    cfg: BridgeConfig,
    resampler: Resampler,
    control: DriftController,
    device_rate: f64,
    nominal_step: f64,
    target: f64,
    state: BridgeState,
    flush: bool,
    last_ns: u64,
    /// Low-passed producer timing offset (s), see `render`.
    offset: Option<f64>,
}

/// Time constants of the producer timing-offset envelope (following earlier / later samples).
const OFFSET_DOWN_S: f64 = 0.1;
const OFFSET_UP_S: f64 = 10.0;

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
            target: 0.0,
            state: BridgeState::Stopped,
            flush: true,
            last_ns: 0,
            offset: None,
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
        self.target = buffer_frames as f64 * step
            + self.cfg.max_block_frames as f64
            + self.cfg.target_latency.as_secs_f64() * self.cfg.source_rate;
        st(&self.stats.target_frames, self.target);
        st(&self.stats.device_rate, device_rate);
        self.stats
            .device_buffer
            .store(buffer_frames as u32, Ordering::Relaxed);
    }

    /// Mark the device as started: the stale ring content is flushed and the bridge primes.
    pub fn start(&mut self) {
        self.state = BridgeState::Priming;
        self.flush = true;
        self.resampler.reset();
        self.control.reset_error();
        self.last_ns = 0;
        self.offset = None;
        self.stats
            .state
            .store(BridgeState::Priming as u32, Ordering::Relaxed);
        self.stats.active.store(true, Ordering::Release);
    }

    /// Mark the device as stopped (producers stop pushing).
    pub fn stop(&mut self) {
        self.state = BridgeState::Stopped;
        self.stats.active.store(false, Ordering::Release);
        self.stats
            .state
            .store(BridgeState::Stopped as u32, Ordering::Relaxed);
    }

    /// Render one device callback of `frames` frames. `now_ns` is the callback's host time.
    /// `write(frame, samples)` receives each produced frame (one sample per bridged channel);
    /// frames not written must be treated as silence by the caller (who pre-zeroes the
    /// buffers). RT-safe: no allocation, locks or syscalls.
    pub fn render(&mut self, frames: usize, now_ns: u64, mut write: impl FnMut(usize, &[f32])) {
        let stats = &*self.stats;
        stats.callbacks.fetch_add(1, Ordering::Relaxed);
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
        // Fill estimate. The raw count is a sawtooth (blocks/bursts arrive at once, with
        // scheduling jitter) that would modulate the ratio. Instead model the producer as a
        // line: `offset = push_time - write_index / fs` is constant for an ideal producer, so
        // its lower envelope gives the producer position at `now` without jitter. Clamped
        // to within two bursts of the raw count so a stalled producer cannot run away.
        let fs = self.cfg.source_rate;
        let phase = self.resampler.phase();
        let raw = info.available as f64 - phase;
        let mut fill = raw;
        if let (Some((t, end)), true) = (info.stamp, now_ns != 0) {
            let sample = t as f64 * 1e-9 - end as f64 / fs;
            let off = match self.offset {
                // Lower-envelope tracker: pushes are only ever late (wake-up jitter, or a
                // stamp from the middle of a burst), so follow earlier samples quickly and
                // later ones slowly (that still tracks producer/host clock drift).
                Some(o) => {
                    let tau = if sample < o {
                        OFFSET_DOWN_S
                    } else {
                        OFFSET_UP_S
                    };
                    o + (1.0 - (-dt / tau).exp()) * (sample - o)
                }
                None => sample,
            };
            self.offset = Some(off);
            let produced = (now_ns as f64 * 1e-9 - off) * fs;
            let b = 2.0 * self.cfg.max_block_frames as f64;
            fill = (produced - (info.read_index as f64 + phase)).clamp(raw - b, raw + b);
        }
        st(&stats.fill_frames, fill);

        if self.state == BridgeState::Priming {
            if fill < self.target {
                return;
            }
            // Start exactly at the target and play this callback already.
            let excess = (fill - self.target).floor().max(0.0) as u64;
            fill -= self.ring.skip(excess) as f64;
            self.state = BridgeState::Running;
            stats
                .state
                .store(BridgeState::Running as u32, Ordering::Relaxed);
        }

        // Way too much queued (e.g. the device stalled): drop back to the target.
        let max_excess = self.target.max(2.0 * self.cfg.max_block_frames as f64)
            + 2.0 * self.cfg.max_block_frames as f64
            + frames as f64;
        if fill - self.target > max_excess {
            self.ring.skip((fill - self.target).floor() as u64);
            stats.resyncs.fetch_add(1, Ordering::Relaxed);
            self.control.reset_error();
            self.ring.commit();
            return;
        }

        let corr = self.control.update(fill - self.target, dt);
        st(&stats.correction, corr);
        let step = self.nominal_step * (1.0 + corr);
        let need = self.resampler.input_needed(frames, step) + 1;
        if need > self.ring.remaining() {
            stats.underruns.fetch_add(1, Ordering::Relaxed);
            self.state = BridgeState::Priming;
            self.control.reset_error();
            stats
                .state
                .store(BridgeState::Priming as u32, Ordering::Relaxed);
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
        stats.frames_out.fetch_add(frames as u64, Ordering::Relaxed);
        stats
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
        let mut t = 1u64;
        c.render(64, t, |_, _| {}); // flush
        for _ in 0..20 {
            p.push(&[&blk], t);
            t += 1_333_333;
        }
        for _ in 0..30 {
            c.render(64, t, |_, _| {});
            t += 1_333_333;
        }
        let s = c.stats().snapshot();
        assert_eq!(s.underruns, 1);
        assert_eq!(s.state, BridgeState::Priming);
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
