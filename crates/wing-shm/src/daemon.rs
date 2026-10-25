//! Daemon side of the transport: the real-time read/write API.
//!
//! Typical RT loop (one thread, no allocation, no locks):
//!
//! ```no_run
//! # use wing_shm::{DaemonLink, OpenOptions, WakeMode, CHANNELS};
//! # use std::time::Duration;
//! let mut link = DaemonLink::attach(wing_shm::SHM_NAME, &OpenOptions::default(), WakeMode::Auto)?;
//! let mut buf = vec![0.0f32; 512];
//! loop {
//!     link.heartbeat();
//!     link.wait(Duration::from_millis(10));
//!     while let Some(block) = link.next_block(buf.len()) {
//!         for ch in 0..CHANNELS {
//!             let b = &mut buf[..block.frames];
//!             link.read_input(&block, ch, b);
//!             for s in b.iter_mut() { *s *= 0.5; }
//!             link.write_output(&block, ch, b);
//!         }
//!         link.commit(&block);
//!     }
//! }
//! # Ok::<(), wing_shm::Error>(())
//! ```

use std::sync::atomic::Ordering::{Acquire, Relaxed, Release};
use std::time::Duration;

use crate::clock;
use crate::layout::{Header, FROM_DAEMON_OFFSET, RING_FRAMES, TO_DAEMON_OFFSET};
use crate::ring::Ring;
use crate::segment::{Error, OpenOptions, Segment};
use crate::wake::{Mechanism, Waiter, WakeMode};

/// If the daemon falls further behind the driver than this, it skips ahead
/// (counted in `to_daemon.overruns`). Must stay below
/// `RING_FRAMES - RING_GUARD_FRAMES`.
pub const MAX_LAG_FRAMES: i64 = RING_FRAMES as i64 / 2;
/// Backlog kept when skipping ahead after an overrun.
pub const RESYNC_BACKLOG_FRAMES: i64 = 512;
/// Consecutive undelivered os_sync wakes before falling back to polling.
pub const MISSED_WAKES_BEFORE_FALLBACK: u32 = 3;
/// Poll interval used after that fallback.
pub const FALLBACK_POLL: Duration = Duration::from_micros(100);

/// A span of device sample time the daemon must produce output for.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Block {
    /// Driver IO epoch this block belongs to.
    pub epoch: u32,
    /// First device sample time.
    pub start: i64,
    /// Number of frames.
    pub frames: usize,
    /// Leading frames that have real driver output (`to_daemon`) behind them;
    /// the rest are record-only frames whose input is silence.
    pub input_frames: usize,
}

impl Block {
    /// Exclusive end sample time.
    pub fn end(&self) -> i64 {
        self.start + self.frames as i64
    }
}

/// Counters for monitoring (relaxed snapshot).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Stats {
    /// Current driver IO epoch.
    pub epoch: u32,
    /// Nominal sample rate published by the driver.
    pub sample_rate: u32,
    /// Times the daemon fell more than [`MAX_LAG_FRAMES`] behind.
    pub overruns: u32,
    /// Times the driver read `from_daemon` data that was not ready.
    pub underruns: u32,
    /// Driver currently in loopback (daemon considered dead).
    pub loopback_active: bool,
    /// Daemon-side cursor (next sample time to produce).
    pub cursor: i64,
}

/// The daemon's handle on the segment. Create and drop it off the RT thread;
/// all other methods are real-time safe (no allocation, locks or syscalls,
/// except the deliberate blocking wait in [`DaemonLink::wait`]).
pub struct DaemonLink {
    seg: Segment,
    to: Ring,
    from: Ring,
    waiter: Waiter,
    epoch: Option<u32>,
    cursor: i64,
    seen_seq: u32,
    input_delay: i64,
    missed_wakes: u32,
}

impl DaemonLink {
    /// Open (see [`Segment::open`]) and attach as the daemon: publishes our
    /// pid and a first heartbeat, so the driver leaves loopback.
    pub fn attach(name: &str, opts: &OpenOptions, wake: WakeMode) -> Result<Self, Error> {
        Ok(Self::from_segment(Segment::open(name, opts)?, wake))
    }

    /// Attach to an already mapped segment.
    pub fn from_segment(seg: Segment, wake: WakeMode) -> Self {
        // SAFETY: offsets validated by Segment (fixed layout).
        let (to, from) = unsafe {
            (
                Ring::new(seg.base().add(TO_DAEMON_OFFSET)),
                Ring::new(seg.base().add(FROM_DAEMON_OFFSET)),
            )
        };
        let input_delay = seg.header().input_delay_frames as i64;
        let seen_seq = seg.header().cycle_seq.load(Acquire);
        let link = Self {
            seg,
            to,
            from,
            waiter: Waiter::new(wake),
            epoch: None,
            cursor: 0,
            seen_seq,
            input_delay,
            missed_wakes: 0,
        };
        let h = link.header();
        h.daemon_pid.store(std::process::id() as u64, Release);
        link.heartbeat();
        link
    }

    /// The header.
    #[inline]
    pub fn header(&self) -> &Header {
        self.seg.header()
    }

    /// The mapped segment.
    pub fn segment(&self) -> &Segment {
        &self.seg
    }

    /// Wake-up mechanism in use.
    pub fn mechanism(&self) -> Mechanism {
        self.waiter.mechanism()
    }

    /// Driver's input delay (the daemon's processing window beyond one buffer).
    pub fn input_delay_frames(&self) -> u32 {
        self.input_delay as u32
    }

    /// Publish liveness. Call at least every few ms (every cycle is fine).
    #[inline]
    pub fn heartbeat(&self) {
        self.header()
            .daemon_heartbeat
            .store(clock::now_ticks(), Release);
    }

    /// True if the driver ran an IO cycle within `max_age`.
    pub fn driver_alive(&self, max_age: Duration) -> bool {
        let hb = self.header().driver_heartbeat.load(Relaxed);
        hb != 0
            && clock::ticks_to_ns(clock::now_ticks().saturating_sub(hb)) < max_age.as_nanos() as u64
    }

    /// Block until the driver signals a new cycle (or epoch), or `timeout`.
    /// Returns true if woken by the driver. Aligns the daemon to the driver's
    /// IO cycle: call it at the top of each loop iteration.
    ///
    /// If os_sync waits keep running to their timeout although the driver
    /// did bump `cycle_seq` (wake-ups not delivered, e.g. a future sandbox
    /// change), the link switches to polling every [`FALLBACK_POLL`] after
    /// [`MISSED_WAKES_BEFORE_FALLBACK`] consecutive misses.
    #[inline]
    pub fn wait(&mut self, timeout: Duration) -> bool {
        let h = self.seg.header();
        let t0 = clock::now_ticks();
        let woke = self
            .waiter
            .wait(&h.cycle_seq, &h.daemon_waiting, self.seen_seq, timeout);
        self.seen_seq = h.cycle_seq.load(Acquire);
        if woke && self.waiter.mechanism() == Mechanism::OsSync {
            let waited = clock::ticks_to_ns(clock::now_ticks() - t0);
            if waited >= timeout.as_nanos() as u64 {
                self.missed_wakes += 1;
                if self.missed_wakes >= MISSED_WAKES_BEFORE_FALLBACK {
                    self.waiter = Waiter::new(WakeMode::Poll {
                        interval: FALLBACK_POLL,
                    });
                }
            } else {
                self.missed_wakes = 0;
            }
        }
        woke
    }

    /// Next span to process, at most `max_frames` long, or `None` if the
    /// daemon is caught up. Handles epoch changes (sample-time restarts),
    /// record-only clocking and overruns.
    pub fn next_block(&mut self, max_frames: usize) -> Option<Block> {
        let h = self.seg.header();
        let e = h.io_epoch.load(Acquire);
        let w = h.to_daemon.write_time.load(Acquire);
        let it = h.input_time.load(Acquire);
        let record_only = it > w;

        if self.epoch != Some(e) {
            // New driver IO session (or first attach): sample time restarted.
            // Keep at most RESYNC_BACKLOG_FRAMES of what the driver already
            // wrote in this epoch, skip anything older.
            self.epoch = Some(e);
            self.cursor = if record_only {
                w.max(it - self.input_delay)
            } else {
                (w - RESYNC_BACKLOG_FRAMES).max(0)
            };
            h.from_daemon.write_time.store(self.cursor, Release);
            h.from_daemon.epoch.store(e, Release);
        }

        let target = if record_only {
            let nf = h.input_frames.load(Relaxed) as i64;
            w.max(it - self.input_delay + 2 * nf)
        } else {
            w
        };
        if target - self.cursor > MAX_LAG_FRAMES {
            h.to_daemon.overruns.fetch_add(1, Relaxed);
            self.cursor = target - RESYNC_BACKLOG_FRAMES;
        }
        if target <= self.cursor || max_frames == 0 {
            return None;
        }
        let frames = ((target - self.cursor) as usize).min(max_frames);
        let input_frames = (w - self.cursor).clamp(0, frames as i64) as usize;
        Some(Block {
            epoch: e,
            start: self.cursor,
            frames,
            input_frames,
        })
    }

    /// Copy channel `ch` of the driver's output (WING "USB in") for `block`
    /// into `dst` (`dst.len()` frames, at most `block.frames`); record-only
    /// frames read as silence.
    #[inline]
    pub fn read_input(&self, block: &Block, ch: usize, dst: &mut [f32]) {
        let n = dst.len().min(block.frames);
        let real = block.input_frames.min(n);
        self.to.read(ch, block.start, &mut dst[..real]);
        dst[real..n].fill(0.0);
    }

    /// Write channel `ch` of the daemon's output (WING "USB out", what the
    /// DAW records) for `block` from `src` (at most `block.frames`).
    #[inline]
    pub fn write_output(&self, block: &Block, ch: usize, src: &[f32]) {
        let n = src.len().min(block.frames);
        self.from.write(ch, block.start, &src[..n]);
    }

    /// Publish `block` (all channels written) to the driver.
    #[inline]
    pub fn commit(&mut self, block: &Block) {
        if self.epoch != Some(block.epoch) || block.start != self.cursor {
            return; // stale block (epoch changed or resynced meanwhile)
        }
        let h = self.seg.header();
        self.cursor = block.end();
        h.from_daemon.write_time.store(self.cursor, Release);
        h.to_daemon
            .read_time
            .store(block.start + block.input_frames as i64, Relaxed);
    }

    /// Relaxed snapshot of the counters.
    pub fn stats(&self) -> Stats {
        let h = self.header();
        Stats {
            epoch: h.io_epoch.load(Relaxed),
            sample_rate: h.sample_rate.load(Relaxed),
            overruns: h.to_daemon.overruns.load(Relaxed),
            underruns: h.from_daemon.underruns.load(Relaxed),
            loopback_active: h.loopback_active.load(Relaxed) != 0,
            cursor: self.cursor,
        }
    }
}

impl Drop for DaemonLink {
    /// Detach: zero heartbeat so the driver switches to loopback immediately.
    fn drop(&mut self) {
        let h = self.header();
        h.daemon_heartbeat.store(0, Release);
        h.daemon_pid.store(0, Release);
    }
}
