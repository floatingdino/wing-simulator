//! Rust port of the driver's `ShmLink` real-time logic, for tests, benches
//! and the wake-latency measurement without the C++ driver. Behaviour must
//! match `driver/src/ShmLink.cpp`; the cross-process test
//! (`tests/cpp_selftest.rs`) exercises the real C++ code against
//! [`crate::DaemonLink`].

use std::sync::atomic::Ordering::{Acquire, Relaxed, Release};

use crate::clock;
use crate::layout::{
    RingCtl, CHANNELS, FROM_DAEMON_OFFSET, HEARTBEAT_TIMEOUT_NS, RING_FRAMES, RING_GUARD_FRAMES,
    TO_DAEMON_OFFSET,
};
use crate::ring::Ring;
use crate::segment::Segment;
use crate::wake;

/// Driver role on a [`Segment`].
pub struct DriverSim {
    seg: Segment,
    to: Ring,
    from: Ring,
    timeout_ticks: u64,
}

impl DriverSim {
    /// Take the driver role on `seg` and start epoch 1.
    pub fn new(seg: Segment, sample_rate: u32) -> Self {
        // SAFETY: fixed layout offsets inside the segment.
        let (to, from) = unsafe {
            (
                Ring::new(seg.base().add(TO_DAEMON_OFFSET)),
                Ring::new(seg.base().add(FROM_DAEMON_OFFSET)),
            )
        };
        let sim = Self {
            seg,
            to,
            from,
            timeout_ticks: clock::ns_to_ticks(HEARTBEAT_TIMEOUT_NS),
        };
        sim.seg
            .header()
            .driver_pid
            .store(std::process::id() as u64, Release);
        sim.begin_epoch(sample_rate);
        sim
    }

    /// The segment.
    pub fn segment(&self) -> &Segment {
        &self.seg
    }

    /// `ShmLink::BeginEpoch`.
    pub fn begin_epoch(&self, sample_rate: u32) {
        let h = self.seg.header();
        h.sample_rate.store(sample_rate, Relaxed);
        h.to_daemon.write_time.store(0, Relaxed);
        h.input_time.store(0, Relaxed);
        let next = h.io_epoch.load(Relaxed).wrapping_add(1);
        h.to_daemon.epoch.store(next, Release);
        h.io_epoch.store(next, Release);
        self.signal();
    }

    /// `ShmLink::SetRunning`.
    pub fn set_running(&self, running: bool) {
        self.seg.header().io_running.store(running as u32, Release);
    }

    fn signal(&self) {
        let h = self.seg.header();
        wake::signal(&h.cycle_seq, &h.daemon_waiting);
    }

    /// `ShmLink::DaemonAlive`.
    pub fn daemon_alive(&self) -> bool {
        let hb = self.seg.header().daemon_heartbeat.load(Acquire);
        if hb == 0 {
            return false;
        }
        let now = clock::now_ticks();
        now < hb || now - hb < self.timeout_ticks
    }

    /// `ShmLink::WriteOutput`: DAW output (interleaved) for `[t, t+frames)`.
    pub fn write_output(&self, t: i64, interleaved: &[f32], frames: usize, channels: usize) {
        let h = self.seg.header();
        let chans = channels.min(CHANNELS);
        for f in 0..frames {
            for c in 0..chans {
                self.to.set(c, t + f as i64, interleaved[f * channels + c]);
            }
        }
        let end = t + frames as i64;
        if end > h.to_daemon.write_time.load(Relaxed) {
            h.to_daemon.write_time.store(end, Release);
        }
        h.driver_heartbeat.store(clock::now_ticks(), Relaxed);
        self.signal();
    }

    fn copy_from_ring(
        ring: &Ring,
        ctl: &RingCtl,
        t: i64,
        out: &mut [f32],
        frames: usize,
        channels: usize,
    ) -> bool {
        let wt = ctl.write_time.load(Acquire);
        let oldest = wt - RING_FRAMES as i64 + RING_GUARD_FRAMES as i64;
        let chans = channels.min(CHANNELS);
        let mut complete = true;
        for f in 0..frames {
            let s = t + f as i64;
            let dst = &mut out[f * channels..(f + 1) * channels];
            if s < 0 || s >= wt || s < oldest {
                dst.fill(0.0);
                complete = false;
                continue;
            }
            for (c, d) in dst.iter_mut().enumerate() {
                *d = if c < chans { ring.get(c, s) } else { 0.0 };
            }
        }
        complete
    }

    /// `ShmLink::ReadInput`: DAW input (interleaved) for `[t, t+frames)`.
    pub fn read_input(&self, t: i64, out: &mut [f32], frames: usize, channels: usize) {
        let h = self.seg.header();
        let delay = h.input_delay_frames as i64;
        let src = t - delay;
        let epoch = h.io_epoch.load(Acquire);
        let end = t + frames as i64;
        h.input_frames.store(frames as u32, Relaxed);
        h.input_time.store(end, Release);
        let out = &mut out[..frames * channels];

        if self.daemon_alive() {
            h.loopback_active.store(0, Relaxed);
            let same_epoch = h.from_daemon.epoch.load(Acquire) == epoch;
            if !same_epoch
                || !Self::copy_from_ring(&self.from, &h.from_daemon, src, out, frames, channels)
            {
                if !same_epoch {
                    out.fill(0.0);
                }
                if src >= 0 {
                    h.from_daemon.underruns.fetch_add(1, Relaxed);
                }
            }
            h.from_daemon.read_time.store(src + frames as i64, Relaxed);
            if h.to_daemon.write_time.load(Relaxed) < end {
                self.signal();
            }
        } else {
            h.loopback_active.store(1, Relaxed);
            Self::copy_from_ring(&self.to, &h.to_daemon, src, out, frames, channels);
        }
        h.driver_heartbeat.store(clock::now_ticks(), Relaxed);
    }
}
