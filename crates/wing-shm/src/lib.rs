//! Daemon side of the driver<->daemon shared-memory transport.
//!
//! The Wing Simulator HAL driver (`driver/`, C++) and the `wing-sim` daemon
//! exchange 48x48 channels of f32 audio through the POSIX shm object
//! [`SHM_NAME`]. The layout is defined in `driver/include/wing_shm.h` and
//! mirrored byte-for-byte in [`layout`] (checked at compile time on both sides
//! and by `tests/layout.rs`, which compiles the C header).
//!
//! - [`Segment`]: attach/create/validate the mapping (non-RT), matching the
//!   driver's `ShmLink::Open` semantics.
//! - [`DaemonLink`]: the real-time API. [`DaemonLink::wait`] aligns the daemon
//!   to the driver's IO cycle; [`DaemonLink::next_block`],
//!   [`DaemonLink::read_input`], [`DaemonLink::write_output`] and
//!   [`DaemonLink::commit`] are wait-free (atomics + memcpy, no allocation,
//!   locks or syscalls).
//! - [`wake`]: cross-process wake-up via `os_sync_wait_on_address` (shared),
//!   with nanosleep polling as fallback.
//! - [`sim::DriverSim`]: Rust port of the driver's ring logic for tests.
//!
//! Run the reference daemon with `cargo run -p wing-shm --example gain_client`.

pub mod clock;
mod daemon;
pub mod layout;
mod ring;
mod segment;
pub mod sim;
pub mod wake;

pub use daemon::{
    Block, DaemonLink, Stats, FALLBACK_POLL, MAX_LAG_FRAMES, MISSED_WAKES_BEFORE_FALLBACK,
    RESYNC_BACKLOG_FRAMES,
};
pub use layout::{Header, LayoutError, CHANNELS, RING_FRAMES, SHM_NAME, VERSION};
pub use segment::{Error, OpenOptions, RingStatus, Segment, Status};
pub use wake::{Mechanism, Waiter, WakeMode};

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sim::DriverSim;
    use std::sync::atomic::AtomicUsize;
    use std::sync::atomic::Ordering::{Relaxed, SeqCst};
    use std::time::Duration;

    /// Unique shm name, unlinked on drop.
    struct TmpName(String);
    impl TmpName {
        fn new() -> Self {
            static N: AtomicUsize = AtomicUsize::new(0);
            let n = format!("/wst.{}.{}", std::process::id(), N.fetch_add(1, SeqCst));
            let _ = Segment::unlink(&n);
            Self(n)
        }
    }
    impl Drop for TmpName {
        fn drop(&mut self) {
            let _ = Segment::unlink(&self.0);
        }
    }

    fn create() -> OpenOptions {
        OpenOptions {
            create: true,
            ..Default::default()
        }
    }

    const BUF: usize = 128;
    const GAIN: f32 = -0.5;

    fn sig(c: usize, s: i64) -> f32 {
        ((s * 7 + c as i64 * 131) % 1000) as f32 / 1000.0 - 0.5
    }

    fn daemon_process(link: &mut DaemonLink, gain: f32) -> usize {
        let mut buf = [0.0f32; 256];
        let mut n = 0;
        while let Some(b) = link.next_block(buf.len()) {
            for ch in 0..CHANNELS {
                let x = &mut buf[..b.frames];
                link.read_input(&b, ch, x);
                x.iter_mut().for_each(|v| *v *= gain);
                link.write_output(&b, ch, x);
            }
            link.commit(&b);
            n += b.frames;
        }
        n
    }

    /// One simulated HAL cycle at `now`: ReadInput(now-BUF), WriteMix(now+BUF),
    /// then the daemon runs. Returns (checked, mismatched) input frames.
    fn cycle(
        drv: &DriverSim,
        link: Option<&mut DaemonLink>,
        now: i64,
        first_written: i64,
        expect_gain: f32,
        output: bool,
    ) -> (usize, usize) {
        let mut io = vec![0.0f32; BUF * CHANNELS];
        let delay = drv.segment().header().input_delay_frames as i64;
        let ti = now - BUF as i64;
        drv.read_input(ti, &mut io, BUF, CHANNELS);
        let (mut checked, mut bad) = (0, 0);
        for f in 0..BUF {
            let src = ti + f as i64 - delay;
            if src < first_written {
                continue;
            }
            checked += 1;
            let ok = (0..CHANNELS).all(|c| {
                let want = if output {
                    expect_gain * sig(c, src)
                } else {
                    0.0
                };
                (io[f * CHANNELS + c] - want).abs() < 1e-6
            });
            bad += !ok as usize;
        }
        if output {
            let to = now + BUF as i64;
            for f in 0..BUF {
                for c in 0..CHANNELS {
                    io[f * CHANNELS + c] = sig(c, to + f as i64);
                }
            }
            drv.write_output(to, &io, BUF, CHANNELS);
        }
        if let Some(l) = link {
            l.heartbeat();
            daemon_process(l, GAIN);
        }
        (checked, bad)
    }

    #[test]
    fn open_semantics() {
        let name = TmpName::new();
        assert!(matches!(
            Segment::open(&name.0, &OpenOptions::default()),
            Err(Error::NotFound)
        ));
        let a = Segment::open(&name.0, &create()).unwrap();
        assert!(a.created());
        let b = Segment::open(&name.0, &OpenOptions::default()).unwrap();
        assert!(!b.created());
        assert_eq!(a.header().segment_id, b.header().segment_id);
        assert!(b.is_current());
        assert!(matches!(
            Segment::open("/this-name-is-way-too-long-for-pshm", &create()),
            Err(Error::BadName)
        ));

        // Unlink + recreate (what the driver does on a version change):
        // old mappings detect it.
        Segment::unlink(&name.0).unwrap();
        assert!(!b.is_current());
        let c = Segment::open(&name.0, &create()).unwrap();
        assert!(c.created() && c.is_current() && !a.is_current());
    }

    #[test]
    fn incompatible_version_is_rejected_or_replaced() {
        let name = TmpName::new();
        let old = Segment::open(&name.0, &create()).unwrap();
        // Pretend it is a v1 segment.
        // SAFETY: test-only write to the version field of our own mapping.
        unsafe {
            let p = old
                .base()
                .add(std::mem::offset_of!(Header, version))
                .cast::<u32>();
            p.write_volatile(1);
        }
        assert!(matches!(
            Segment::open(&name.0, &create()),
            Err(Error::Incompatible(LayoutError::Version(1)))
        ));
        let opts = OpenOptions {
            replace_incompatible: true,
            ..Default::default()
        };
        let new = Segment::open(&name.0, &opts).unwrap();
        assert!(new.created());
        assert_eq!(new.header().version, VERSION);
        assert!(!old.is_current());
    }

    #[test]
    fn loopback_then_daemon_gain_roundtrip() {
        let name = TmpName::new();
        let drv = DriverSim::new(Segment::open(&name.0, &create()).unwrap(), 48_000);

        // No daemon: loopback.
        let mut tot = (0, 0);
        for k in 0..100 {
            let r = cycle(&drv, None, k * BUF as i64, BUF as i64, 1.0, true);
            tot = (tot.0 + r.0, tot.1 + r.1);
        }
        assert!(tot.0 > 0 && tot.1 == 0, "{tot:?}");

        // Daemon attached (second mapping of the same name).
        let mut link =
            DaemonLink::attach(&name.0, &OpenOptions::default(), WakeMode::Auto).unwrap();
        assert!(drv.daemon_alive());
        drv.begin_epoch(48_000);
        let mut tot = (0, 0);
        for k in 0..200 {
            let r = cycle(
                &drv,
                Some(&mut link),
                k * BUF as i64,
                BUF as i64,
                GAIN,
                true,
            );
            tot = (tot.0 + r.0, tot.1 + r.1);
        }
        assert!(tot.0 > 0 && tot.1 == 0, "{tot:?}");
        let st = link.stats();
        assert_eq!(
            (st.underruns, st.overruns, st.loopback_active),
            (0, 0, false)
        );

        // IO restart: sample time back to 0, daemon follows.
        drv.begin_epoch(44_100);
        let mut tot = (0, 0);
        for k in 0..100 {
            let r = cycle(
                &drv,
                Some(&mut link),
                k * BUF as i64,
                BUF as i64,
                GAIN,
                true,
            );
            tot = (tot.0 + r.0, tot.1 + r.1);
        }
        assert!(tot.0 > 0 && tot.1 == 0, "{tot:?}");
        assert_eq!(link.stats().sample_rate, 44_100);
        assert_eq!(link.stats().underruns, 0);

        // Detach -> loopback immediately.
        drop(link);
        assert!(!drv.daemon_alive());
    }

    #[test]
    fn record_only_clocks_from_input_time() {
        let name = TmpName::new();
        let drv = DriverSim::new(Segment::open(&name.0, &create()).unwrap(), 48_000);
        let mut link =
            DaemonLink::attach(&name.0, &OpenOptions::default(), WakeMode::Auto).unwrap();
        drv.begin_epoch(48_000);
        let mut tot = (0, 0);
        for k in 0..200 {
            let r = cycle(&drv, Some(&mut link), k * BUF as i64, 0, GAIN, false);
            tot = (tot.0 + r.0, tot.1 + r.1);
        }
        assert!(tot.0 > 0 && tot.1 == 0, "{tot:?}");
        // Only the very first read (before the daemon ever ran) may underrun.
        assert!(link.stats().underruns <= 1, "{:?}", link.stats());
        assert_eq!(link.stats().overruns, 0);
    }

    #[test]
    fn overrun_skips_ahead() {
        let name = TmpName::new();
        let drv = DriverSim::new(Segment::open(&name.0, &create()).unwrap(), 48_000);
        let mut link =
            DaemonLink::attach(&name.0, &OpenOptions::default(), WakeMode::Auto).unwrap();
        assert!(link.next_block(64).is_none());
        let silence = vec![0.0f32; BUF * CHANNELS];
        for k in 0..64 {
            drv.write_output(k * BUF as i64, &silence, BUF, CHANNELS);
        }
        let b = link.next_block(1 << 20).unwrap();
        assert_eq!(link.stats().overruns, 1);
        assert_eq!(b.end(), 64 * BUF as i64);
        assert_eq!(b.frames as i64, RESYNC_BACKLOG_FRAMES);
        link.commit(&b);
        assert_eq!(
            link.header().from_daemon.write_time.load(Relaxed),
            64 * BUF as i64
        );
        assert!(link.next_block(64).is_none());
    }

    #[test]
    fn wait_is_woken_by_driver_cycle() {
        let name = TmpName::new();
        let drv = DriverSim::new(Segment::open(&name.0, &create()).unwrap(), 48_000);
        let mut link =
            DaemonLink::attach(&name.0, &OpenOptions::default(), WakeMode::Auto).unwrap();
        assert!(!link.wait(Duration::from_millis(5)));
        let t = std::thread::spawn(move || {
            std::thread::sleep(Duration::from_millis(5));
            drv.write_output(0, &[0.0; CHANNELS * 16], 16, CHANNELS);
            drv
        });
        assert!(link.wait(Duration::from_secs(2)));
        let _drv = t.join().unwrap();
        assert_eq!(link.next_block(64).map(|b| b.frames), Some(16));
    }

    #[test]
    fn falls_back_to_polling_when_wakes_are_not_delivered() {
        let name = TmpName::new();
        let seg = Segment::open(&name.0, &create()).unwrap();
        let mut link =
            DaemonLink::attach(&name.0, &OpenOptions::default(), WakeMode::Auto).unwrap();
        if link.mechanism() != Mechanism::OsSync {
            return;
        }
        // A "driver" that bumps cycle_seq but never calls the wake.
        for _ in 0..MISSED_WAKES_BEFORE_FALLBACK {
            assert_eq!(link.mechanism(), Mechanism::OsSync);
            let seq = &seg.header().cycle_seq;
            std::thread::scope(|s| {
                s.spawn(|| {
                    std::thread::sleep(Duration::from_millis(1));
                    seq.fetch_add(1, SeqCst);
                });
                assert!(link.wait(Duration::from_millis(5)));
            });
        }
        assert_eq!(link.mechanism(), Mechanism::Poll);
    }
}
