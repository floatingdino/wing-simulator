//! Cross-process wake-up on `cycle_seq`.
//!
//! The coreaudiod sandbox denies POSIX named semaphores (`sem_open`), and Mach
//! services need launchd registration. `os_sync_wait_on_address` /
//! `os_sync_wake_by_address_all` with the `SHARED` flags (macOS 14.4+, the
//! public face of `__ulock_wait2`) work on any shared-memory address and are
//! not sandbox operations, so they are the primary mechanism. The symbols are
//! resolved with `dlsym` at setup time; when absent the waiter polls
//! `cycle_seq` with `nanosleep`.
//!
//! Protocol (Dekker-style, all seq_cst):
//! - driver: `cycle_seq += 1`; `if daemon_waiting != 0 { wake_all(&cycle_seq) }`
//! - daemon: `daemon_waiting += 1`; `if cycle_seq == seen { wait(&cycle_seq, seen) }`;
//!   `daemon_waiting -= 1`
//!
//! The kernel compares `*addr == seen` atomically before sleeping, so a bump
//! between the re-check and the wait cannot be lost; the timeout bounds any
//! other failure.

use std::ffi::c_void;
use std::sync::atomic::AtomicU32;
use std::sync::atomic::Ordering::SeqCst;
#[cfg(unix)]
use std::sync::OnceLock;
use std::time::Duration;

use crate::clock;

type WaitFn = unsafe extern "C" fn(*mut c_void, u64, usize, u32, u32, u64) -> libc::c_int;
type WakeFn = unsafe extern "C" fn(*mut c_void, usize, u32) -> libc::c_int;

const OS_SYNC_WAIT_ON_ADDRESS_SHARED: u32 = 1;
const OS_SYNC_WAKE_BY_ADDRESS_SHARED: u32 = 1;
const OS_CLOCK_MACH_ABSOLUTE_TIME: u32 = 32;

#[derive(Clone, Copy)]
struct OsSync {
    wait: WaitFn,
    wake: WakeFn,
}

#[cfg(not(unix))]
fn os_sync() -> Option<OsSync> {
    None
}

#[cfg(unix)]
fn os_sync() -> Option<OsSync> {
    static SYMS: OnceLock<Option<OsSync>> = OnceLock::new();
    *SYMS.get_or_init(|| {
        // SAFETY: dlsym with static NUL-terminated names; the transmutes match
        // the prototypes in <os/os_sync_wait_on_address.h>.
        unsafe {
            let w = libc::dlsym(
                libc::RTLD_DEFAULT,
                c"os_sync_wait_on_address_with_timeout".as_ptr(),
            );
            let k = libc::dlsym(libc::RTLD_DEFAULT, c"os_sync_wake_by_address_all".as_ptr());
            if w.is_null() || k.is_null() {
                return None;
            }
            Some(OsSync {
                wait: std::mem::transmute::<*mut c_void, WaitFn>(w),
                wake: std::mem::transmute::<*mut c_void, WakeFn>(k),
            })
        }
    })
}

/// True if this OS provides `os_sync_wait_on_address` (macOS 14.4+).
pub fn os_sync_available() -> bool {
    os_sync().is_some()
}

/// How the daemon waits for the driver's next IO cycle.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WakeMode {
    /// `os_sync_wait_on_address` if available, else [`WakeMode::Poll`] at 100 us.
    Auto,
    /// Block in `os_sync_wait_on_address` (falls back to polling if absent).
    OsSync {
        /// Busy-spin this long before blocking (0 = never spin).
        spin: Duration,
    },
    /// Re-check `cycle_seq` every `interval` with `nanosleep`.
    Poll {
        /// Sleep between checks.
        interval: Duration,
    },
}

/// Which mechanism a [`Waiter`] actually uses.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Mechanism {
    /// os_sync_wait_on_address (shared).
    OsSync,
    /// nanosleep polling.
    Poll,
}

/// Waits for `cycle_seq` to change. Construct off the RT thread.
#[derive(Clone, Copy)]
pub struct Waiter {
    os: Option<OsSync>,
    spin_ticks: u64,
    poll: Duration,
}

impl Waiter {
    /// Resolve the mechanism for `mode` (may call dlsym; not RT).
    pub fn new(mode: WakeMode) -> Self {
        clock::init();
        let default_poll = Duration::from_micros(100);
        match mode {
            WakeMode::Auto => Self {
                os: os_sync(),
                spin_ticks: 0,
                poll: default_poll,
            },
            WakeMode::OsSync { spin } => Self {
                os: os_sync(),
                spin_ticks: clock::ns_to_ticks(spin.as_nanos() as u64),
                poll: default_poll,
            },
            WakeMode::Poll { interval } => Self {
                os: None,
                spin_ticks: 0,
                poll: interval,
            },
        }
    }

    /// The mechanism in use.
    pub fn mechanism(&self) -> Mechanism {
        if self.os.is_some() {
            Mechanism::OsSync
        } else {
            Mechanism::Poll
        }
    }

    /// Block until `seq != seen` or `timeout` elapses. Returns true if `seq`
    /// changed. Allocation- and lock-free; blocks only in the kernel wait (or
    /// nanosleep when polling).
    pub fn wait(&self, seq: &AtomicU32, waiting: &AtomicU32, seen: u32, timeout: Duration) -> bool {
        if seq.load(SeqCst) != seen {
            return true;
        }
        let start = clock::now_ticks();
        let deadline = start + clock::ns_to_ticks(timeout.as_nanos() as u64);
        if self.spin_ticks > 0 {
            let spin_end = (start + self.spin_ticks).min(deadline);
            while clock::now_ticks() < spin_end {
                if seq.load(SeqCst) != seen {
                    return true;
                }
                std::hint::spin_loop();
            }
        }
        match self.os {
            Some(os) => loop {
                let now = clock::now_ticks();
                if now >= deadline {
                    return seq.load(SeqCst) != seen;
                }
                let remaining_ns = clock::ticks_to_ns(deadline - now).max(1);
                waiting.fetch_add(1, SeqCst);
                if seq.load(SeqCst) != seen {
                    waiting.fetch_sub(1, SeqCst);
                    return true;
                }
                // SAFETY: `seq` is a live, 4-byte aligned AtomicU32 (in shm).
                unsafe {
                    (os.wait)(
                        seq.as_ptr().cast(),
                        seen as u64,
                        4,
                        OS_SYNC_WAIT_ON_ADDRESS_SHARED,
                        OS_CLOCK_MACH_ABSOLUTE_TIME,
                        remaining_ns,
                    )
                };
                waiting.fetch_sub(1, SeqCst);
                if seq.load(SeqCst) != seen {
                    return true;
                }
                // Timeout, EINTR or spurious wake: loop re-checks the deadline.
            },
            None => loop {
                if seq.load(SeqCst) != seen {
                    return true;
                }
                if clock::now_ticks() >= deadline {
                    return false;
                }
                std::thread::sleep(self.poll);
            },
        }
    }
}

/// Driver-side signal: bump `seq` and wake a blocked waiter. Used by
/// [`crate::sim::DriverSim`]; the C++ driver does the same in `ShmLink::Signal`.
pub fn signal(seq: &AtomicU32, waiting: &AtomicU32) {
    seq.fetch_add(1, SeqCst);
    if waiting.load(SeqCst) != 0 {
        if let Some(os) = os_sync() {
            // SAFETY: `seq` is a live, 4-byte aligned AtomicU32.
            unsafe { (os.wake)(seq.as_ptr().cast(), 4, OS_SYNC_WAKE_BY_ADDRESS_SHARED) };
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::Arc;

    fn roundtrip(mode: WakeMode) {
        let seq = Arc::new(AtomicU32::new(0));
        let waiting = Arc::new(AtomicU32::new(0));
        let w = Waiter::new(mode);
        // Times out when nothing happens.
        assert!(!w.wait(&seq, &waiting, 0, Duration::from_millis(5)));
        assert_eq!(waiting.load(SeqCst), 0);
        // Wakes on a signal from another thread.
        let (s2, w2) = (seq.clone(), waiting.clone());
        let t = std::thread::spawn(move || {
            std::thread::sleep(Duration::from_millis(5));
            signal(&s2, &w2);
        });
        assert!(w.wait(&seq, &waiting, 0, Duration::from_secs(2)));
        t.join().unwrap();
        // Already changed: returns immediately.
        assert!(w.wait(&seq, &waiting, 0, Duration::from_secs(2)));
    }

    #[test]
    fn os_sync_roundtrip() {
        #[cfg(target_os = "macos")]
        assert!(os_sync_available(), "this project targets macOS 14.4+");
        roundtrip(WakeMode::OsSync {
            spin: Duration::ZERO,
        });
        roundtrip(WakeMode::OsSync {
            spin: Duration::from_micros(20),
        });
    }

    #[test]
    fn poll_roundtrip() {
        roundtrip(WakeMode::Poll {
            interval: Duration::from_micros(200),
        });
        assert_eq!(
            Waiter::new(WakeMode::Poll {
                interval: Duration::from_micros(1)
            })
            .mechanism(),
            Mechanism::Poll
        );
    }
}
