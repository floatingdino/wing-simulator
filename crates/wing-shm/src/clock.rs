//! Host clock used for heartbeats: `mach_absolute_time()` ticks, the same
//! clock the driver uses. Reading it is a commpage load, not a syscall, so it
//! is real-time safe.

use std::sync::OnceLock;

#[cfg(target_vendor = "apple")]
mod sys {
    #[repr(C)]
    pub struct MachTimebaseInfo {
        pub numer: u32,
        pub denom: u32,
    }
    extern "C" {
        pub fn mach_absolute_time() -> u64;
        pub fn mach_timebase_info(info: *mut MachTimebaseInfo) -> i32;
    }
}

/// Current host time in ticks (`mach_absolute_time`). RT safe.
#[inline]
pub fn now_ticks() -> u64 {
    #[cfg(target_vendor = "apple")]
    {
        // SAFETY: no preconditions.
        unsafe { sys::mach_absolute_time() }
    }
    #[cfg(not(target_vendor = "apple"))]
    {
        let mut ts = libc::timespec {
            tv_sec: 0,
            tv_nsec: 0,
        };
        // SAFETY: valid out pointer.
        unsafe { libc::clock_gettime(libc::CLOCK_MONOTONIC, &mut ts) };
        ts.tv_sec as u64 * 1_000_000_000 + ts.tv_nsec as u64
    }
}

fn timebase() -> (u64, u64) {
    static TB: OnceLock<(u64, u64)> = OnceLock::new();
    *TB.get_or_init(|| {
        #[cfg(target_vendor = "apple")]
        {
            let mut info = sys::MachTimebaseInfo { numer: 0, denom: 0 };
            // SAFETY: valid out pointer.
            unsafe { sys::mach_timebase_info(&mut info) };
            if info.numer != 0 && info.denom != 0 {
                return (info.numer as u64, info.denom as u64);
            }
        }
        (1, 1)
    })
}

/// Initialise the cached timebase (call once off the RT thread).
pub fn init() {
    let _ = timebase();
}

/// Convert ticks to nanoseconds.
#[inline]
pub fn ticks_to_ns(ticks: u64) -> u64 {
    let (n, d) = timebase();
    ((ticks as u128 * n as u128) / d as u128) as u64
}

/// Convert nanoseconds to ticks.
#[inline]
pub fn ns_to_ticks(ns: u64) -> u64 {
    let (n, d) = timebase();
    ((ns as u128 * d as u128) / n as u128) as u64
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn monotonic_and_convertible() {
        let a = now_ticks();
        std::thread::sleep(std::time::Duration::from_millis(2));
        let b = now_ticks();
        let ns = ticks_to_ns(b - a);
        assert!((1_500_000..200_000_000).contains(&ns), "{ns}");
        let back = ticks_to_ns(ns_to_ticks(1_000_000));
        assert!((999_000..=1_000_000).contains(&back), "{back}");
    }
}
