//! An audio work interval (`os_workgroup_interval`) for the daemon's RT driver thread and the
//! engine workers.
//!
//! The HAL driver's IO thread lives in another process (coreaudiod's driver host), so there is
//! no device workgroup the daemon could join. Instead the daemon creates its own interval
//! workgroup with `AudioWorkIntervalCreate` (AudioToolbox, macOS 14+; resolved with `dlsym`), the
//! driver thread and every engine worker join it, and the driver thread brackets each burst of
//! engine cycles with `os_workgroup_interval_start/finish` (deadline = the driver's IO buffer
//! period). The scheduler then treats the threads as one real-time unit (core placement,
//! frequency). Without the symbol everything still works, just without the workgroup.

use wing_engine::RawWorkgroup;

#[cfg(target_os = "macos")]
mod ffi {
    use std::ffi::{c_char, c_int, c_void};

    pub const OS_CLOCK_MACH_ABSOLUTE_TIME: u32 = 32;
    pub type CreateFn =
        unsafe extern "C" fn(name: *const c_char, clock: u32, attr: *mut c_void) -> *mut c_void;

    #[link(name = "AudioToolbox", kind = "framework")]
    extern "C" {}

    extern "C" {
        pub fn os_workgroup_interval_start(
            wg: *mut c_void,
            start: u64,
            deadline: u64,
            data: *mut c_void,
        ) -> c_int;
        pub fn os_workgroup_interval_finish(wg: *mut c_void, data: *mut c_void) -> c_int;
        pub fn os_release(obj: *mut c_void);
    }
}

/// An owned interval workgroup. Must outlive every thread that joined it.
pub struct WorkInterval {
    ptr: *mut std::ffi::c_void,
}

// SAFETY: os_workgroup objects are thread-safe OS objects.
unsafe impl Send for WorkInterval {}
// SAFETY: as above.
unsafe impl Sync for WorkInterval {}

impl WorkInterval {
    /// Create one (`None` if `AudioWorkIntervalCreate` is unavailable or fails).
    #[cfg(target_os = "macos")]
    pub fn create(name: &str) -> Option<Self> {
        let sym = c"AudioWorkIntervalCreate";
        // SAFETY: dlsym with RTLD_DEFAULT and a valid C string.
        let f = unsafe { libc::dlsym(libc::RTLD_DEFAULT, sym.as_ptr()) };
        if f.is_null() {
            return None;
        }
        // SAFETY: the symbol has this signature (AudioToolbox/AudioWorkInterval.h).
        let create: ffi::CreateFn = unsafe { std::mem::transmute(f) };
        let cname = std::ffi::CString::new(name).ok()?;
        // SAFETY: valid name, NULL attr = defaults.
        let ptr = unsafe {
            create(
                cname.as_ptr(),
                ffi::OS_CLOCK_MACH_ABSOLUTE_TIME,
                std::ptr::null_mut(),
            )
        };
        (!ptr.is_null()).then_some(Self { ptr })
    }

    /// Create one (unsupported platform).
    #[cfg(not(target_os = "macos"))]
    pub fn create(_name: &str) -> Option<Self> {
        None
    }

    /// The workgroup for [`wing_engine::WorkgroupMembership::join`] / engine workers.
    pub fn raw(&self) -> RawWorkgroup {
        // SAFETY: we own a reference for as long as `self` lives.
        unsafe { RawWorkgroup::from_raw(self.ptr) }
    }

    /// Begin an interval on the calling (joined) thread. Times in mach ticks. RT-safe.
    #[inline]
    pub fn start(&self, start_ticks: u64, deadline_ticks: u64) -> bool {
        #[cfg(target_os = "macos")]
        // SAFETY: valid workgroup; the OS rejects calls from non-members with an error code.
        unsafe {
            ffi::os_workgroup_interval_start(
                self.ptr,
                start_ticks,
                deadline_ticks,
                std::ptr::null_mut(),
            ) == 0
        }
        #[cfg(not(target_os = "macos"))]
        {
            let _ = (start_ticks, deadline_ticks);
            false
        }
    }

    /// End the current interval. RT-safe.
    #[inline]
    pub fn finish(&self) {
        #[cfg(target_os = "macos")]
        // SAFETY: as for `start`.
        unsafe {
            ffi::os_workgroup_interval_finish(self.ptr, std::ptr::null_mut());
        }
    }
}

impl Drop for WorkInterval {
    fn drop(&mut self) {
        #[cfg(target_os = "macos")]
        // SAFETY: we own one reference from the create call.
        unsafe {
            ffi::os_release(self.ptr)
        };
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    #[cfg(target_os = "macos")]
    fn create_join_start_finish() {
        let Some(wi) = WorkInterval::create("wing-sim-test") else {
            eprintln!("AudioWorkIntervalCreate unavailable; skipped");
            return;
        };
        let raw = wi.raw();
        let ok = std::thread::spawn(move || {
            let m = wing_engine::WorkgroupMembership::join(raw).expect("join");
            let now = wing_shm::clock::now_ticks();
            let ok = wi_start(raw, now, now + wing_shm::clock::ns_to_ticks(1_000_000));
            drop(m);
            ok
        })
        .join()
        .unwrap();
        assert!(ok);
        drop(wi);

        fn wi_start(raw: RawWorkgroup, s: u64, d: u64) -> bool {
            // SAFETY: test-only use of the raw pointer while the owner is alive.
            unsafe {
                let r = ffi::os_workgroup_interval_start(raw.as_ptr(), s, d, std::ptr::null_mut());
                ffi::os_workgroup_interval_finish(raw.as_ptr(), std::ptr::null_mut());
                r == 0
            }
        }
    }
}
