//! Real-time thread setup.
//!
//! * Time constraint: [`set_current_thread_time_constraint`] calls
//!   `thread_policy_set(THREAD_TIME_CONSTRAINT_POLICY)` for the calling thread, sized from the
//!   audio cycle period.
//! * Workgroups: [`WorkgroupMembership`] joins/leaves an `os_workgroup` (macOS 11+). It normally
//!   comes from the CoreAudio device (`kAudioDevicePropertyIOThreadOSWorkgroup`); this module only
//!   takes a raw pointer ([`RawWorkgroup`]), the device hookup lives elsewhere (T023).
//!
//! On non-macOS targets every call returns [`RtError::Unsupported`].

use std::ffi::c_void;
use std::fmt;

/// Error from RT setup calls.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RtError {
    /// Not supported on this platform.
    Unsupported,
    /// `thread_policy_set` failed with this `kern_return_t`.
    Policy(i32),
    /// `os_workgroup_join` failed with this errno (EINVAL: cancelled, EALREADY: already in one).
    Join(i32),
    /// Null workgroup pointer.
    NullWorkgroup,
}

impl fmt::Display for RtError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            RtError::Unsupported => {
                write!(f, "real-time thread setup unsupported on this platform")
            }
            RtError::Policy(k) => write!(f, "thread_policy_set failed: kern_return {k}"),
            RtError::Join(e) => write!(f, "os_workgroup_join failed: errno {e}"),
            RtError::NullWorkgroup => write!(f, "null os_workgroup pointer"),
        }
    }
}

impl std::error::Error for RtError {}

/// Mach time-constraint parameters, in nanoseconds.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TimeConstraint {
    /// Nominal period between activations.
    pub period_ns: u64,
    /// CPU time needed per period.
    pub computation_ns: u64,
    /// Deadline from activation start.
    pub constraint_ns: u64,
    /// Whether the computation may be interrupted.
    pub preemptible: bool,
}

impl TimeConstraint {
    /// Policy for an audio thread with cycle `period_ns`: computation = half the period
    /// (what CoreAudio's own IO threads request), constraint = the full period.
    pub fn for_period(period_ns: u64) -> Self {
        Self {
            period_ns,
            computation_ns: period_ns / 2,
            constraint_ns: period_ns,
            preemptible: true,
        }
    }
}

/// Apply a time-constraint policy to the calling thread.
#[cfg(target_os = "macos")]
pub fn set_current_thread_time_constraint(tc: TimeConstraint) -> Result<(), RtError> {
    let (numer, denom) = timebase();
    let to_abs = |ns: u64| -> u32 {
        (ns as u128 * denom as u128 / numer as u128).min(u32::MAX as u128) as u32
    };
    let mut policy = libc::thread_time_constraint_policy {
        period: to_abs(tc.period_ns),
        computation: to_abs(tc.computation_ns),
        constraint: to_abs(tc.constraint_ns),
        preemptible: tc.preemptible as libc::boolean_t,
    };
    // SAFETY: plain FFI on the current thread's port; `pthread_mach_thread_np` does not create a
    // new port right (unlike `mach_thread_self`), so nothing leaks.
    let kr = unsafe {
        let port = libc::pthread_mach_thread_np(libc::pthread_self());
        libc::thread_policy_set(
            port,
            libc::THREAD_TIME_CONSTRAINT_POLICY as libc::thread_policy_flavor_t,
            &mut policy as *mut _ as libc::thread_policy_t,
            libc::THREAD_TIME_CONSTRAINT_POLICY_COUNT,
        )
    };
    if kr == 0 {
        Ok(())
    } else {
        Err(RtError::Policy(kr))
    }
}

/// Apply a time-constraint policy to the calling thread.
#[cfg(not(target_os = "macos"))]
pub fn set_current_thread_time_constraint(_tc: TimeConstraint) -> Result<(), RtError> {
    Err(RtError::Unsupported)
}

#[cfg(target_os = "macos")]
fn timebase() -> (u32, u32) {
    let mut tb = ffi::Timebase { numer: 0, denom: 0 };
    // SAFETY: writes into a valid struct.
    let r = unsafe { ffi::mach_timebase_info(&mut tb) };
    if r != 0 || tb.numer == 0 || tb.denom == 0 {
        (1, 1)
    } else {
        (tb.numer, tb.denom)
    }
}

/// A raw `os_workgroup_t` pointer (e.g. from `kAudioDevicePropertyIOThreadOSWorkgroup`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RawWorkgroup(*mut c_void);

// SAFETY: os_workgroup objects are thread-safe OS objects; the pointer is only an identity.
unsafe impl Send for RawWorkgroup {}
// SAFETY: as above.
unsafe impl Sync for RawWorkgroup {}

impl RawWorkgroup {
    /// Wrap a raw `os_workgroup_t`.
    ///
    /// # Safety
    /// `ptr` must be a valid `os_workgroup_t` that the caller keeps alive (retained) for as long
    /// as any thread may join or be a member of it.
    pub unsafe fn from_raw(ptr: *mut c_void) -> Self {
        Self(ptr)
    }

    /// The raw pointer.
    pub fn as_ptr(self) -> *mut c_void {
        self.0
    }
}

#[cfg(target_os = "macos")]
mod ffi {
    use std::ffi::c_void;

    /// `struct os_workgroup_join_token_opaque_s { uint32_t sig; char opaque[36]; }`
    #[repr(C)]
    pub struct JoinToken {
        pub sig: u32,
        pub opaque: [u8; 36],
    }

    /// `mach_timebase_info_data_t`
    #[repr(C)]
    pub struct Timebase {
        pub numer: u32,
        pub denom: u32,
    }

    extern "C" {
        pub fn mach_timebase_info(info: *mut Timebase) -> i32;
        pub fn os_workgroup_join(wg: *mut c_void, token_out: *mut JoinToken) -> i32;
        pub fn os_workgroup_leave(wg: *mut c_void, token: *mut JoinToken);
        pub fn os_workgroup_max_parallel_threads(wg: *mut c_void, attr: *mut c_void) -> i32;
    }
}

/// Membership of the *current thread* in an `os_workgroup`. Leaves on drop; must be dropped on
/// the thread that joined (hence `!Send`).
pub struct WorkgroupMembership {
    wg: RawWorkgroup,
    #[cfg(target_os = "macos")]
    token: Box<ffi::JoinToken>,
    _not_send: std::marker::PhantomData<*mut ()>,
}

impl WorkgroupMembership {
    /// Join `wg` from the calling thread. Allocates the join token; call off the audio path
    /// (worker threads do this between cycles).
    #[cfg(target_os = "macos")]
    pub fn join(wg: RawWorkgroup) -> Result<Self, RtError> {
        if wg.0.is_null() {
            return Err(RtError::NullWorkgroup);
        }
        let mut token = Box::new(ffi::JoinToken {
            sig: 0,
            opaque: [0; 36],
        });
        // SAFETY: `wg` validity is guaranteed by `RawWorkgroup::from_raw`'s contract.
        let r = unsafe { ffi::os_workgroup_join(wg.0, &mut *token) };
        if r == 0 {
            Ok(Self {
                wg,
                token,
                _not_send: std::marker::PhantomData,
            })
        } else {
            Err(RtError::Join(r))
        }
    }

    /// Join `wg` from the calling thread.
    #[cfg(not(target_os = "macos"))]
    pub fn join(_wg: RawWorkgroup) -> Result<Self, RtError> {
        Err(RtError::Unsupported)
    }

    /// The joined workgroup.
    pub fn workgroup(&self) -> RawWorkgroup {
        self.wg
    }
}

impl Drop for WorkgroupMembership {
    fn drop(&mut self) {
        #[cfg(target_os = "macos")]
        // SAFETY: we joined `wg` on this thread with this token.
        unsafe {
            ffi::os_workgroup_leave(self.wg.0, &mut *self.token)
        };
    }
}

/// Maximum number of threads the workgroup recommends (0 if unknown/unsupported).
pub fn workgroup_max_parallel_threads(wg: RawWorkgroup) -> usize {
    #[cfg(target_os = "macos")]
    {
        if wg.0.is_null() {
            return 0;
        }
        // SAFETY: `wg` validity is guaranteed by `RawWorkgroup::from_raw`'s contract.
        let n = unsafe { ffi::os_workgroup_max_parallel_threads(wg.0, std::ptr::null_mut()) };
        n.max(0) as usize
    }
    #[cfg(not(target_os = "macos"))]
    {
        let _ = wg;
        0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    #[cfg(target_os = "macos")]
    fn time_constraint_applies() {
        // 64 samples @ 48 kHz.
        std::thread::spawn(|| {
            set_current_thread_time_constraint(TimeConstraint::for_period(1_333_333))
        })
        .join()
        .unwrap()
        .unwrap();
    }

    #[test]
    fn null_workgroup_rejected() {
        // SAFETY: null is explicitly handled.
        let wg = unsafe { RawWorkgroup::from_raw(std::ptr::null_mut()) };
        assert!(WorkgroupMembership::join(wg).is_err());
        assert_eq!(workgroup_max_parallel_threads(wg), 0);
    }
}
