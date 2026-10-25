//! Denormal (subnormal) protection.
//!
//! Recursive filters decaying towards zero produce subnormal floats, which are slow on x86 and add needless
//! work everywhere. Primary defence: [`DenormalGuard`] sets flush-to-zero for the current thread
//! (aarch64: `FPCR.FZ` (bit 24), which flushes subnormal inputs *and* outputs for f32/f64 in AArch64 state, so
//! no separate DAZ exists; x86_64: `MXCSR.FTZ` (bit 15) + `MXCSR.DAZ` (bit 6)). Create one at the top of every
//! audio thread's run loop (or around a process callback); the previous mode is restored on drop.
//!
//! Note: LLVM assumes the default floating-point environment, so compile-time constant folding is unaffected
//! by FTZ; this is the standard, accepted practice for audio threads (JUCE `ScopedNoDenormals`, etc.).
//!
//! Fallbacks when FTZ cannot be relied on (other targets, offline renders in threads we do not own):
//! - add [`ANTI_DENORMAL`] (a tiny constant, -400 dBFS) into feedback paths, or
//! - call the `flush_denormals()` methods on filters once per block, or [`flush`] on single values.

use core::marker::PhantomData;

/// Tiny DC offset (~ -400 dB) to inject into recursive paths when FTZ is unavailable.
pub const ANTI_DENORMAL: f32 = 1e-20;

/// Zero `x` if its magnitude is below `1e-15` (well above the subnormal range, inaudible).
#[inline(always)]
pub fn flush(x: f32) -> f32 {
    if x.abs() < 1e-15 {
        0.0
    } else {
        x
    }
}

#[cfg(target_arch = "aarch64")]
mod imp {
    const FZ: u64 = 1 << 24;

    pub fn get() -> u64 {
        let v: u64;
        // SAFETY: reading FPCR has no side effects.
        unsafe { core::arch::asm!("mrs {}, fpcr", out(reg) v, options(nomem, nostack, preserves_flags)) };
        v
    }

    pub fn set(v: u64) {
        // SAFETY: writing FPCR only changes FP rounding/flush behaviour of this thread.
        unsafe { core::arch::asm!("msr fpcr, {}", in(reg) v, options(nomem, nostack, preserves_flags)) };
    }

    pub fn enable(v: u64) -> u64 {
        v | FZ
    }

    pub fn is_enabled() -> bool {
        get() & FZ != 0
    }
}

#[cfg(target_arch = "x86_64")]
mod imp {
    const FTZ_DAZ: u64 = (1 << 15) | (1 << 6);

    pub fn get() -> u64 {
        let mut csr: u32 = 0;
        // SAFETY: stores MXCSR into a valid local.
        unsafe {
            core::arch::asm!("stmxcsr [{}]", in(reg) &mut csr as *mut u32, options(nostack, preserves_flags))
        };
        csr as u64
    }

    pub fn set(v: u64) {
        let csr = v as u32;
        // SAFETY: loads MXCSR from a valid local; only control bits we read previously are changed.
        unsafe {
            core::arch::asm!("ldmxcsr [{}]", in(reg) &csr as *const u32, options(nostack, preserves_flags))
        };
    }

    pub fn enable(v: u64) -> u64 {
        v | FTZ_DAZ
    }

    pub fn is_enabled() -> bool {
        get() & FTZ_DAZ == FTZ_DAZ
    }
}

#[cfg(not(any(target_arch = "aarch64", target_arch = "x86_64")))]
mod imp {
    pub fn get() -> u64 {
        0
    }
    pub fn set(_: u64) {}
    pub fn enable(v: u64) -> u64 {
        v
    }
    pub fn is_enabled() -> bool {
        false
    }
}

/// RAII guard enabling flush-to-zero (and DAZ on x86_64) for the current thread. Not `Send`: the restore
/// must happen on the thread that set it.
pub struct DenormalGuard {
    prev: u64,
    _not_send: PhantomData<*const ()>,
}

impl DenormalGuard {
    pub fn new() -> Self {
        let prev = imp::get();
        imp::set(imp::enable(prev));
        Self {
            prev,
            _not_send: PhantomData,
        }
    }

    /// Whether FTZ is active on this thread (false on unsupported targets).
    pub fn is_active() -> bool {
        imp::is_enabled()
    }
}

impl Default for DenormalGuard {
    fn default() -> Self {
        Self::new()
    }
}

impl Drop for DenormalGuard {
    fn drop(&mut self) {
        imp::set(self.prev);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::hint::black_box;

    #[test]
    fn guard_flushes_and_restores() {
        let tiny = black_box(f32::MIN_POSITIVE);
        let before = black_box(tiny) * black_box(0.25);
        assert!(before > 0.0 && before.is_subnormal(), "default mode keeps subnormals");
        {
            let _g = DenormalGuard::new();
            if cfg!(any(target_arch = "aarch64", target_arch = "x86_64")) {
                assert!(DenormalGuard::is_active());
                let r = black_box(tiny) * black_box(0.25);
                assert_eq!(r, 0.0, "FTZ flushes subnormal results");
            }
        }
        let after = black_box(tiny) * black_box(0.25);
        assert!(after.is_subnormal(), "mode restored on drop");
    }

    #[test]
    fn flush_fallback() {
        assert_eq!(flush(1e-30), 0.0);
        assert_eq!(flush(-1e-20), 0.0);
        assert_eq!(flush(1e-3), 1e-3);
    }
}
