//! DSP primitives shared by every WING block: biquads, SVF, detectors, oversampling, smoothing, delay lines.
//!
//! Design rules (see `CLAUDE.md`, real-time rules):
//! - Every type allocates (if at all) only in its constructor; all `process*`/`next*`/`set*` methods are
//!   allocation-, lock- and syscall-free. `tests/no_alloc.rs` enforces this with a counting allocator.
//! - Filter *design* is done in `f64`; runtime state is `f32` by default with `f64` variants where precision
//!   matters (low-frequency biquads, long time constants).
//! - Multi-channel processing uses structure-of-arrays lane groups ([`bank::BiquadBank`]) so the compiler
//!   vectorises across channels (NEON on Apple silicon) behind a per-channel API.
//! - Denormals: call [`denormal::DenormalGuard::new`] once per audio thread (FTZ on aarch64 / FTZ+DAZ on x86_64);
//!   [`denormal::ANTI_DENORMAL`] / `flush_denormals` methods are the portable fallback.
//!
//! Module map:
//! - [`gain`]: dB/linear conversions, fast approximations with bounded error, gain tables.
//! - [`smooth`]: one-pole and linear-ramp parameter smoothers (per-sample and per-block).
//! - [`biquad`]: RBJ cookbook designs, Butterworth/Linkwitz-Riley cascades, TDF-II biquads (f32/f64),
//!   coefficient-smoothed biquad, analytic response evaluation.
//! - [`bank`]: N-channel x M-stage SoA biquad bank.
//! - [`svf`]: TPT (Zavalishin/Simper) state-variable filter for modulated filters.
//! - [`detector`]: peak, one-pole attack/release (peak/mean-square, program-dependent release), RMS window.
//! - [`delay`]: fractional delay line (integer, linear, Lagrange-3, allpass interpolation).
//! - [`oversample`]: 2x/4x polyphase half-band FIR up/down samplers with documented latency.
//! - [`denormal`]: FTZ/DAZ setup and fallbacks.

pub mod bank;
pub mod biquad;
pub mod delay;
pub mod denormal;
pub mod detector;
pub mod gain;
pub mod oversample;
pub mod smooth;
pub mod svf;

pub use bank::BiquadBank;
pub use biquad::{Biquad, BiquadF64, Cascade, Coeffs, SmoothedBiquad};
pub use delay::{AllpassReader, DelayLine};
pub use denormal::DenormalGuard;
pub use detector::{EnvelopeFollower, PeakDetector, RmsWindow};
pub use oversample::{Oversampler2x, Oversampler4x};
pub use smooth::{LinearRamp, OnePoleSmoother};
pub use svf::Svf;

/// Minimal complex number used for analytic frequency-response evaluation (tests, UI curves). Not used on
/// the audio path.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Complex {
    pub re: f64,
    pub im: f64,
}

impl Complex {
    pub const fn new(re: f64, im: f64) -> Self {
        Self { re, im }
    }
    /// `e^{j w}`.
    pub fn expj(w: f64) -> Self {
        Self::new(w.cos(), w.sin())
    }
    pub fn norm(self) -> f64 {
        self.re.hypot(self.im)
    }
    pub fn arg(self) -> f64 {
        self.im.atan2(self.re)
    }
    pub fn db(self) -> f64 {
        20.0 * self.norm().log10()
    }
}

impl core::ops::Add for Complex {
    type Output = Self;
    fn add(self, o: Self) -> Self {
        Self::new(self.re + o.re, self.im + o.im)
    }
}
impl core::ops::Sub for Complex {
    type Output = Self;
    fn sub(self, o: Self) -> Self {
        Self::new(self.re - o.re, self.im - o.im)
    }
}
impl core::ops::Mul for Complex {
    type Output = Self;
    fn mul(self, o: Self) -> Self {
        Self::new(
            self.re * o.re - self.im * o.im,
            self.re * o.im + self.im * o.re,
        )
    }
}
impl core::ops::Mul<f64> for Complex {
    type Output = Self;
    fn mul(self, k: f64) -> Self {
        Self::new(self.re * k, self.im * k)
    }
}
impl core::ops::Div for Complex {
    type Output = Self;
    fn div(self, o: Self) -> Self {
        let d = o.re * o.re + o.im * o.im;
        Self::new(
            (self.re * o.re + self.im * o.im) / d,
            (self.im * o.re - self.re * o.im) / d,
        )
    }
}

/// DTFT of a finite impulse response at normalised angular frequency `w` (radians/sample). Test helper for
/// measuring implemented filters against analytic responses.
pub fn dtft(h: &[f64], w: f64) -> Complex {
    let mut acc = Complex::new(0.0, 0.0);
    for (n, &x) in h.iter().enumerate() {
        acc = acc + Complex::expj(-w * n as f64) * x;
    }
    acc
}

/// Wrap a phase difference to (-pi, pi].
pub fn wrap_phase(p: f64) -> f64 {
    let tau = core::f64::consts::TAU;
    let mut x = p % tau;
    if x > core::f64::consts::PI {
        x -= tau;
    } else if x <= -core::f64::consts::PI {
        x += tau;
    }
    x
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn complex_ops() {
        let a = Complex::new(1.0, 2.0);
        let b = Complex::new(3.0, -1.0);
        let q = (a * b) / b;
        assert!((q.re - 1.0).abs() < 1e-12 && (q.im - 2.0).abs() < 1e-12);
        assert!((wrap_phase(3.0 * core::f64::consts::PI) - core::f64::consts::PI).abs() < 1e-12);
    }
}
