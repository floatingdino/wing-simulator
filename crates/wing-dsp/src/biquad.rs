//! Biquad filters: designs (RBJ Audio-EQ-Cookbook, Butterworth, Linkwitz-Riley) and runtime sections.
//!
//! All designs are bilinear transforms of the analog prototype, pre-warped at the design frequency, so the
//! digital response at angular frequency `w` equals the analog prototype evaluated at
//! `s = j * tan(w/2) / tan(w0/2)` (verified exactly in tests).
//!
//! Runtime sections use the transposed direct form II (TDF-II):
//! ```text
//! y  = b0*x + s1
//! s1 = b1*x - a1*y + s2
//! s2 = b2*x - a2*y
//! ```
//! [`Biquad`] keeps f32 coefficients/state; [`BiquadF64`] keeps f64 coefficients/state (use it for very low
//! cutoff frequencies relative to fs, e.g. < 30 Hz HPF, where f32 coefficient quantisation shifts the poles).
//! [`SmoothedBiquad`] ramps coefficients linearly per sample to avoid zipper noise; for fast modulation prefer
//! [`crate::svf::Svf`], which stays well-behaved under per-sample coefficient changes.

use crate::Complex;
use core::f64::consts::PI;

/// Normalised biquad coefficients (`a0 == 1`). Designed in f64.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Coeffs {
    pub b0: f64,
    pub b1: f64,
    pub b2: f64,
    pub a1: f64,
    pub a2: f64,
}

impl Default for Coeffs {
    fn default() -> Self {
        Self::IDENTITY
    }
}

/// `tan(pi * f / fs)` with the frequency clamped just below Nyquist.
#[inline]
fn prewarp(fs: f64, f: f64) -> f64 {
    let f = f.clamp(1e-6, fs * 0.499_9);
    (PI * f / fs).tan()
}

/// Q giving a bandwidth of `bw_oct` octaves (RBJ: between -3 dB points for BPF/notch, between
/// half-gain-in-dB points for peaking EQ), with RBJ's approximate bilinear-warping compensation
/// (measured error < 0.002 octave for f0 <= fs/6).
pub fn q_from_bandwidth(fs: f64, f0: f64, bw_oct: f64) -> f64 {
    let w0 = 2.0 * PI * f0 / fs;
    1.0 / (2.0 * (core::f64::consts::LN_2 / 2.0 * bw_oct * w0 / w0.sin()).sinh())
}

/// Q for a shelf of `gain_db` with RBJ "shelf slope" `s` (s = 1: steepest monotonic slope).
pub fn q_from_shelf_slope(gain_db: f64, s: f64) -> f64 {
    let a = 10f64.powf(gain_db / 40.0);
    1.0 / ((a + 1.0 / a) * (1.0 / s - 1.0) + 2.0).sqrt()
}

impl Coeffs {
    /// Pass-through.
    pub const IDENTITY: Self = Self {
        b0: 1.0,
        b1: 0.0,
        b2: 0.0,
        a1: 0.0,
        a2: 0.0,
    };

    /// Build from un-normalised `b0..b2, a0..a2`.
    pub fn from_raw(b0: f64, b1: f64, b2: f64, a0: f64, a1: f64, a2: f64) -> Self {
        let n = 1.0 / a0;
        Self {
            b0: b0 * n,
            b1: b1 * n,
            b2: b2 * n,
            a1: a1 * n,
            a2: a2 * n,
        }
    }

    /// Shared RBJ intermediates: (cos w0, alpha) with alpha = sin(w0)/(2Q).
    fn rbj(fs: f64, f0: f64, q: f64) -> (f64, f64) {
        let f0 = f0.clamp(1e-6, fs * 0.499_9);
        let w0 = 2.0 * PI * f0 / fs;
        (w0.cos(), w0.sin() / (2.0 * q))
    }

    /// 2nd-order low-pass, H(s) = 1/(s^2 + s/Q + 1).
    pub fn lowpass(fs: f64, f0: f64, q: f64) -> Self {
        let (c, al) = Self::rbj(fs, f0, q);
        Self::from_raw(
            (1.0 - c) / 2.0,
            1.0 - c,
            (1.0 - c) / 2.0,
            1.0 + al,
            -2.0 * c,
            1.0 - al,
        )
    }

    /// 2nd-order high-pass, H(s) = s^2/(s^2 + s/Q + 1).
    pub fn highpass(fs: f64, f0: f64, q: f64) -> Self {
        let (c, al) = Self::rbj(fs, f0, q);
        Self::from_raw(
            (1.0 + c) / 2.0,
            -(1.0 + c),
            (1.0 + c) / 2.0,
            1.0 + al,
            -2.0 * c,
            1.0 - al,
        )
    }

    /// Band-pass with 0 dB peak gain, H(s) = (s/Q)/(s^2 + s/Q + 1).
    pub fn bandpass(fs: f64, f0: f64, q: f64) -> Self {
        let (c, al) = Self::rbj(fs, f0, q);
        Self::from_raw(al, 0.0, -al, 1.0 + al, -2.0 * c, 1.0 - al)
    }

    /// Notch, H(s) = (s^2 + 1)/(s^2 + s/Q + 1).
    pub fn notch(fs: f64, f0: f64, q: f64) -> Self {
        let (c, al) = Self::rbj(fs, f0, q);
        Self::from_raw(1.0, -2.0 * c, 1.0, 1.0 + al, -2.0 * c, 1.0 - al)
    }

    /// 2nd-order allpass, H(s) = (s^2 - s/Q + 1)/(s^2 + s/Q + 1).
    pub fn allpass(fs: f64, f0: f64, q: f64) -> Self {
        let (c, al) = Self::rbj(fs, f0, q);
        Self::from_raw(1.0 - al, -2.0 * c, 1.0 + al, 1.0 + al, -2.0 * c, 1.0 - al)
    }

    /// Peaking EQ, H(s) = (s^2 + s*A/Q + 1)/(s^2 + s/(A*Q) + 1), A = 10^(dB/40).
    pub fn peak(fs: f64, f0: f64, q: f64, gain_db: f64) -> Self {
        let (c, al) = Self::rbj(fs, f0, q);
        let a = 10f64.powf(gain_db / 40.0);
        Self::from_raw(
            1.0 + al * a,
            -2.0 * c,
            1.0 - al * a,
            1.0 + al / a,
            -2.0 * c,
            1.0 - al / a,
        )
    }

    /// Low shelf, H(s) = A * (s^2 + (sqrt(A)/Q) s + A)/(A s^2 + (sqrt(A)/Q) s + 1).
    pub fn low_shelf(fs: f64, f0: f64, q: f64, gain_db: f64) -> Self {
        let (c, al) = Self::rbj(fs, f0, q);
        let a = 10f64.powf(gain_db / 40.0);
        let sa = 2.0 * a.sqrt() * al;
        Self::from_raw(
            a * ((a + 1.0) - (a - 1.0) * c + sa),
            2.0 * a * ((a - 1.0) - (a + 1.0) * c),
            a * ((a + 1.0) - (a - 1.0) * c - sa),
            (a + 1.0) + (a - 1.0) * c + sa,
            -2.0 * ((a - 1.0) + (a + 1.0) * c),
            (a + 1.0) + (a - 1.0) * c - sa,
        )
    }

    /// High shelf, H(s) = A * (A s^2 + (sqrt(A)/Q) s + 1)/(s^2 + (sqrt(A)/Q) s + A).
    pub fn high_shelf(fs: f64, f0: f64, q: f64, gain_db: f64) -> Self {
        let (c, al) = Self::rbj(fs, f0, q);
        let a = 10f64.powf(gain_db / 40.0);
        let sa = 2.0 * a.sqrt() * al;
        Self::from_raw(
            a * ((a + 1.0) + (a - 1.0) * c + sa),
            -2.0 * a * ((a - 1.0) + (a + 1.0) * c),
            a * ((a + 1.0) + (a - 1.0) * c - sa),
            (a + 1.0) - (a - 1.0) * c + sa,
            2.0 * ((a - 1.0) - (a + 1.0) * c),
            (a + 1.0) - (a - 1.0) * c - sa,
        )
    }

    /// Shelf variants parameterised by RBJ shelf slope `s` instead of Q.
    pub fn low_shelf_slope(fs: f64, f0: f64, s: f64, gain_db: f64) -> Self {
        Self::low_shelf(fs, f0, q_from_shelf_slope(gain_db, s), gain_db)
    }
    pub fn high_shelf_slope(fs: f64, f0: f64, s: f64, gain_db: f64) -> Self {
        Self::high_shelf(fs, f0, q_from_shelf_slope(gain_db, s), gain_db)
    }

    /// 1st-order low-pass (6 dB/oct), H(s) = 1/(s + 1).
    pub fn lowpass1(fs: f64, f0: f64) -> Self {
        let k = prewarp(fs, f0);
        Self::from_raw(k, k, 0.0, k + 1.0, k - 1.0, 0.0)
    }

    /// 1st-order high-pass (6 dB/oct), H(s) = s/(s + 1).
    pub fn highpass1(fs: f64, f0: f64) -> Self {
        let k = prewarp(fs, f0);
        Self::from_raw(1.0, -1.0, 0.0, k + 1.0, k - 1.0, 0.0)
    }

    /// 1st-order allpass, H(s) = (1 - s)/(1 + s) (phase -90 deg at f0).
    pub fn allpass1(fs: f64, f0: f64) -> Self {
        let k = prewarp(fs, f0);
        Self::from_raw(k - 1.0, k + 1.0, 0.0, k + 1.0, k - 1.0, 0.0)
    }

    /// Complex frequency response at `f` Hz.
    pub fn response(&self, fs: f64, f: f64) -> Complex {
        self.response_w(2.0 * PI * f / fs)
    }

    /// Complex frequency response at angular frequency `w` (rad/sample).
    pub fn response_w(&self, w: f64) -> Complex {
        let z1 = Complex::expj(-w);
        let z2 = Complex::expj(-2.0 * w);
        let num = Complex::new(self.b0, 0.0) + z1 * self.b1 + z2 * self.b2;
        let den = Complex::new(1.0, 0.0) + z1 * self.a1 + z2 * self.a2;
        num / den
    }

    /// Magnitude in dB at `f` Hz.
    pub fn magnitude_db(&self, fs: f64, f: f64) -> f64 {
        self.response(fs, f).db()
    }
}

/// Filter family for cascaded cuts.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CutKind {
    Lowpass,
    Highpass,
}

/// Up to four biquad sections (8th order / 48 dB/oct) stored inline.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Sections {
    pub coeffs: [Coeffs; 4],
    pub len: usize,
}

impl Sections {
    fn push(&mut self, c: Coeffs) {
        self.coeffs[self.len] = c;
        self.len += 1;
    }
    pub fn as_slice(&self) -> &[Coeffs] {
        &self.coeffs[..self.len]
    }
    /// Product of section responses.
    pub fn response(&self, fs: f64, f: f64) -> Complex {
        self.as_slice()
            .iter()
            .fold(Complex::new(1.0, 0.0), |acc, c| acc * c.response(fs, f))
    }
}

/// Butterworth sections of `order` 1..=8 (6..48 dB/oct). Pole-pair Qs: `1/(2 cos(theta))` with
/// theta = (2k-1)pi/(2n) (even n) or k*pi/n (odd n, plus one 1st-order section).
pub fn butterworth(kind: CutKind, fs: f64, f0: f64, order: usize) -> Sections {
    assert!((1..=8).contains(&order), "Butterworth order 1..=8");
    let mut s = Sections {
        coeffs: [Coeffs::IDENTITY; 4],
        len: 0,
    };
    let n = order as f64;
    if order % 2 == 1 {
        s.push(match kind {
            CutKind::Lowpass => Coeffs::lowpass1(fs, f0),
            CutKind::Highpass => Coeffs::highpass1(fs, f0),
        });
    }
    for k in 1..=order / 2 {
        let theta = if order % 2 == 0 {
            (2 * k - 1) as f64 * PI / (2.0 * n)
        } else {
            k as f64 * PI / n
        };
        let q = 1.0 / (2.0 * theta.cos());
        s.push(match kind {
            CutKind::Lowpass => Coeffs::lowpass(fs, f0, q),
            CutKind::Highpass => Coeffs::highpass(fs, f0, q),
        });
    }
    s
}

/// Linkwitz-Riley sections of `order` 2, 4, 6 or 8 (12..48 dB/oct) = Butterworth(order/2) squared.
/// LP and HP are -6.02 dB at f0; LR4/LR8 LP+HP sum to an allpass (flat magnitude).
pub fn linkwitz_riley(kind: CutKind, fs: f64, f0: f64, order: usize) -> Sections {
    assert!(
        matches!(order, 2 | 4 | 6 | 8),
        "Linkwitz-Riley order 2/4/6/8"
    );
    let half = butterworth(kind, fs, f0, order / 2);
    let mut s = half;
    for c in half.as_slice() {
        s.push(*c);
    }
    s
}

macro_rules! tdf2_biquad {
    ($name:ident, $t:ty, $doc:literal) => {
        #[doc = $doc]
        #[derive(Clone, Copy, Debug, Default)]
        pub struct $name {
            pub b0: $t,
            pub b1: $t,
            pub b2: $t,
            pub a1: $t,
            pub a2: $t,
            s1: $t,
            s2: $t,
        }

        impl $name {
            pub fn new(c: Coeffs) -> Self {
                let mut b = Self::default();
                b.set_coeffs(c);
                b
            }

            /// Change coefficients, keeping state (may click on large jumps; see [`SmoothedBiquad`]).
            #[inline]
            pub fn set_coeffs(&mut self, c: Coeffs) {
                self.b0 = c.b0 as $t;
                self.b1 = c.b1 as $t;
                self.b2 = c.b2 as $t;
                self.a1 = c.a1 as $t;
                self.a2 = c.a2 as $t;
            }

            pub fn reset(&mut self) {
                self.s1 = 0.0;
                self.s2 = 0.0;
            }

            /// Zero state values below 1e-15 (portable denormal fallback; call once per block).
            #[inline]
            pub fn flush_denormals(&mut self) {
                if self.s1.abs() < 1e-15 {
                    self.s1 = 0.0;
                }
                if self.s2.abs() < 1e-15 {
                    self.s2 = 0.0;
                }
            }

            #[inline(always)]
            pub fn tick(&mut self, x: $t) -> $t {
                let y = self.b0 * x + self.s1;
                self.s1 = self.b1 * x - self.a1 * y + self.s2;
                self.s2 = self.b2 * x - self.a2 * y;
                y
            }
        }
    };
}

tdf2_biquad!(
    Biquad,
    f32,
    "TDF-II biquad with f32 coefficients and state."
);
tdf2_biquad!(
    BiquadF64,
    f64,
    "TDF-II biquad with f64 coefficients and state (f32 or f64 I/O)."
);

impl Biquad {
    #[inline]
    pub fn process(&mut self, x: f32) -> f32 {
        self.tick(x)
    }

    #[inline]
    pub fn process_block(&mut self, buf: &mut [f32]) {
        for x in buf {
            *x = self.tick(*x);
        }
    }
}

impl BiquadF64 {
    #[inline]
    pub fn process(&mut self, x: f32) -> f32 {
        self.tick(x as f64) as f32
    }

    #[inline]
    pub fn process_f64(&mut self, x: f64) -> f64 {
        self.tick(x)
    }

    #[inline]
    pub fn process_block(&mut self, buf: &mut [f32]) {
        for x in buf {
            *x = self.tick(*x as f64) as f32;
        }
    }
}

/// TDF-II biquad whose coefficients ramp linearly (per sample) to a new target over `ramp_len` samples.
/// Linear interpolation between two stable coefficient sets is stable for 2nd-order sections (the stability
/// triangle in (a1, a2) is convex), so this is click-free for EQ automation.
#[derive(Clone, Copy, Debug)]
pub struct SmoothedBiquad {
    cur: [f32; 5],
    step: [f32; 5],
    target: [f32; 5],
    remaining: u32,
    ramp_len: u32,
    s1: f32,
    s2: f32,
}

#[inline]
fn coeff_array(c: Coeffs) -> [f32; 5] {
    [
        c.b0 as f32,
        c.b1 as f32,
        c.b2 as f32,
        c.a1 as f32,
        c.a2 as f32,
    ]
}

impl SmoothedBiquad {
    pub fn new(c: Coeffs, ramp_len: u32) -> Self {
        let a = coeff_array(c);
        Self {
            cur: a,
            step: [0.0; 5],
            target: a,
            remaining: 0,
            ramp_len,
            s1: 0.0,
            s2: 0.0,
        }
    }

    pub fn set_ramp_len(&mut self, ramp_len: u32) {
        self.ramp_len = ramp_len;
    }

    /// Start ramping towards `c`.
    pub fn set_target(&mut self, c: Coeffs) {
        self.target = coeff_array(c);
        if self.ramp_len == 0 {
            self.cur = self.target;
            self.remaining = 0;
            return;
        }
        let inv = 1.0 / self.ramp_len as f32;
        for i in 0..5 {
            self.step[i] = (self.target[i] - self.cur[i]) * inv;
        }
        self.remaining = self.ramp_len;
    }

    /// Current (possibly mid-ramp) coefficients.
    pub fn current(&self) -> [f32; 5] {
        self.cur
    }

    pub fn reset(&mut self) {
        self.s1 = 0.0;
        self.s2 = 0.0;
    }

    #[inline]
    pub fn process(&mut self, x: f32) -> f32 {
        if self.remaining > 0 {
            self.remaining -= 1;
            if self.remaining == 0 {
                self.cur = self.target;
            } else {
                for i in 0..5 {
                    self.cur[i] += self.step[i];
                }
            }
        }
        let [b0, b1, b2, a1, a2] = self.cur;
        let y = b0 * x + self.s1;
        self.s1 = b1 * x - a1 * y + self.s2;
        self.s2 = b2 * x - a2 * y;
        y
    }

    #[inline]
    pub fn process_block(&mut self, buf: &mut [f32]) {
        for x in buf {
            *x = self.process(*x);
        }
    }
}

/// Cascade of up to four [`BiquadF64`] sections (e.g. 6-48 dB/oct cuts). f64 sections because steep low cuts
/// (e.g. 8th-order 20 Hz) need the precision: f32 sections deviate by > 0.01 dB in the stopband.
#[derive(Clone, Copy, Debug, Default)]
pub struct Cascade {
    sections: [BiquadF64; 4],
    len: usize,
}

impl Cascade {
    pub fn new(s: &Sections) -> Self {
        let mut c = Self::default();
        c.set_sections(s);
        c
    }

    /// Butterworth cut with slope `6 * order` dB/oct.
    pub fn butterworth(kind: CutKind, fs: f64, f0: f64, order: usize) -> Self {
        Self::new(&butterworth(kind, fs, f0, order))
    }

    /// Linkwitz-Riley cut of `order` 2/4/6/8.
    pub fn linkwitz_riley(kind: CutKind, fs: f64, f0: f64, order: usize) -> Self {
        Self::new(&linkwitz_riley(kind, fs, f0, order))
    }

    /// Replace sections, keeping state of sections that remain (changing the section count resets new ones).
    pub fn set_sections(&mut self, s: &Sections) {
        for (i, c) in s.as_slice().iter().enumerate() {
            if i >= self.len {
                self.sections[i].reset();
            }
            self.sections[i].set_coeffs(*c);
        }
        self.len = s.len;
    }

    pub fn len(&self) -> usize {
        self.len
    }

    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    pub fn reset(&mut self) {
        for s in &mut self.sections {
            s.reset();
        }
    }

    #[inline]
    pub fn process(&mut self, x: f32) -> f32 {
        let mut y = x as f64;
        for s in &mut self.sections[..self.len] {
            y = s.tick(y);
        }
        y as f32
    }

    #[inline]
    pub fn process_block(&mut self, buf: &mut [f32]) {
        for x in buf {
            *x = self.process(*x);
        }
    }

    pub fn flush_denormals(&mut self) {
        for s in &mut self.sections[..self.len] {
            s.flush_denormals();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{dtft, wrap_phase};

    const FS: f64 = 48_000.0;

    /// Analog prototype evaluated at the bilinear-warped frequency.
    fn s_at(f: f64, f0: f64) -> Complex {
        Complex::new(0.0, (PI * f / FS).tan() / (PI * f0 / FS).tan())
    }

    fn poly2(s: Complex, a: f64, b: f64, c: f64) -> Complex {
        s * s * a + s * b + Complex::new(c, 0.0)
    }

    fn test_freqs() -> impl Iterator<Item = f64> {
        (0..200).map(|i| 10.0 * (2000f64).powf(i as f64 / 199.0)) // 10 Hz .. 20 kHz
    }

    fn assert_close(d: &Coeffs, analog: impl Fn(Complex) -> Complex, f0: f64, name: &str) {
        for f in test_freqs() {
            let hd = d.response(FS, f);
            let ha = analog(s_at(f, f0));
            let mag_err = (hd.db() - ha.db()).abs();
            assert!(mag_err < 0.01, "{name}: f={f} mag err {mag_err} dB");
            if ha.norm() > 1e-6 {
                let ph_err = wrap_phase(hd.arg() - ha.arg()).abs();
                assert!(ph_err < 1e-6, "{name}: f={f} phase err {ph_err}");
            }
        }
    }

    #[test]
    fn rbj_designs_match_analog_prototypes() {
        let one = Complex::new(1.0, 0.0);
        for &f0 in &[40.0, 1000.0, 12_000.0] {
            for &q in &[0.5, 0.707, 2.0, 8.0] {
                let den = |s: Complex| poly2(s, 1.0, 1.0 / q, 1.0);
                assert_close(
                    &Coeffs::lowpass(FS, f0, q),
                    |s| one / den(s),
                    f0,
                    "lowpass",
                );
                assert_close(
                    &Coeffs::highpass(FS, f0, q),
                    |s| s * s / den(s),
                    f0,
                    "highpass",
                );
                assert_close(
                    &Coeffs::bandpass(FS, f0, q),
                    |s| s * (1.0 / q) / den(s),
                    f0,
                    "bandpass",
                );
                assert_close(
                    &Coeffs::notch(FS, f0, q),
                    |s| poly2(s, 1.0, 0.0, 1.0) / den(s),
                    f0,
                    "notch",
                );
                assert_close(
                    &Coeffs::allpass(FS, f0, q),
                    |s| poly2(s, 1.0, -1.0 / q, 1.0) / den(s),
                    f0,
                    "allpass",
                );
                for &g in &[-15.0, -3.0, 6.0, 15.0] {
                    let a: f64 = 10f64.powf(g / 40.0);
                    assert_close(
                        &Coeffs::peak(FS, f0, q, g),
                        |s| poly2(s, 1.0, a / q, 1.0) / poly2(s, 1.0, 1.0 / (a * q), 1.0),
                        f0,
                        "peak",
                    );
                    let sa = a.sqrt() / q;
                    assert_close(
                        &Coeffs::low_shelf(FS, f0, q, g),
                        |s| poly2(s, a, sa * a, a * a) / poly2(s, a, sa, 1.0),
                        f0,
                        "low_shelf",
                    );
                    assert_close(
                        &Coeffs::high_shelf(FS, f0, q, g),
                        |s| poly2(s, a * a, sa * a, a) / poly2(s, 1.0, sa, a),
                        f0,
                        "high_shelf",
                    );
                }
            }
            assert_close(
                &Coeffs::lowpass1(FS, f0),
                |s| one / (s + one),
                f0,
                "lowpass1",
            );
            assert_close(
                &Coeffs::highpass1(FS, f0),
                |s| s / (s + one),
                f0,
                "highpass1",
            );
            assert_close(
                &Coeffs::allpass1(FS, f0),
                |s| (one - s) / (one + s),
                f0,
                "allpass1",
            );
        }
    }

    #[test]
    fn landmark_gains() {
        assert!((Coeffs::peak(FS, 1000.0, 1.0, 12.0).magnitude_db(FS, 1000.0) - 12.0).abs() < 1e-9);
        assert!((Coeffs::low_shelf(FS, 200.0, 0.707, 9.0).magnitude_db(FS, 1.0) - 9.0).abs() < 1e-3);
        assert!((Coeffs::low_shelf(FS, 200.0, 0.707, 9.0).magnitude_db(FS, 200.0) - 4.5).abs() < 1e-9);
        assert!(
            (Coeffs::high_shelf(FS, 5000.0, 0.707, -6.0).magnitude_db(FS, 5000.0) + 3.0).abs() < 1e-9
        );
        assert!((Coeffs::lowpass(FS, 1000.0, core::f64::consts::FRAC_1_SQRT_2).magnitude_db(FS, 1000.0) + 3.0103).abs() < 1e-3);
        assert!(Coeffs::notch(FS, 1000.0, 4.0).magnitude_db(FS, 1000.0) < -100.0);
    }

    #[test]
    fn q_variants() {
        // Shelf slope S = 1 -> Q = 1/sqrt(2).
        assert!((q_from_shelf_slope(6.0, 1.0) - core::f64::consts::FRAC_1_SQRT_2).abs() < 1e-12);
        // Bandwidth: BPF -3 dB points are bw_oct apart in the digital domain.
        for &(f0, bw) in &[(1000.0, 1.0), (8000.0, 0.5), (200.0, 2.0)] {
            let c = Coeffs::bandpass(FS, f0, q_from_bandwidth(FS, f0, bw));
            let edge = |lo: f64, hi: f64| {
                let (mut lo, mut hi) = (lo, hi);
                let below = c.magnitude_db(FS, lo) < -3.0103;
                for _ in 0..200 {
                    let mid = (lo * hi).sqrt();
                    if (c.magnitude_db(FS, mid) < -3.0103) == below {
                        lo = mid;
                    } else {
                        hi = mid;
                    }
                }
                lo
            };
            let f1 = edge(1.0, f0);
            let f2 = edge(f0, FS * 0.4999);
            let measured = (f2 / f1).log2();
            // RBJ's w0/sin(w0) warping compensation is approximate: < 0.002 octave error up to fs/6.
            assert!((measured - bw).abs() < 2e-3, "f0={f0} bw={bw} measured={measured}");
        }
    }

    #[test]
    fn butterworth_and_lr_match_analog() {
        for &f0 in &[30.0, 1000.0, 10_000.0] {
            for order in 1..=8 {
                for kind in [CutKind::Lowpass, CutKind::Highpass] {
                    let s = butterworth(kind, FS, f0, order);
                    assert_eq!(s.len, order.div_ceil(2));
                    for f in test_freqs() {
                        let w = (PI * f / FS).tan() / (PI * f0 / FS).tan();
                        let wn = if kind == CutKind::Lowpass { w } else { 1.0 / w };
                        let analog_db = -10.0 * (1.0 + wn.powi(2 * order as i32)).log10();
                        let d = s.response(FS, f).db();
                        if analog_db > -200.0 {
                            assert!((d - analog_db).abs() < 0.01, "BW{order} f={f} {d} vs {analog_db}");
                        }
                        if order % 2 == 0 {
                            // LR(order) = Butterworth(order/2)^2
                            let half = -10.0 * (1.0 + wn.powi(order as i32)).log10();
                            if half > -100.0 {
                                let lr = linkwitz_riley(kind, FS, f0, order).response(FS, f).db();
                                assert!((lr - 2.0 * half).abs() < 0.01, "LR{order} f={f}");
                            }
                        }
                    }
                }
            }
            // -3.01 dB / -6.02 dB at f0
            assert!((butterworth(CutKind::Highpass, FS, f0, 5).response(FS, f0).db() + 3.0103).abs() < 1e-3);
            assert!((linkwitz_riley(CutKind::Lowpass, FS, f0, 4).response(FS, f0).db() + 6.0206).abs() < 1e-3);
        }
    }

    #[test]
    fn lr_crossover_sums_flat() {
        for order in [4, 8] {
            let lp = linkwitz_riley(CutKind::Lowpass, FS, 2000.0, order);
            let hp = linkwitz_riley(CutKind::Highpass, FS, 2000.0, order);
            for f in test_freqs() {
                let sum = lp.response(FS, f) + hp.response(FS, f);
                assert!(sum.db().abs() < 0.01, "LR{order} f={f} sum {} dB", sum.db());
            }
        }
    }

    fn impulse_response(mut f: impl FnMut(f32) -> f32, n: usize) -> Vec<f64> {
        (0..n)
            .map(|i| f(if i == 0 { 1.0 } else { 0.0 }) as f64)
            .collect()
    }

    #[test]
    fn runtime_sections_match_design() {
        let designs = [
            Coeffs::peak(FS, 1000.0, 2.0, 9.0),
            Coeffs::highpass(FS, 100.0, 0.707),
            Coeffs::low_shelf(FS, 300.0, 0.707, -6.0),
            Coeffs::allpass(FS, 3000.0, 1.0),
        ];
        for c in designs {
            let mut b = Biquad::new(c);
            let h32 = impulse_response(|x| b.process(x), 16_384);
            let mut b64 = BiquadF64::new(c);
            let h64 = impulse_response(|x| b64.process(x), 16_384);
            for f in test_freqs().filter(|&f| f > 30.0) {
                let w = 2.0 * PI * f / FS;
                let want = c.response_w(w).db();
                // f32 coefficient quantisation: deep stopband of a 100 Hz HPF below ~35 Hz deviates by
                // > 0.01 dB (use BiquadF64 there); check f32 from 40 Hz.
                if f > 40.0 {
                    assert!((dtft(&h32, w).db() - want).abs() < 0.01, "f32 f={f}");
                }
                assert!((dtft(&h64, w).db() - want).abs() < 0.001, "f64 f={f}");
            }
        }
        let s = butterworth(CutKind::Highpass, FS, 80.0, 8);
        let mut cas = Cascade::new(&s);
        let h = impulse_response(|x| cas.process(x), 32_768);
        for f in [40.0, 80.0, 160.0, 1000.0, 10_000.0] {
            let w = 2.0 * PI * f / FS;
            assert!((dtft(&h, w).db() - s.response(FS, f).db()).abs() < 0.01, "cascade f={f}");
        }
    }

    #[test]
    fn smoothed_biquad_ramps_to_target() {
        let a = Coeffs::peak(FS, 1000.0, 1.0, 0.0);
        let b = Coeffs::peak(FS, 4000.0, 3.0, 12.0);
        let mut sb = SmoothedBiquad::new(a, 64);
        sb.set_target(b);
        let mut max = 0.0f32;
        for i in 0..2000 {
            let y = sb.process((i as f32 * 0.3).sin());
            max = max.max(y.abs());
        }
        assert_eq!(sb.current(), coeff_array(b));
        assert!(max.is_finite() && max < 10.0);
    }

    #[test]
    fn denormal_flush() {
        let mut b = Biquad::new(Coeffs::lowpass(FS, 1000.0, 0.707));
        b.process(1e-20);
        b.flush_denormals();
        assert_eq!(b.process(0.0), 0.0);
    }
}
