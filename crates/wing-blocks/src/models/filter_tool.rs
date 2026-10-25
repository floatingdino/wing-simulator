//! Filter-tool models (T046, models.md §2): `TILT` (TILT EQ), `MAX` (MAXER), `AP1` (AP 90), `AP2` (AP 180).
//!
//! WING-Edit computes **every** tool filter as one biquad (4th section of `Cchfilter`, see
//! `data/reference-curves/filter.json`). Since T04J the designs are a port of `ae::Cchfilter::update`
//! (0x1000111e4..0x1000114f4, float32 arithmetic and fused multiply-adds as in the arm64 code), so the
//! coefficients equal WING-Edit's at every sample rate (checked at 48 and 44.1 kHz); T046 had fitted 48 kHz
//! constants. Each model is one stereo-linked 2nd-order section run in f64 (poles at 100-160 Hz are too close to
//! z = 1 for an f32 filter state):
//!
//! - **TILT** `tilt` -6..+6 dB (slot 1): `10^(tilt/20) * hs12(fc, -2 tilt, Q 0.2)` (WING's bilinear high shelf,
//!   [`crate::wingcoef::hs12`]), `fc` interpolated 650 -> 780 -> 925 Hz for |tilt| 0 -> 3 -> 6 dB. DC = +tilt,
//!   Nyquist = -tilt: **positive tilt = more low end**.
//! - **MAX** `low` (LOW CONT, slot 1), `proc` (HIGH PROC, slot 2), both 0..100 %. Poles from
//!   `s1 = 2 sin(pi 252.4/fs)`, `s2 = 2 sin(pi 1688.7/fs)`; at `low = proc = 0` the section is a 2nd-order
//!   **all-pass** (BBE-style low-band phase realignment); `proc` raises `b2 = 1 + 0.02015 proc` (HF boost, about
//!   +10.8 dB at Nyquist for 100 %), `low` scales the `s1 s2` numerator term by `1 + 0.025 low` (LF contour).
//! - **AP1** `f` 100..10000 Hz (slot 1): 1st-order all-pass, `K = tanf(pi f/fs)`, -90 deg at `f`.
//! - **AP2** `f` (slot 1), `q` 0.5..5 (slot 2): 2nd-order all-pass with `alpha = tanf(w0/(2Q))` instead of RBJ's
//!   `sin(w0)/(2Q)`.
//!
//! **Linear vs not:** all four are LTI. The BBE Sonic Maximizer's level-dependent processing (VCA-driven HF
//! boost) is *not* in WING's MAX: the curves show a fixed linear filter, so none is modelled (`proc` boosts
//! the same amount at every level).
//!
//! Smoothing: the target coefficient set is recomputed at block rate when a parameter changed and the live
//! coefficients ramp linearly per sample over [`RAMP_S`] (stable between two stable sets: convex stability
//! region). No allocation after construction; `prepare` only stores the sample rate.

use crate::model::{Model, ParamValue};
use crate::wingcoef::{self, Bq, Ctx};
use wing_dsp::biquad::Coeffs;

/// Coefficient ramp time after a parameter change.
pub const RAMP_S: f64 = 0.010;

pub const TILT_MAX_DB: f32 = 6.0;
pub const AP_F_MIN: f32 = 100.0;
pub const AP_F_MAX: f32 = 10_000.0;
pub const AP_Q_MIN: f32 = 0.5;
pub const AP_Q_MAX: f32 = 5.0;
pub const AP_Q_DEFAULT: f32 = 2.5;
pub const AP_F_DEFAULT: f32 = 1000.0;

// `ae::Cchfilter::update` constants (T04J port, see the module doc).
const TILT_Q: f32 = f32::from_bits(0x3e4c_cccd); // 0.2
const LN10_20: f64 = f64::from_bits(0x3fbd_791c_6109_b02d);
const MAX_F1: f64 = f64::from_bits(0x406f_8ccc_cccc_cccd); // 252.4 Hz
const MAX_F2: f64 = f64::from_bits(0x409a_62cc_cccc_cccd); // 1688.7 Hz
const MAX_K: f64 = f64::from_bits(0xbffa_589b_7694_20e2); // -1.6466
const MAX_C: f64 = f64::from_bits(0x3fe3_6f00_68db_8bac); // 0.6073
const MAX_PROC: f64 = f64::from_bits(0x4000_1eb8_51eb_851f); // 2.015
const AP2_NYQ: f64 = f64::from_bits(0x3fdf_5c28_f5c2_8f5c); // 0.49

fn coeffs(q: Bq) -> Coeffs {
    let [b0, b1, b2, a1, a2] = q.map(f64::from);
    Coeffs { b0, b1, b2, a1, a2 }
}

/// Which tool filter.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ToolKind {
    Tilt,
    Max,
    Ap1,
    Ap2,
}

/// TILT EQ section, `tilt_db` -6..+6 (positive = low boost / high cut): `10^(t/20) * hs12(fc, -2t, 0.2)`, `fc`
/// interpolated 650 -> 780 -> 925 Hz over |t| = 0 -> 3 -> 6 dB.
pub fn tilt_coeffs(fs: f64, tilt_db: f64) -> Coeffs {
    let c = Ctx::new(fs);
    let t = (tilt_db as f32).clamp(-TILT_MAX_DB, TILT_MAX_DB);
    let x = ((t.abs() + t.abs()) / 6.0).min(2.0);
    let fc = if x >= 1.0 {
        (x - 1.0).mul_add(925.0, (2.0 - x) * 780.0)
    } else {
        x.mul_add(780.0, (1.0 - x) * 650.0)
    };
    let [b0, b1, b2, a1, a2] = wingcoef::hs12(&c, fc, t * -2.0, TILT_Q);
    let g = if t > -98.0 {
        (t as f64 * LN10_20).exp() as f32
    } else {
        0.0
    };
    coeffs([g * b0, g * b1, g * b2, a1, a2])
}

/// MAXER section, `low` (LOW CONT) and `proc` (HIGH PROC) in percent: an all-pass at 0/0 (poles from
/// `2 sin(pi 252.4/fs)` and `2 sin(pi 1688.7/fs)`), `proc` raises `b2` (HF), `low` the LF contour term.
pub fn max_coeffs(fs: f64, low_pct: f64, proc_pct: f64) -> Coeffs {
    let (low, pr) = (low_pct.clamp(0.0, 100.0), proc_pct.clamp(0.0, 100.0));
    let pifs = Ctx::new(fs).pi_over_fs() as f64;
    let s11 = (2.0 * (pifs * MAX_F1).sin()) as f32;
    let s0 = (2.0 * (pifs * MAX_F2).sin()) as f32;
    let s1 = (MAX_K - s11 as f64) as f32;
    let lo = ((low as f32) as f64 * 2.5 / 100.0 + 1.0) as f32;
    let b2 = ((pr as f32) as f64 * MAX_PROC / 100.0 + 1.0) as f32;
    let b0 = s0.mul_add(s1, (lo * s11).mul_add(s0, b2));
    let b1 = b2.mul_add(-2.0, -s0 * s1);
    let d = s0 as f64 / MAX_C;
    let a1 = (d + s11.mul_add(s0, -2.0) as f64) as f32;
    let a2 = (1.0 - d) as f32;
    coeffs([b0, b1, b2, a1, a2])
}

/// AP 90: first-order all-pass `(a + z^-1)/(1 + a z^-1)`, `a = -(1-K)/(1+K)`, `K = tanf(pi f/fs)`.
pub fn ap1_coeffs(fs: f64, f: f64) -> Coeffs {
    let c = Ctx::new(fs);
    let f = (f as f32).clamp(AP_F_MIN, AP_F_MAX);
    let k = (c.pi_over_fs() * f).tan();
    let r = (1.0 - k) / (1.0 + k);
    coeffs([-r, 1.0, 0.0, -r, 0.0])
}

/// AP 180: second-order all-pass with `alpha = tanf(w0/(2Q))` (`r = (1-alpha)/(1+alpha)`, `cosf(w0)`).
pub fn ap2_coeffs(fs: f64, f: f64, q: f64) -> Coeffs {
    let c = Ctx::new(fs);
    let pifs = c.pi_over_fs();
    let q = (q as f32).clamp(AP_Q_MIN, AP_Q_MAX);
    let mut f = (f as f32).clamp(AP_F_MIN, AP_F_MAX);
    // WING clamps f to 0.49 q fs (T = 1/fs on the console; never reached inside the parameter ranges).
    if ((1.0 / fs) as f32 * f / q) as f64 >= AP2_NYQ {
        f = (q as f64 * AP2_NYQ * fs as f32 as f64) as f32;
    }
    let cs = (f * (pifs + pifs)).cos();
    let t = (f * pifs / q).tan();
    let r = (1.0 - t) / (1.0 + t);
    let r1 = r + 1.0;
    coeffs([r, r1 * -cs, 1.0, -(cs * r1), r])
}

/// One of the four tool filters as a selectable [`Model`].
pub struct FilterToolModel {
    kind: ToolKind,
    id: &'static str,
    fs: f64,
    /// Staged parameters: TILT tilt | MAX low | AP f (`p1`); MAX proc | AP2 q (`p2`).
    p1: f32,
    p2: f32,
    dirty: bool,
    cur: [f64; 5],
    step: [f64; 5],
    target: [f64; 5],
    remaining: u32,
    ramp_len: u32,
    /// TDF-II state per channel.
    s: [[f64; 2]; 2],
}

fn arr(c: Coeffs) -> [f64; 5] {
    [c.b0, c.b1, c.b2, c.a1, c.a2]
}

impl FilterToolModel {
    fn new(kind: ToolKind, id: &'static str) -> Self {
        let mut m = Self {
            kind,
            id,
            fs: 48_000.0,
            p1: 0.0,
            p2: 0.0,
            dirty: false,
            cur: arr(Coeffs::IDENTITY),
            step: [0.0; 5],
            target: arr(Coeffs::IDENTITY),
            remaining: 0,
            ramp_len: (RAMP_S * 48_000.0) as u32,
            s: [[0.0; 2]; 2],
        };
        m.load_defaults();
        m.snap();
        m
    }

    pub fn tilt() -> Self {
        Self::new(ToolKind::Tilt, "TILT")
    }
    pub fn maxer() -> Self {
        Self::new(ToolKind::Max, "MAX")
    }
    pub fn ap1() -> Self {
        Self::new(ToolKind::Ap1, "AP1")
    }
    pub fn ap2() -> Self {
        Self::new(ToolKind::Ap2, "AP2")
    }

    pub fn kind(&self) -> ToolKind {
        self.kind
    }

    /// Coefficients for the staged parameters at the current sample rate.
    pub fn design(&self) -> Coeffs {
        let (a, b) = (self.p1 as f64, self.p2 as f64);
        match self.kind {
            ToolKind::Tilt => tilt_coeffs(self.fs, a),
            ToolKind::Max => max_coeffs(self.fs, a, b),
            ToolKind::Ap1 => ap1_coeffs(self.fs, a),
            ToolKind::Ap2 => ap2_coeffs(self.fs, a, b),
        }
    }

    fn snap(&mut self) {
        self.target = arr(self.design());
        self.cur = self.target;
        self.remaining = 0;
        self.dirty = false;
    }

    fn retarget(&mut self) {
        self.dirty = false;
        self.target = arr(self.design());
        let inv = 1.0 / self.ramp_len as f64;
        for i in 0..5 {
            self.step[i] = (self.target[i] - self.cur[i]) * inv;
        }
        self.remaining = self.ramp_len;
    }
}

impl Model for FilterToolModel {
    fn id(&self) -> &'static str {
        self.id
    }

    fn prepare(&mut self, sample_rate: f64, _max_block: usize) {
        self.fs = sample_rate;
        self.ramp_len = ((RAMP_S * sample_rate) as u32).max(1);
        self.snap();
        self.s = [[0.0; 2]; 2];
    }

    fn process(&mut self, left: &mut [f32], right: &mut [f32], _sc: Option<(&[f32], &[f32])>) {
        if self.dirty {
            self.retarget();
        }
        let n = left.len().min(right.len());
        for i in 0..n {
            if self.remaining > 0 {
                self.remaining -= 1;
                if self.remaining == 0 {
                    self.cur = self.target;
                } else {
                    for k in 0..5 {
                        self.cur[k] += self.step[k];
                    }
                }
            }
            let [b0, b1, b2, a1, a2] = self.cur;
            let x = left[i] as f64;
            let y = b0 * x + self.s[0][0];
            self.s[0][0] = b1 * x - a1 * y + self.s[0][1];
            self.s[0][1] = b2 * x - a2 * y;
            left[i] = y as f32;
            let x = right[i] as f64;
            let y = b0 * x + self.s[1][0];
            self.s[1][0] = b1 * x - a1 * y + self.s[1][1];
            self.s[1][1] = b2 * x - a2 * y;
            right[i] = y as f32;
        }
        // Flush denormals out of the recursive state (decaying tails after silence).
        for st in self.s.iter_mut().flatten() {
            if st.abs() < 1e-30 {
                *st = 0.0;
            }
        }
    }

    fn reset(&mut self) {
        self.snap();
        self.s = [[0.0; 2]; 2];
    }

    fn set_param(&mut self, slot: u16, value: ParamValue) {
        let v = value.as_f32();
        if !v.is_finite() {
            return;
        }
        let ok = match (self.kind, slot) {
            (ToolKind::Tilt, 1) => {
                self.p1 = v.clamp(-TILT_MAX_DB, TILT_MAX_DB);
                true
            }
            (ToolKind::Max, 1) => {
                self.p1 = v.clamp(0.0, 100.0);
                true
            }
            (ToolKind::Max, 2) => {
                self.p2 = v.clamp(0.0, 100.0);
                true
            }
            (ToolKind::Ap1 | ToolKind::Ap2, 1) => {
                self.p1 = v.clamp(AP_F_MIN, AP_F_MAX);
                true
            }
            (ToolKind::Ap2, 2) => {
                self.p2 = v.clamp(AP_Q_MIN, AP_Q_MAX);
                true
            }
            _ => false,
        };
        self.dirty |= ok;
    }

    fn param(&self, slot: u16) -> Option<f32> {
        match (self.kind, slot) {
            (_, 1) => Some(self.p1),
            (ToolKind::Max | ToolKind::Ap2, 2) => Some(self.p2),
            _ => None,
        }
    }

    fn load_defaults(&mut self) {
        (self.p1, self.p2) = match self.kind {
            ToolKind::Tilt | ToolKind::Max => (0.0, 0.0),
            ToolKind::Ap1 => (AP_F_DEFAULT, 0.0),
            ToolKind::Ap2 => (AP_F_DEFAULT, AP_Q_DEFAULT),
        };
        self.dirty = true;
    }

    fn commit(&mut self) {
        if self.dirty {
            self.retarget();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn tilt_endpoints_and_pivot() {
        let c = tilt_coeffs(48_000.0, 6.0);
        assert!((c.magnitude_db(48_000.0, 1.0) - 6.0).abs() < 0.05);
        assert!((c.magnitude_db(48_000.0, 23_999.0) + 6.0).abs() < 0.05);
        assert!(c.magnitude_db(48_000.0, 655.7).abs() < 0.01);
        let z = tilt_coeffs(48_000.0, 0.0);
        assert!(z.magnitude_db(48_000.0, 3000.0).abs() < 1e-4);
    }

    #[test]
    fn max_is_allpass_when_zeroed_and_linear_in_params() {
        let c = max_coeffs(48_000.0, 0.0, 0.0);
        // all-pass to float32 coefficient rounding
        for f in [50.0, 500.0, 5000.0, 20_000.0] {
            assert!(c.magnitude_db(48_000.0, f).abs() < 1e-4);
        }
        let hi = max_coeffs(48_000.0, 0.0, 100.0);
        assert!(hi.magnitude_db(48_000.0, 20.0).abs() < 0.2);
        assert!(hi.magnitude_db(48_000.0, 20_000.0) > 8.0);
        let lo = max_coeffs(48_000.0, 100.0, 0.0);
        assert!(lo.magnitude_db(48_000.0, 20.0) > 9.0);
        let mid = max_coeffs(48_000.0, 0.0, 50.0);
        assert!((mid.b1 - (hi.b1 + c.b1) / 2.0).abs() < 1e-6);
    }

    #[test]
    fn allpass_flat_and_phase() {
        let a1 = ap1_coeffs(48_000.0, 1000.0);
        let a2 = ap2_coeffs(48_000.0, 1000.0, 2.5);
        for f in [30.0, 1000.0, 15_000.0] {
            assert!(a1.magnitude_db(48_000.0, f).abs() < 1e-4);
            assert!(a2.magnitude_db(48_000.0, f).abs() < 1e-4);
        }
        let p1 = a1.response(48_000.0, 1000.0).arg().to_degrees();
        let p2 = a2.response(48_000.0, 1000.0).arg().to_degrees();
        assert!((p1 + 90.0).abs() < 0.2, "{p1}");
        assert!((p2.abs() - 180.0).abs() < 0.2, "{p2}");
    }

    #[test]
    fn defaults_tilt_is_identity() {
        let mut m = FilterToolModel::tilt();
        m.prepare(48_000.0, 64);
        let mut l = vec![0.5f32; 64];
        let mut r = l.clone();
        m.process(&mut l, &mut r, None);
        assert!(!m.is_stub());
        assert!(l.iter().all(|&v| (v - 0.5).abs() < 1e-6));
    }

    #[test]
    fn param_smoothing_is_continuous_and_stable() {
        let mut m = FilterToolModel::ap2();
        m.prepare(48_000.0, 64);
        let mut ph = 0.0f32;
        let mut max_jump = 0.0f32;
        let mut prev = 0.0f32;
        for blk in 0..400 {
            if blk == 100 {
                m.set_param(1, ParamValue::Float(8000.0));
                m.set_param(2, ParamValue::Float(0.5));
            }
            let mut l: Vec<f32> = (0..64)
                .map(|_| {
                    ph += 0.05;
                    ph.sin() * 0.5
                })
                .collect();
            let mut r = l.clone();
            m.process(&mut l, &mut r, None);
            for &v in &l {
                assert!(v.abs() < 1.5);
                max_jump = max_jump.max((v - prev).abs());
                prev = v;
            }
        }
        assert!(max_jump < 0.6, "{max_jump}");
    }

    #[test]
    fn sample_rate_scaling_keeps_k_plane_shape() {
        let a = tilt_coeffs(48_000.0, 4.0).magnitude_db(48_000.0, 500.0);
        let b = tilt_coeffs(96_000.0, 4.0).magnitude_db(96_000.0, 500.0);
        assert!((a - b).abs() < 0.05, "{a} {b}");
    }
}
