//! Filter-tool models (T046, models.md §2): `TILT` (TILT EQ), `MAX` (MAXER), `AP1` (AP 90), `AP2` (AP 180).
//!
//! WING-Edit computes **every** tool filter as one biquad (4th section of `Cchfilter`, see
//! `data/reference-curves/filter.json`), so each model here is one stereo-linked 2nd-order section, designed in
//! f64 and run in f64 (poles at 100-160 Hz are too close to z = 1 for f32 coefficients):
//!
//! - **TILT** `tilt` -6..+6 dB (slot 1). Two real pole/zero pairs in the bilinear K-plane (`u = tan(w/2)`):
//!   `H(u) = (1/g) (u + g k1)(u + g k2) / ((u + k1)(u + k2))`, `g = 10^(tilt/20)`, `k1 = K1/sqrt(g)`,
//!   `k2 = K2/sqrt(g)`. DC gain = +tilt dB, Nyquist = -tilt dB, 0 dB at the geometric pivot ~ 656 Hz
//!   (`K0 = sqrt(K1 K2)`), shelves ~137 Hz / ~3.1 kHz. **Positive tilt = more low end** (the reference curves,
//!   not the "positive = brighter" guess of the STD-EQ tilt). K1/K2 are fitted to the reference
//!   (max 0.003 dB, 0.012 deg).
//! - **MAX** `low` (LOW CONT, slot 1), `proc` (HIGH PROC, slot 2), both 0..100 %. `H = (A(z) + proc*kappa*(1 -
//!   z^-1)^2 + low*c) / D(z)` where `D` has real poles at ~161 Hz and ~3.19 kHz and `A` is its mirror
//!   polynomial: at `low = proc = 0` the unit is a 2nd-order **all-pass** (the BBE-style low-band delay /
//!   phase realignment). `proc` adds an HF boost (0 dB at DC, about +10.8 dB at Nyquist for 100 %), `low`
//!   adds a constant to the numerator = LF contour boost (about +10.9 dB at DC for 100 %, tapering off above
//!   ~160 Hz). The coefficients are exact (linear in `low`, `proc`).
//! - **AP1** `f` 100..10000 Hz (slot 1): 1st-order all-pass, -90 deg at `f` (bilinear, pre-warped).
//! - **AP2** `f` (slot 1), `q` 0.5..5 (slot 2): RBJ-form 2nd-order all-pass but with `alpha = tan(w0/(2Q))`
//!   instead of `sin(w0)/(2Q)` (verified on all 20 reference curves; differs visibly near Nyquist).
//!
//! **Linear vs not:** all four are LTI. The BBE Sonic Maximizer's level-dependent processing (VCA-driven HF
//! boost) is *not* in WING's MAX: the curves show a fixed linear filter, so none is modelled (`proc` boosts
//! the same amount at every level).
//!
//! Smoothing: the target coefficient set is recomputed at block rate when a parameter changed and the live
//! coefficients ramp linearly per sample over [`RAMP_S`] (stable between two stable sets: convex stability
//! region). No allocation after construction; `prepare` only stores the sample rate.

use crate::model::{Model, ParamValue};
use core::f64::consts::PI;
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

/// Reference sample rate of the fitted constants (the curves are exported at 48 kHz).
const REF_FS: f64 = 48_000.0;

// TILT: K-plane (tan(w/2) at 48 kHz) of the low / high pole pair at tilt = 0, fitted to filter.json.
const TILT_K1: f64 = 0.008_962_1;
const TILT_K2: f64 = 0.205_741_8;
// MAX: K-plane poles at 48 kHz (~160.9 Hz, ~3190 Hz), HF-process and LF-contour slopes per percent.
const MAX_K1: f64 = 0.010_530_298;
const MAX_K2: f64 = 0.211_897_16;
const MAX_KAPPA: f64 = 0.020_15;
const MAX_LOW: f64 = 1.822_03e-4;

/// Which tool filter.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ToolKind {
    Tilt,
    Max,
    Ap1,
    Ap2,
}

/// `K = tan(pi f / fs)` at `fs` of the analog frequency that maps to `k48` at 48 kHz.
fn rescale_k(k48: f64, fs: f64) -> f64 {
    if (fs - REF_FS).abs() < 1e-9 {
        return k48;
    }
    let f = k48.atan() * REF_FS / PI;
    (PI * f.min(fs * 0.499) / fs).tan()
}

/// TILT EQ section, `tilt_db` -6..+6 (positive = low boost / high cut).
pub fn tilt_coeffs(fs: f64, tilt_db: f64) -> Coeffs {
    let g = 10f64.powf(tilt_db.clamp(-6.0, 6.0) / 20.0);
    let sg = g.sqrt();
    let k1 = rescale_k(TILT_K1, fs) / sg;
    let k2 = rescale_k(TILT_K2, fs) / sg;
    // (u + K) -> [(1+K) - (1-K) z^-1] / (1 + z^-1)
    let (n1a, n1b) = (1.0 + g * k1, 1.0 - g * k1);
    let (n2a, n2b) = (1.0 + g * k2, 1.0 - g * k2);
    let (d1a, d1b) = (1.0 + k1, 1.0 - k1);
    let (d2a, d2b) = (1.0 + k2, 1.0 - k2);
    let gi = 1.0 / g;
    let a0 = d1a * d2a;
    Coeffs {
        b0: gi * n1a * n2a / a0,
        b1: -gi * (n1a * n2b + n1b * n2a) / a0,
        b2: gi * n1b * n2b / a0,
        a1: -(d1a * d2b + d1b * d2a) / a0,
        a2: d1b * d2b / a0,
    }
}

/// MAXER section, `low` (LOW CONT) and `proc` (HIGH PROC) in percent.
pub fn max_coeffs(fs: f64, low_pct: f64, proc_pct: f64) -> Coeffs {
    let (low, pr) = (low_pct.clamp(0.0, 100.0), proc_pct.clamp(0.0, 100.0));
    let (k1, k2) = (rescale_k(MAX_K1, fs), rescale_k(MAX_K2, fs));
    let (p1, p2) = ((1.0 - k1) / (1.0 + k1), (1.0 - k2) / (1.0 + k2));
    let (a1, a2) = (-(p1 + p2), p1 * p2);
    // Terms are invariant in the K-plane, so rescale by the (1+K1)(1+K2) normaliser ratio.
    let s = (1.0 + MAX_K1) * (1.0 + MAX_K2) / ((1.0 + k1) * (1.0 + k2));
    let hf = pr * MAX_KAPPA * s;
    Coeffs {
        b0: a2 + hf + low * MAX_LOW * s,
        b1: a1 - 2.0 * hf,
        b2: 1.0 + hf,
        a1,
        a2,
    }
}

/// AP 90: first-order all-pass `(a + z^-1)/(1 + a z^-1)`, `a = (K-1)/(K+1)`.
pub fn ap1_coeffs(fs: f64, f: f64) -> Coeffs {
    let f = f.clamp(AP_F_MIN as f64, AP_F_MAX as f64).min(fs * 0.499);
    let k = (PI * f / fs).tan();
    let a = (k - 1.0) / (k + 1.0);
    Coeffs {
        b0: a,
        b1: 1.0,
        b2: 0.0,
        a1: a,
        a2: 0.0,
    }
}

/// AP 180: second-order all-pass, `alpha = tan(w0/(2Q))`.
pub fn ap2_coeffs(fs: f64, f: f64, q: f64) -> Coeffs {
    let f = f.clamp(AP_F_MIN as f64, AP_F_MAX as f64).min(fs * 0.499);
    let q = q.clamp(AP_Q_MIN as f64, AP_Q_MAX as f64);
    let w0 = 2.0 * PI * f / fs;
    let al = (w0 / (2.0 * q)).tan();
    let c = -2.0 * w0.cos() / (1.0 + al);
    let r = (1.0 - al) / (1.0 + al);
    Coeffs {
        b0: r,
        b1: c,
        b2: 1.0,
        a1: c,
        a2: r,
    }
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
            fs: REF_FS,
            p1: 0.0,
            p2: 0.0,
            dirty: false,
            cur: arr(Coeffs::IDENTITY),
            step: [0.0; 5],
            target: arr(Coeffs::IDENTITY),
            remaining: 0,
            ramp_len: (RAMP_S * REF_FS) as u32,
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
        assert!(z.magnitude_db(48_000.0, 3000.0).abs() < 1e-9);
    }

    #[test]
    fn max_is_allpass_when_zeroed_and_linear_in_params() {
        let c = max_coeffs(48_000.0, 0.0, 0.0);
        for f in [50.0, 500.0, 5000.0, 20_000.0] {
            assert!(c.magnitude_db(48_000.0, f).abs() < 1e-9);
        }
        let hi = max_coeffs(48_000.0, 0.0, 100.0);
        assert!(hi.magnitude_db(48_000.0, 20.0).abs() < 0.2);
        assert!(hi.magnitude_db(48_000.0, 20_000.0) > 8.0);
        let lo = max_coeffs(48_000.0, 100.0, 0.0);
        assert!(lo.magnitude_db(48_000.0, 20.0) > 9.0);
        let mid = max_coeffs(48_000.0, 0.0, 50.0);
        assert!((mid.b1 - (hi.b1 + c.b1) / 2.0).abs() < 1e-12);
    }

    #[test]
    fn allpass_flat_and_phase() {
        let a1 = ap1_coeffs(48_000.0, 1000.0);
        let a2 = ap2_coeffs(48_000.0, 1000.0, 2.5);
        for f in [30.0, 1000.0, 15_000.0] {
            assert!(a1.magnitude_db(48_000.0, f).abs() < 1e-9);
            assert!(a2.magnitude_db(48_000.0, f).abs() < 1e-9);
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
