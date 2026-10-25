//! Gain laws (fader taper, pan, width) and block-rate coefficient ramps.
//!
//! Every gain the mix applies is expressed as a 2x2 stereo matrix `[a, b, c, d]` (`L' = a L + b R`,
//! `R' = c L + d R`), so fader, pan, width, mono-sum and send level fold into one set of coefficients
//! per signal path. Changes are smoothed by [`Ramp`]: a one-pole approach evaluated once per block
//! (time constant [`MIX_TAU_S`]) with linear interpolation inside the block, i.e. no zipper noise and
//! no per-sample transcendental math.

use std::f32::consts::{FRAC_PI_4, SQRT_2};

/// Fader / send level value that means -oo (the schema's `fader` minimum).
pub const FADER_OFF_DB: f32 = -144.0;

/// Smoothing time constant of every mix gain (fader, mute, pan, width, sends).
pub const MIX_TAU_S: f64 = 0.005;

/// Linear gain of a WING fader / send level in dB (`fdr`, `lvl`: -144 = -oo .. +10 dB). The
/// position <-> dB taper (`CparLevel`) is wing-model's business ([`wing_model::fader_pos_to_db`]);
/// the store carries dB, so the audio gain is plain `10^(dB/20)` with -oo at the bottom.
#[inline]
pub fn fader_gain(db: f32) -> f32 {
    if db <= FADER_OFF_DB + 0.01 || db.is_nan() {
        0.0
    } else {
        10f32.powf(db.min(10.0) / 20.0)
    }
}

/// Pan law. WING's law was not recovered from WING-Edit (signal-flow.md §2, open question 7), so
/// the choice per strip type is configurable ([`super::MixConfig`]) and every default is [I].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PanLaw {
    /// Constant power, -3 dB at centre: `gL = cos t`, `gR = sin t`, `t = (p+1) pi/4`. Default for
    /// channel/aux pans and channel send pans [I].
    ConstPower,
    /// Balance, 0 dB at centre, constant-power taper of the far side (`min(1, sqrt2 cos t)`).
    /// Default for bus/main/matrix pans (a stereo master must be unity at centre) [I].
    Balance,
    /// Linear balance, 0 dB at centre, far side `1 - |p|`.
    LinearBalance,
}

impl PanLaw {
    /// `(gL, gR)` for a pan of -100 (left) .. +100 (right).
    #[inline]
    pub fn gains(self, pan: f32) -> (f32, f32) {
        let p = (pan / 100.0).clamp(-1.0, 1.0);
        match self {
            PanLaw::ConstPower => {
                let t = (p + 1.0) * FRAC_PI_4;
                (t.cos(), t.sin())
            }
            PanLaw::Balance => {
                let t = (p + 1.0) * FRAC_PI_4;
                let l = if p <= 0.0 { 1.0 } else { SQRT_2 * t.cos() };
                let r = if p >= 0.0 { 1.0 } else { SQRT_2 * t.sin() };
                (l, r)
            }
            PanLaw::LinearBalance => ((1.0 - p).min(1.0), (1.0 + p).min(1.0)),
        }
    }
}

/// A stereo gain matrix `[a, b, c, d]`: `L' = a L + b R`, `R' = c L + d R`.
pub type Matrix = [f32; 4];

/// Identity matrix.
pub const IDENTITY: Matrix = [1.0, 0.0, 0.0, 1.0];
/// Silence.
pub const ZERO: Matrix = [0.0; 4];

/// Scale a matrix.
#[inline]
pub fn scale(m: Matrix, g: f32) -> Matrix {
    [m[0] * g, m[1] * g, m[2] * g, m[3] * g]
}

/// `x * y` (apply `y` first).
#[inline]
pub fn mul(x: Matrix, y: Matrix) -> Matrix {
    [
        x[0] * y[0] + x[1] * y[2],
        x[0] * y[1] + x[1] * y[3],
        x[2] * y[0] + x[3] * y[2],
        x[2] * y[1] + x[3] * y[3],
    ]
}

/// Stereo width (`wid` -150..+150 %): mid/side scaling `M = (L+R)/2`, `S = (L-R)/2`,
/// `L' = M + w S`, `R' = M - w S` with `w = wid/100`. 100 % = unchanged, 0 % = mono, negative =
/// swapped sides, > 100 % = wider [I]. A mono source (L = R) is unaffected.
#[inline]
pub fn width(wid_pct: f32) -> Matrix {
    let w = (wid_pct / 100.0).clamp(-1.5, 1.5);
    let (s, d) = (0.5 * (1.0 + w), 0.5 * (1.0 - w));
    [s, d, d, s]
}

/// Pan gains as a diagonal matrix.
#[inline]
pub fn pan(law: PanLaw, pan: f32) -> Matrix {
    let (l, r) = law.gains(pan);
    [l, 0.0, 0.0, r]
}

/// Sum to mono (`busmono`): both sides carry `(L + R) / 2` [I].
pub const MONO: Matrix = [0.5, 0.5, 0.5, 0.5];

/// Width then pan (the strip order: pan/width after the fader).
#[inline]
pub fn pan_width(law: PanLaw, p: f32, wid_pct: f32) -> Matrix {
    mul(pan(law, p), width(wid_pct))
}

/// Per-block alpha of a one-pole with time constant `tau_s` (1 = jump).
pub fn block_alpha(tau_s: f64, fs: f64, block: usize) -> f32 {
    if tau_s <= 0.0 || fs <= 0.0 {
        return 1.0;
    }
    (1.0 - (-(block as f64) / (tau_s * fs)).exp()) as f32
}

/// Snap threshold of [`Ramp`] (-100 dBFS).
const SNAP: f32 = 1e-5;

/// `N` smoothed coefficients. Once per block [`Ramp::step`] moves the current values a one-pole
/// step towards the targets and returns the block's start and end values (interpolate linearly
/// inside the block). The first step after construction or [`Ramp::settle`] jumps.
#[derive(Clone, Copy, Debug)]
pub struct Ramp<const N: usize> {
    cur: [f32; N],
    tgt: [f32; N],
    alpha: f32,
    primed: bool,
}

impl<const N: usize> Ramp<N> {
    pub fn new(alpha: f32) -> Self {
        Self {
            cur: [0.0; N],
            tgt: [0.0; N],
            alpha,
            primed: false,
        }
    }

    #[inline]
    pub fn set(&mut self, tgt: [f32; N]) {
        self.tgt = tgt;
    }

    #[inline]
    pub fn target(&self) -> &[f32; N] {
        &self.tgt
    }

    #[inline]
    pub fn current(&self) -> &[f32; N] {
        &self.cur
    }

    /// Jump to the targets at the next step.
    pub fn settle(&mut self) {
        self.primed = false;
    }

    /// Targets reached (nothing moving).
    #[inline]
    pub fn is_settled(&self) -> bool {
        self.cur == self.tgt
    }

    /// Current and target all zero: the path is silent.
    #[inline]
    pub fn is_silent(&self) -> bool {
        self.cur.iter().chain(self.tgt.iter()).all(|&v| v == 0.0)
    }

    /// Advance one block; returns `(start, end)` coefficients.
    #[inline]
    pub fn step(&mut self) -> ([f32; N], [f32; N]) {
        if !self.primed {
            self.primed = true;
            self.cur = self.tgt;
            return (self.cur, self.cur);
        }
        let start = self.cur;
        if start != self.tgt {
            for (c, &t) in self.cur.iter_mut().zip(&self.tgt) {
                let d = t - *c;
                *c = if d.abs() <= SNAP {
                    t
                } else {
                    *c + d * self.alpha
                };
            }
        }
        (start, self.cur)
    }
}

/// `out = g * x` with `g` ramping linearly from `g0` to `g1` across the block.
#[inline]
pub fn gain_write(out: &mut [f32], x: &[f32], g0: f32, g1: f32) {
    if g0 == g1 {
        if g0 == 0.0 {
            out.fill(0.0);
        } else {
            for (o, &v) in out.iter_mut().zip(x) {
                *o = v * g0;
            }
        }
        return;
    }
    let inc = (g1 - g0) / out.len().max(1) as f32;
    for (i, (o, &v)) in out.iter_mut().zip(x).enumerate() {
        *o = v * (g0 + inc * (i + 1) as f32);
    }
}

/// `(ol, or) = m * (l, r)`, `m` ramping from `m0` to `m1` (write).
#[inline]
pub fn matrix_write(ol: &mut [f32], or: &mut [f32], l: &[f32], r: &[f32], m0: Matrix, m1: Matrix) {
    let n = ol.len();
    if m0 == m1 {
        let [a, b, c, d] = m0;
        if b == 0.0 && c == 0.0 {
            gain_write(ol, l, a, a);
            gain_write(or, r, d, d);
            return;
        }
        for i in 0..n {
            let (x, y) = (l[i], r[i]);
            ol[i] = a * x + b * y;
            or[i] = c * x + d * y;
        }
        return;
    }
    let k = 1.0 / n.max(1) as f32;
    let inc = [
        (m1[0] - m0[0]) * k,
        (m1[1] - m0[1]) * k,
        (m1[2] - m0[2]) * k,
        (m1[3] - m0[3]) * k,
    ];
    for i in 0..n {
        let t = (i + 1) as f32;
        let (x, y) = (l[i], r[i]);
        ol[i] = (m0[0] + inc[0] * t) * x + (m0[1] + inc[1] * t) * y;
        or[i] = (m0[2] + inc[2] * t) * x + (m0[3] + inc[3] * t) * y;
    }
}

/// `(ol, or) += m * (l, r)`, `m` ramping from `m0` to `m1` (accumulate).
#[inline]
pub fn matrix_add(ol: &mut [f32], or: &mut [f32], l: &[f32], r: &[f32], m0: Matrix, m1: Matrix) {
    let n = ol.len().min(l.len()).min(r.len()).min(or.len());
    let (ol, or, l, r) = (&mut ol[..n], &mut or[..n], &l[..n], &r[..n]);
    if m0 == m1 {
        let [a, b, c, d] = m0;
        if b == 0.0 && c == 0.0 {
            for i in 0..n {
                ol[i] += a * l[i];
                or[i] += d * r[i];
            }
        } else {
            for i in 0..n {
                let (x, y) = (l[i], r[i]);
                ol[i] += a * x + b * y;
                or[i] += c * x + d * y;
            }
        }
        return;
    }
    let k = 1.0 / n.max(1) as f32;
    let inc = [
        (m1[0] - m0[0]) * k,
        (m1[1] - m0[1]) * k,
        (m1[2] - m0[2]) * k,
        (m1[3] - m0[3]) * k,
    ];
    for i in 0..n {
        let t = (i + 1) as f32;
        let (x, y) = (l[i], r[i]);
        ol[i] += (m0[0] + inc[0] * t) * x + (m0[1] + inc[1] * t) * y;
        or[i] += (m0[2] + inc[2] * t) * x + (m0[3] + inc[3] * t) * y;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn db(x: f32) -> f32 {
        20.0 * x.log10()
    }

    #[test]
    fn fader_taper() {
        assert_eq!(fader_gain(-144.0), 0.0);
        assert_eq!(fader_gain(0.0), 1.0);
        assert!((db(fader_gain(10.0)) - 10.0).abs() < 1e-4);
        assert!((db(fader_gain(-60.0)) + 60.0).abs() < 1e-3);
        assert!((db(fader_gain(-143.0)) + 143.0).abs() < 1e-2);
        // the schema's taper maps positions to these dB values (wing-model owns it)
        assert!((wing_model::fader_pos_to_db(0.75) - 0.0).abs() < 1e-4);
        assert_eq!(fader_gain(wing_model::fader_pos_to_db(0.0)), 0.0);
    }

    #[test]
    fn pan_laws() {
        let (l, r) = PanLaw::ConstPower.gains(0.0);
        assert!((db(l) + 3.0103).abs() < 1e-3 && (l - r).abs() < 1e-6);
        for p in [-100.0, -37.0, 0.0, 12.0, 100.0] {
            let (l, r) = PanLaw::ConstPower.gains(p);
            assert!((l * l + r * r - 1.0).abs() < 1e-5, "constant power at {p}");
        }
        assert_eq!(PanLaw::ConstPower.gains(-100.0).0, 1.0);
        assert!(PanLaw::ConstPower.gains(-100.0).1.abs() < 1e-7);
        assert_eq!(PanLaw::Balance.gains(0.0), (1.0, 1.0));
        let (l, r) = PanLaw::Balance.gains(50.0);
        assert!((r - 1.0).abs() < 1e-6);
        assert!((l - (SQRT_2 * (1.5 * FRAC_PI_4).cos())).abs() < 1e-6);
        assert!(PanLaw::Balance.gains(100.0).0.abs() < 1e-6);
        assert_eq!(PanLaw::LinearBalance.gains(-50.0), (1.0, 0.5));
        // out-of-range clamps
        assert_eq!(
            PanLaw::ConstPower.gains(250.0),
            PanLaw::ConstPower.gains(100.0)
        );
    }

    #[test]
    fn width_and_matrices() {
        assert_eq!(width(100.0), IDENTITY);
        assert_eq!(width(0.0), MONO);
        assert_eq!(width(-100.0), [0.0, 1.0, 1.0, 0.0]);
        // a mono signal (L = R) is unaffected by width
        let m = width(150.0);
        assert!((m[0] + m[1] - 1.0).abs() < 1e-6);
        // pan_width = pan after width
        let pw = pan_width(PanLaw::ConstPower, 30.0, 50.0);
        let (gl, gr) = PanLaw::ConstPower.gains(30.0);
        assert!((pw[0] - gl * 0.75).abs() < 1e-6 && (pw[1] - gl * 0.25).abs() < 1e-6);
        assert!((pw[2] - gr * 0.25).abs() < 1e-6 && (pw[3] - gr * 0.75).abs() < 1e-6);
        assert_eq!(mul(IDENTITY, pw), pw);
        assert_eq!(scale(IDENTITY, 0.5), [0.5, 0.0, 0.0, 0.5]);
    }

    #[test]
    fn ramp_smooths_and_settles() {
        let a = block_alpha(MIX_TAU_S, 48_000.0, 64);
        assert!((a - (1.0 - (-64.0f64 / 240.0).exp()) as f32).abs() < 1e-6);
        let mut r = Ramp::<1>::new(a);
        r.set([1.0]);
        assert_eq!(r.step(), ([1.0], [1.0])); // first step jumps
        r.set([0.0]);
        let (s, e) = r.step();
        assert_eq!(s, [1.0]);
        assert!(e[0] < 1.0 && e[0] > 0.5);
        let mut n = 0;
        while !r.is_settled() {
            r.step();
            n += 1;
            assert!(n < 200);
        }
        assert_eq!(r.current(), &[0.0]);
        assert!(r.is_silent());
        // ~5 tau to -100 dB: 11.5 tau = 2760 samples = 43 blocks
        assert!(n > 30 && n < 60, "{n}");
    }

    #[test]
    fn interpolated_writes() {
        let x = [1.0f32; 4];
        let mut o = [0.0f32; 4];
        gain_write(&mut o, &x, 0.0, 1.0);
        assert_eq!(o, [0.25, 0.5, 0.75, 1.0]);
        let (mut ol, mut or) = ([0.0f32; 4], [0.0f32; 4]);
        matrix_write(
            &mut ol,
            &mut or,
            &x,
            &[2.0; 4],
            [1.0, 1.0, 0.0, 1.0],
            [1.0, 1.0, 0.0, 1.0],
        );
        assert_eq!((ol, or), ([3.0; 4], [2.0; 4]));
        matrix_add(&mut ol, &mut or, &x, &x, ZERO, IDENTITY);
        assert_eq!(ol, [3.25, 3.5, 3.75, 4.0]);
        assert_eq!(or, [2.25, 2.5, 2.75, 3.0]);
    }
}
