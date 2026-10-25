//! `E84` ("EVEN 84", Neve 1084): port of `ae::eq_parchg_neve84`.
//!
//! Slots: `g lf lg mf mg mq hf hg` (`g` -20..+20 dB output gain, stepped frequency selectors, knob gains).
//! Sections: LF 2nd-order shelf (carries the output gain), LF resonance peak, HF 2nd-order shelf, HF
//! resonance bell, MID band, unity. The LF and HF tables are indexed by the *pair* (LF selector, HF selector)
//! because the passive 1084 shelves interact; each entry holds (f, g, q1, q2) of the shelf plus (f, g, Q) of
//! its resonance section per gain column |g| = 0..5. Table gains are cut-signed (negated for boost).

use super::prims::*;
use super::tables::*;
use super::Layout;

pub(super) const DEFAULTS: [f32; 8] = [0.0, 1.0, 0.0, 3.0, 0.0, 0.0, 2.0, 0.0];

/// Gain position: `x = |g| m`, `3 + 0.4 x` above 5 (mix > 100 %), column = trunc, fraction.
fn pos(g: f32, m: f32) -> (usize, f32) {
    let mut x = g.abs() * m;
    if x > 5.0 {
        x = (x as f64).mul_add(0.4, 3.0) as f32;
    }
    let i = x as i32;
    (i.max(0) as usize, x - i as f32)
}

/// Interpolated value `v` of a 7-value column record (`stride` floats per column).
fn lv(tab: &[f32], base: usize, stride: usize, i: usize, t: f32, v: usize) -> f32 {
    fma(
        1.0 - t,
        tab[base + i * stride + v],
        t * tab[base + (i + 1) * stride + v],
    )
}

/// `-(1-t)*a - t*b` (WING-Edit's fused negation of [`lv`]).
fn lv_neg(tab: &[f32], base: usize, stride: usize, i: usize, t: f32, v: usize) -> f32 {
    (-(1.0 - t)).mul_add(
        tab[base + i * stride + v],
        -(t * tab[base + (i + 1) * stride + v]),
    )
}

/// The shelf of a band: `ls12q2`/`hs12q2` with cut-signed table gain.
#[allow(clippy::too_many_arguments)]
fn shelf(
    ctx: &Ctx,
    hi: bool,
    g: f32,
    tab: &[f32],
    base: usize,
    stride: usize,
    i: usize,
    t: f32,
) -> Bq {
    let f = lv(tab, base, stride, i, t, 0);
    let gv = lv(tab, base, stride, i, t, 1);
    let q1 = lv(tab, base, stride, i, t, 2);
    let q2 = lv(tab, base, stride, i, t, 3);
    let d = if hi { hs12q2 } else { ls12q2 };
    if g < 0.0 {
        d(ctx, f, gv, 1.0 / q1, 1.0 / q2)
    } else {
        d(ctx, f, -gv, 1.0 / q2, 1.0 / q1)
    }
}

/// Inline LF resonance section: `1 + (A-1) * bandpass`, bandwidth narrowed for cut (`Q * min(A,1)`).
fn lf_peak(ctx: &Ctx, f: f32, g: f32, q: f32) -> Bq {
    let a = db_lin_guard(g);
    let k = tan_w(ctx, f);
    let s1 = k / (a.min(1.0) * q);
    let k2 = k * k;
    let n = 1.0 / (k2 + (s1 + 1.0));
    let s5 = k2.mul_add(n, -n);
    let a2n = n * -(k2 + (1.0 - s1));
    let b0 = fma((a + -1.0) * s1, n, 1.0);
    let b1 = -(s5 * -2.0);
    let b2 = (s1 * -(a + -1.0)).mul_add(n, -a2n);
    let a1n = s5 * -2.0;
    [b0, b1, b2, -a1n, -a2n]
}

pub(super) fn design(ctx: &Ctx, p: &[f32], m: f32, out: &mut [Bq]) -> Layout {
    let (g, lf, lg, mf, mg, mq, hf, hg) = (p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7]);
    let lf = (lf as i32).clamp(0, 4);
    let mf = (mf as i32).clamp(0, 6);
    let hf = (hf as i32).clamp(0, 3);
    let combo = (4 * lf + 3 - hf) as usize * 49;

    // LF shelf (+ output gain on its numerator) and LF resonance.
    let (i, t) = pos(lg, m);
    out[0] = shelf(ctx, false, lg, &N84_LOSHVTAB, combo, 7, i, t);
    let a = db_lin_guard(g * m);
    for b in &mut out[0][..3] {
        *b *= a;
    }
    let f2 = lv(&N84_LOSHVTAB, combo, 7, i, t, 4);
    let g2 = if lg < 0.0 {
        lv(&N84_LOSHVTAB, combo, 7, i, t, 5)
    } else {
        lv_neg(&N84_LOSHVTAB, combo, 7, i, t, 5)
    };
    let q2 = lv(&N84_LOSHVTAB, combo, 7, i, t, 6);
    out[1] = lf_peak(ctx, f2, g2, q2);

    // HF shelf and HF resonance bell.
    let (i, t) = pos(hg, m);
    out[2] = shelf(ctx, true, hg, &N84_HISHVTAB, combo, 7, i, t);
    let f2 = lv(&N84_HISHVTAB, combo, 7, i, t, 4);
    let g2 = if hg < 0.0 {
        lv(&N84_HISHVTAB, combo, 7, i, t, 5)
    } else {
        lv_neg(&N84_HISHVTAB, combo, 7, i, t, 5)
    };
    let q2 = lv(&N84_HISHVTAB, combo, 7, i, t, 6);
    out[3] = hqpeq(ctx, f2, g2, q2);

    // MID: (selector, Q) record of 7 columns x (f, g, q1, q2).
    let (i, t) = pos(mg, m);
    let base = ((mq != 0.0) as usize | (mf as usize) << 1) * 28;
    out[4] = shelf(ctx, true, mg, &N84_PARA, base, 4, i, t);
    out[5] = UNITY;
    Layout::cascade(6)
}
