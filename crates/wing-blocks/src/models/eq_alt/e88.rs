//! `E88` ("EVEN 88 FORMANT", Neve 88RS channel EQ): port of `ae::eq_parchg_neve88rs`.
//!
//! Slots: `lf lg lq lt lmf lmg lmq hmf hmg hmq hf hg hq ht` (`lq/hq` LOW/HIGH, `lt/ht` BELL/SHELV).
//! Sections: LF (shelf or bell), LMF bell, HMF bell, HF (shelf or bell), unity, unity.
//! Bell frequencies are linear in the knob between three anchors; shelf frequencies come from gain-indexed
//! tables (LF: scaled by a knob factor and by `ratio^-1.4` for mix > 100 %; HF: five knob anchors).
//! Gain columns |g| = 0..4 step 1, then 4..5 spans two columns (the top of the gain law is steeper).

use super::prims::*;
use super::tables::*;
use super::Layout;

pub(super) const DEFAULTS: [f32; 14] = [
    2.0, 0.0, 0.0, 1.0, 3.0, 0.0, 5.0, 5.0, 0.0, 5.0, 7.0, 0.0, 0.0, 1.0,
];

#[inline]
fn c(bits: u32) -> f32 {
    f32::from_bits(bits)
}

/// Linear knob law between three anchor frequencies (knob 0, 5, 10).
fn knob_freq(knob: f32, a: f32, b: f32, cc: f32) -> f32 {
    let k = knob_pos(knob, 3);
    if k < 1.0 {
        fma(k, b, (1.0 - k) * a)
    } else {
        fma(k + -1.0, cc, (2.0 - k) * b)
    }
}

/// Gain position shared by the bells: (column, fraction, mix ratio).
fn bell_pos(g: f32, m: f32) -> (usize, f32, f32) {
    let x0 = g.abs() * m;
    let xc = x0.min(5.0);
    let ratio = if (xc as f64) > 0.01 { x0 / xc } else { 1.0 };
    let xi = if xc > 4.0 { fma(xc, 2.0, -4.0) } else { xc };
    let (i, t) = col(xi, 5);
    (i, t, ratio)
}

fn bell(ctx: &Ctx, f: f32, g: f32, gv: f32, qv: f32, ratio: f32) -> Bq {
    let gs = if g > 0.0 { ratio * gv } else { -(ratio * gv) };
    hqpeq(ctx, f, gs, ratio * ratio.sqrt() * qv)
}

pub(super) fn design(ctx: &Ctx, p: &[f32], m: f32, out: &mut [Bq]) -> Layout {
    let (lf, lg, lq, lt, lmf, lmg, lmq) = (p[0], p[1], p[2], p[3], p[4], p[5], p[6]);
    let (hmf, hmg, hmq, hf, hg, hq, ht) = (p[7], p[8], p[9], p[10], p[11], p[12], p[13]);

    // LF
    let (i, t, ratio) = bell_pos(lg, m);
    out[0] = if lt != 0.0 {
        let f = lerp_col(&N88_LFFTAB, i, t);
        let gv = lerp_col(&N88_LFGTAB, i, t);
        let q1 = lerp_col(&N88_LFQ1TAB, i, t);
        let q2 = lerp_col(&N88_LFQ2TAB, i, t);
        let k = knob_pos(lf, 3);
        let mult = if k < 1.0 {
            fma(k, c(0x3ea6_dc5d), (1.0 - k) * c(0x3d8d_844d))
        } else {
            (k + -1.0) + (2.0 - k) * c(0x3ea6_dc5d)
        };
        let fs = f * mult;
        let gs = ratio * gv;
        let fr = ((ratio as f64).powf(-1.4) * fs as f64) as f32;
        if lg >= 0.0 {
            ls12q2(ctx, fr, gs, 1.0 / q1, 1.0 / q2)
        } else {
            ls12q2(ctx, fr, -gs, 1.0 / q2, 1.0 / q1)
        }
    } else {
        let gv = lerp_col(&N88_LFBGTAB, i, t);
        let set = if lq != 0.0 { 7 } else { 0 };
        let qv = lerp_col(&N88_LFBQTAB[set..], i, t);
        let f = knob_freq(lf, c(0x41fa_6666), c(0x4313_6666), c(0x43e1_a666));
        let gs = if lg > 0.0 { ratio * gv } else { -(ratio * gv) };
        hqpeq(ctx, f, gs, ratio * ratio * qv)
    };

    // LMF: Q rows by the Q knob.
    let (i, t, ratio) = bell_pos(lmg, m);
    let qv = tab_k(&N88_LMQTAB, 7, 0, 3, knob_pos(lmq, 3), i, t);
    let f = knob_freq(lmf, c(0x42ed_3333), c(0x442d_8666), c(0x4506_b333));
    out[1] = bell(ctx, f, lmg, lerp_col(&N88_LMGTAB, i, t), qv, ratio);

    // HMF: Q table rows = 3 Q anchors x 3 frequency anchors.
    let (i, t, ratio) = bell_pos(hmg, m);
    let kf = knob_pos(hmf, 3);
    let qa = [
        tab_k(&N88_HMQTAB, 7, 0, 3, kf, i, t),
        tab_k(&N88_HMQTAB, 7, 3, 3, kf, i, t),
        tab_k(&N88_HMQTAB, 7, 6, 3, kf, i, t),
    ];
    let qv = lerp_anchor(&qa, knob_pos(hmq, 3));
    let f = knob_freq(hmf, c(0x443f_2ccd), c(0x4537_6bae), c(0x4609_6b9a));
    out[2] = bell(ctx, f, hmg, lerp_col(&N88_HMGTAB, i, t), qv, ratio);

    // HF
    out[3] = if ht != 0.0 {
        let mut x = hg.abs() * m;
        if x > 5.0 {
            x = (x as f64).mul_add(0.4, 4.0) as f32;
        } else if x > 4.0 {
            x = fma(x, 2.0, -4.0);
        }
        let (i, t) = col(x, 6);
        let k = knob_pos(hf, 5);
        let tk = |tab: &[f32]| tab_k(tab, 8, 0, 5, k, i, t);
        let (f, g, q1, q2) = (
            tk(&N88_HFFTAB),
            tk(&N88_HFGTAB),
            tk(&N88_HFQ1TAB),
            tk(&N88_HFQ2TAB),
        );
        if hg >= 0.0 {
            hs12q2(ctx, f, g, 1.0 / q1, 1.0 / q2)
        } else {
            hs12q2(ctx, f, -g, 1.0 / q2, 1.0 / q1)
        }
    } else {
        let (i, t, ratio) = bell_pos(hg, m);
        // WING-Edit selects the Q table by `hq` and also blends 20 % towards its second row when HIGH.
        let set = if hq != 0.0 { 21 } else { 0 };
        let kq = if hq != 0.0 { 0.2f32 } else { 0.0 };
        let r0 = lerp_col(&N88_HFBQTAB[set..], i, t);
        let r1 = lerp_col(&N88_HFBQTAB[set + 7..], i, t);
        let qv = fma(r1, kq, (1.0 - kq) * r0);
        let f = knob_freq(hf, c(0x44ba_4b33), c(0x45dc_1400), c(0x46a8_749a));
        bell(ctx, f, hg, lerp_col(&N88_HFBGTAB, i, t), qv, ratio)
    };
    out[4] = UNITY;
    out[5] = UNITY;
    Layout::cascade(6)
}
