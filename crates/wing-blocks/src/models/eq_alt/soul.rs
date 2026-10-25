//! `SOUL` ("SOUL ANALOGUE", SSL 4000 G-series channel EQ): port of `ae::eq_parchg_ssl_g`.
//!
//! Slots: `lf lg lmf lmf3 lmq lmg hmf hmf3 hmq hmg hf hg` (knobs 0..10, gains -5..+5).
//! Sections: LF 2nd-order shelf + LF 1st-order shelf, LMF and HMF Orfanidis bells, HF 2nd-order shelf, unity.
//! Every band is a 2-D table lookup: gain columns at |g| = 0,1,..,5 (linear in between) and frequency /
//! Q knob anchors at 0, 5, 10 (linear in between). The LMF/HMF tables hold (f, g, Q) per (f-knob, Q-knob,
//! |g|) grid point; `lmf3`/`hmf3` select the "/3" and "x3" table halves.

use super::prims::*;
use super::tables::*;
use super::Layout;

pub(super) const DEFAULTS: [f32; 12] = [2.0, 0.0, 6.0, 1.0, 5.0, 0.0, 3.0, 0.0, 5.0, 0.0, 8.0, 0.0];

/// LMF/HMF parametric band from a 6-block (f-anchor, Q-anchor, |g| column) x (f, g, q) table.
fn mid(ctx: &Ctx, tab: &[f32], x3: bool, f: f32, q: f32, g: f32, m: f32) -> Bq {
    let x0 = g.abs() * m;
    let xc = x0.min(5.0);
    let (i, t) = col(xc, 4);
    let b0 = if x3 { 3 } else { 0 };
    let kq = knob_pos(q, 3);
    let mut v = [[0.0f32; 3]; 3];
    for (bi, vb) in v.iter_mut().enumerate() {
        let blk = &tab[(b0 + bi) * 54..];
        for (k, out) in vb.iter_mut().enumerate() {
            let mut r = [0.0f32; 3];
            for (row, rv) in r.iter_mut().enumerate() {
                let base = row * 18 + k;
                *rv = fma(1.0 - t, blk[base + 3 * i], t * blk[base + 3 * (i + 1)]);
            }
            *out = lerp_anchor(&r, kq);
        }
    }
    let ratio = if (xc as f64) > 0.01 { x0 / xc } else { 1.0 };
    let kf = knob_pos(f, 3);
    let fr = lerp_anchor(&[v[0][0], v[1][0], v[2][0]], kf);
    let gv = lerp_anchor(&[v[0][1], v[1][1], v[2][1]], kf);
    let qv = lerp_anchor(&[v[0][2], v[1][2], v[2][2]], kf);
    let gs = if g > 0.0 { gv } else { -gv };
    hqpeq(ctx, fr, ratio * gs, ratio * (ratio * qv))
}

pub(super) fn design(ctx: &Ctx, p: &[f32], m: f32, out: &mut [Bq]) -> Layout {
    let (lf, lg, lmf, lmf3, lmq, lmg) = (p[0], p[1], p[2], p[3], p[4], p[5]);
    let (hmf, hmf3, hmq, hmg, hf, hg) = (p[6], p[7], p[8], p[9], p[10], p[11]);

    // LF: 2nd-order shelf + 1st-order shelf, columns |g| = 0..5 (+ extrapolation column for mix > 100 %).
    let mut x = lg.abs() * m;
    if x > 5.0 {
        x = (x as f64).mul_add(0.4, 3.0) as f32;
    }
    let (i, t) = col(x, 5);
    let k = knob_pos(lf, 3);
    let tk = |tab: &[f32]| tab_k(tab, 7, 0, 3, k, i, t);
    let (f1, g1, q1, q2) = (
        tk(&SSL_LFFTAB1),
        tk(&SSL_LFGTAB1),
        tk(&SSL_LFQ1TAB1),
        tk(&SSL_LFQ2TAB1),
    );
    out[0] = if lg >= 0.0 {
        ls12q2(ctx, f1, g1, 1.0 / q1, 1.0 / q2)
    } else {
        ls12q2(ctx, f1, -g1, 1.0 / q2, 1.0 / q1)
    };
    let (f2, g2) = (tk(&SSL_LFFTAB2), tk(&SSL_LFGTAB2));
    out[1] = ls6_ssl(ctx, f2, if lg > 0.0 { g2 } else { -g2 });

    out[2] = mid(ctx, &SSL_LMFTAB, lmf3 != 0.0, lmf, lmq, lmg, m);
    out[3] = mid(ctx, &SSL_HMFTAB, hmf3 != 0.0, hmf, hmq, hmg, m);

    // HF: 2nd-order shelf, columns |g| = 0..4, 4..5 spans two columns, beyond (mix > 100 %) one more.
    let mut x = hg.abs() * m;
    if x > 5.0 {
        x = (x as f64).mul_add(0.4, 4.0) as f32;
    } else if x > 4.0 {
        x = fma(x, 2.0, -4.0);
    }
    let (i, t) = col(x, 6);
    let k = knob_pos(hf, 3);
    let tk = |tab: &[f32]| tab_k(tab, 8, 0, 3, k, i, t);
    let (f, g, q1, q2) = (
        tk(&SSL_HFFTAB1),
        tk(&SSL_HFGTAB1),
        tk(&SSL_HFQ1TAB1),
        tk(&SSL_HFQ2TAB1),
    );
    out[4] = if hg >= 0.0 {
        hs12q2(ctx, f, g, 1.0 / q1, 1.0 / q2)
    } else {
        hs12q2(ctx, f, -g, 1.0 / q2, 1.0 / q1)
    };
    out[5] = UNITY;
    Layout::cascade(6)
}
