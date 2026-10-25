//! `PIA` ("PIA 560 GEQ", API 560 10-band graphic, bus/main/matrix only): port of `ae::eq_parchg_api560`.
//!
//! Slots: `g 31 63 125 250 500 1k 2k 4k 8k 16k` (all -12..+12 dB; `g` = output gain on the first section).
//! Sections: 31 Hz..2 kHz Orfanidis bells, 4/8/16 kHz bilinear high-shelf-type sections (separate tables for
//! 48 and 44.1 kHz). Per band the table holds (f, g, Q) at |gain|/12 = 0, 1/4, .., 1 (anchors, linear in
//! between), so centre frequency and bandwidth move with gain (API proportional Q).

use super::prims::*;
use super::tables::*;
use super::Layout;

pub(super) const DEFAULTS: [f32; 11] = [0.0; 11];

/// Value `v` of a band record (`cols` x `stride` floats) at gain position `u = |g|/12` (mix-scaled).
fn val(tab: &[f32], stride: usize, u: f32, v: usize) -> f32 {
    let at = |c: usize| tab[c * stride + v];
    if u > 1.0 {
        let k = ((u + -1.0) + (u + -1.0)) * 2.0;
        let k = k.max(0.0).min(2.0);
        lerp_anchor(&[at(4), at(5), at(6)], k)
    } else {
        let k = (u * 4.0).min(4.0);
        lerp_anchor(&[at(0), at(1), at(2), at(3), at(4)], k)
    }
}

pub(super) fn design(ctx: &Ctx, p: &[f32], m: f32, out: &mut [Bq]) -> Layout {
    for b in 0..7 {
        let gain = p[1 + b];
        let u = (gain * m / 12.0).abs();
        let rec = &API_TAB1[b * 21..];
        let f = val(rec, 3, u, 0);
        let g = val(rec, 3, u, 1);
        let q = val(rec, 3, u, 2);
        out[b] = hqpeq(ctx, f, if gain < 0.0 { -g } else { g }, q);
    }
    let tab2: &[f32] = if ctx.fs > 46_000.0 {
        &API_TAB2_48
    } else {
        &API_TAB2_44
    };
    for b in 0..3 {
        let gain = p[8 + b];
        let u = (gain * m / 12.0).abs();
        let rec = &tab2[b * 28..];
        let f = val(rec, 4, u, 0);
        let g = val(rec, 4, u, 1);
        let q1 = val(rec, 4, u, 2);
        let q2 = val(rec, 4, u, 3);
        out[7 + b] = if gain >= 0.0 {
            hs12q2(ctx, f, g, 1.0 / q1, 1.0 / q2)
        } else {
            hs12q2(ctx, f, -g, 1.0 / q2, 1.0 / q1)
        };
    }
    let a = db_lin_guard(p[0] * m);
    for c in &mut out[0][..3] {
        *c *= a;
    }
    Layout::cascade(10)
}
