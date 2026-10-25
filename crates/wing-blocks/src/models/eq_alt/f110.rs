//! `F110` ("FORTISSIMO 110", Focusrite ISA 110): port of `ae::eq_parchg_isa110`.
//!
//! Slots: `peq lmf lmg lmq lmf3 hmf hmg hmq hmf3 shv lf lg hf hg g`.
//! Sections: LF low shelf (carries the `g` output gain) + LF high-shelf correction, HF shelf, LMF and HMF
//! bells, unity. Shelves are table-driven per frequency selector (LF: 7 gain columns, HF: 9 columns on a
//! square-root gain taper). The mid bands evaluate a small resistor network: the gain knob splits a 22 kΩ pot,
//! the Q knob sets a 1.2..6.2 kΩ resistor, and the network's two node gains give the bell's gain (dB) and Q.
//! Frequencies are piecewise linear in the knob between five anchors (`x3` switches the anchor set).
//! `peq` off zeroes the mid gains, `shv` off zeroes the shelf gains.

use super::prims::*;
use super::tables::*;
use super::Layout;

pub(super) const DEFAULTS: [f32; 15] = [
    1.0, 7.0, 0.0, 5.0, 0.0, 3.0, 0.0, 5.0, 0.0, 1.0, 1.0, 0.0, 3.0, 0.0, 0.0,
];

#[inline]
fn c(bits: u32) -> f32 {
    f32::from_bits(bits)
}

const LMF_X1: [u32; 5] = [
    0x4214_0000,
    0x42ac_cccd,
    0x42ef_0000,
    0x432e_8000,
    0x43cd_199a,
];
const LMF_X3: [u32; 5] = [
    0x42eb_999a,
    0x438b_0ccd,
    0x43be_f333,
    0x440b_e000,
    0x44a3_8000,
];
const HMF_X1: [u32; 5] = [
    0x4408_e666,
    0x44a0_8ccd,
    0x44dd_5000,
    0x4521_6333,
    0x45bd_bf33,
];
const HMF_X3: [u32; 5] = [
    0x44d5_1000,
    0x457b_299a,
    0x45ac_a8cd,
    0x45fb_c666,
    0x4694_7d66,
];

fn knob5(knob: f32, anchors: &[u32; 5]) -> f32 {
    lerp_anchor(&anchors.map(c), knob_pos(knob, 5))
}

/// ISA-style mid band: gain pot `g` (-5..5) and Q pot `q` (0..10) through the band's resistor network.
fn mid(ctx: &Ctx, f: f32, on: bool, g: f32, q: f32, m: f32) -> Bq {
    let u = if on { g / 5.0 * m } else { 0.0 };
    let umax = u.abs().max(1.0);
    let w = ((u as f64).mul_add(0.5, 0.5) as f32).max(0.0).min(1.0);
    let ra = (1.0 - w) * 22000.0;
    let rb = 22000.0 - ra;
    let rq = fma(q / -10.0 + 1.0, 5000.0, 1200.0) as f64;
    let r = (ra + rb) as f64;
    let (ra, rb) = (ra as f64, rb as f64);
    let k = |b: u64| f64::from_bits(b);
    let mut d = r.mul_add(k(0xc1bf_957f_0ccc_cccd), r * k(0xc0ef_8d9b_a5e3_53f8) * rq);
    d = (ra * k(0xc0e7_857d_566c_f41f)).mul_add(rb, d);
    d = (ra * k(0xc017_7f9d_6f11_2fd3) * rb).mul_add(rq, d);
    let d = (d as f32) as f64;
    let e5 = ra * k(0x4021_150d_ae3e_6c4c) * rb;
    let mut n1 = rb.mul_add(k(0xc1a7_bc5f_7ae1_47ae), rb * k(0xc122_640a_f837_b4a2) * rq);
    n1 = (ra * k(0xc0f6_f005_1eb8_51ec)).mul_add(rq, n1);
    n1 = (-e5).mul_add(rq, n1);
    let n1 = (n1 / d) as f32;
    let mut n2 = ra.mul_add(k(0x41a7_bc5f_7ae1_47ae), ra * k(0x4122_640a_f837_b4a2) * rq);
    n2 = (rb * k(0x40f6_f005_1eb8_51ec)).mul_add(rq, n2);
    n2 = e5.mul_add(rq, n2);
    let n2 = (n2 / d) as f32;
    let ratio = -n1 / n2;
    let db = if (ratio as f64) > f64::from_bits(0x3ea0_c6f7_a0b5_ed8d) {
        ratio.log10() * 20.0
    } else {
        -127.0
    };
    let gain = umax.sqrt() * db;
    let qv = if n1 > -n2 { -umax / n2 } else { umax / n1 };
    hqpeq(ctx, f, gain, qv)
}

pub(super) fn design(ctx: &Ctx, p: &[f32], m: f32, out: &mut [Bq]) -> Layout {
    let (peq, lmf, lmg, lmq, lmf3) = (p[0] != 0.0, p[1], p[2], p[3], p[4] != 0.0);
    let (hmf, hmg, hmq, hmf3, shv) = (p[5], p[6], p[7], p[8] != 0.0, p[9] != 0.0);
    let (lf, lg, hf, hg, g) = (p[10], p[11], p[12], p[13], p[14]);
    let lf = (lf as i32).clamp(0, 5) as usize;
    let hf = (hf as i32).clamp(0, 5) as usize;

    // LF: (f, g, q1, q2) low shelf + (f2, g2, q1b, q2b) high-shelf section per gain column; cut-signed gains.
    let mut x = if shv { lg.abs() * m } else { 0.0 };
    if x > 5.0 {
        x = (x as f64).mul_add(0.2, 4.0) as f32;
    }
    let i = (x as i32).max(0) as usize;
    let t = x - (x as i32) as f32;
    let base = lf * 56;
    let v = |k: usize| {
        fma(
            1.0 - t,
            ISA_LF[base + i * 8 + k],
            t * ISA_LF[base + (i + 1) * 8 + k],
        )
    };
    let (f, gv, q1, q2) = (v(0), v(1), v(2), v(3));
    out[0] = if lg < 0.0 {
        ls12q2(ctx, f, gv, 1.0 / q1, 1.0 / q2)
    } else {
        ls12q2(ctx, f, -gv, 1.0 / q2, 1.0 / q1)
    };
    let a = db_lin_guard(g * m);
    out[0][1] *= a;
    out[0][2] *= a;
    out[0][0] *= a;
    let (f, gv, q1, q2) = (v(4), v(5), v(6), v(7));
    out[1] = if lg < 0.0 {
        hs12q2(ctx, f, gv, 1.0 / q1, 1.0 / q2)
    } else {
        hs12q2(ctx, f, -gv, 1.0 / q2, 1.0 / q1)
    };

    // HF shelf: square-root gain taper onto 8 columns (9th for mix > 100 %).
    let x = {
        let u = if shv { hg.abs() / 5.0 * m } else { 0.0 };
        if u > 1.0 {
            fma(u + -1.0, 2.0, 7.0)
        } else {
            let h = ((u as f64).mul_add(0.5, 0.5) as f32) as f64;
            let s = h
                .mul_add(
                    f64::from_bits(0xc002_f6ff_3d8d_543c),
                    f64::from_bits(0x4003_0893_d6b7_0667),
                )
                .sqrt();
            ((f64::from_bits(0x3ff1_7b7f_a01e_430c) - s) as f32) * 7.0
        }
    };
    let (i, t) = col(x, 7);
    let r = hf * 9;
    let hv = |tab: &[f32]| lerp_col(&tab[r..], i, t);
    let (f, gv, q1, q2) = (hv(&ISA_HF_F), hv(&ISA_HF_G), hv(&ISA_HF_Q1), hv(&ISA_HF_Q2));
    out[2] = if hg > 0.0 {
        hs12q2(ctx, f, gv, 1.0 / q1, 1.0 / q2)
    } else {
        hs12q2(ctx, f, -gv, 1.0 / q2, 1.0 / q1)
    };

    let fl = knob5(lmf, if lmf3 { &LMF_X3 } else { &LMF_X1 });
    out[3] = mid(ctx, fl, peq, lmg, lmq, m);
    let fh = knob5(hmf, if hmf3 { &HMF_X3 } else { &HMF_X1 });
    out[4] = mid(ctx, fh, peq, hmg, hmq, m);
    out[5] = UNITY;
    Layout::cascade(6)
}
