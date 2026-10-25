//! `PULSAR` (Pultec EQP-1A + MEQ-5): port of `ae::eq_parchg_pultec_duo`.
//!
//! Slots: `eq1 1lb 1latt 1lf 1hw 1hb 1hf 1hatt 1hattf eq5 5lb 5lf 5md 5mf 5hb 5hf`.
//! WING-Edit models the passive networks: each band's analog transfer function is built from component
//! values (`*Tab`: capacitor/inductor pairs per frequency selector, pot resistances from the knob through
//! fitted pot tapers) and then either mapped to an equivalent Orfanidis bell (`bilin_z_trans_2_par`: bilinear
//! at 4x the sample rate, read back as centre frequency / gain / Q) or bilinear-transformed directly.
//! Sections: MEQ-5 low-mid boost, mid dip, high-mid boost (bells), EQP-1A low network (boost + attenuation
//! interacting in one biquad), HF attenuation 1st-order shelf, HF boost bell.
//!
//! WING-Edit quirk kept for fidelity: the mid-dip band's mix ratio divides the *low-boost* amount (a stale
//! register), so with `5lb` above `5md` the dip's gain/Q are scaled by `5lb / 5md`.

use super::prims::*;
use super::tables::*;
use super::Layout;

pub(super) const DEFAULTS: [f32; 16] = [
    1.0, 0.0, 0.0, 1.0, 5.0, 0.0, 4.0, 0.0, 2.0, 1.0, 0.0, 1.0, 0.0, 4.0, 0.0, 2.0,
];

const K_NUM: f32 = 12_000_000_000.0; // 0x5032d05e
#[inline]
fn k_den() -> f32 {
    f32::from_bits(0x523d_a54b)
}
#[inline]
fn d(bits: u64) -> f64 {
    f64::from_bits(bits)
}

/// `bilin_z_trans_2_par(k, fs4, num, den)`: bilinear transform (c = 2*fs4) of the analog biquad
/// `k*(n0 + n1 s + n2 s^2)/(d0 + d1 s + d2 s^2)` and its read-back as (f, gain dB, Q) of a peaking EQ.
fn bilin_par(k: f32, fs4: f32, num: [f32; 3], den: [f32; 3]) -> (f32, f32, f32) {
    let (n0, n1, n2) = (num[0] as f64, num[1] as f64, num[2] as f64);
    let (e0, e1, e2) = (den[0] as f64, den[1] as f64, den[2] as f64);
    let c = (fs4 + fs4) as f64;
    let c2 = c * c;
    let inv = 1.0 / e2.mul_add(c2, e1.mul_add(c, e0));
    let k = k as f64;
    let b2 = n2.mul_add(c2, (-n1).mul_add(c, n0)) * k * inv;
    let b0 = n2.mul_add(c2, n1.mul_add(c, n0)) * k * inv;
    let b1 = n0.mul_add(2.0, c2 * (n2 * -2.0)) * k * inv;
    let a2 = inv * (-e2).mul_add(c2, -(-e1).mul_add(c, e0));
    let a1 = inv * (-e0).mul_add(2.0, -(c2 * (e2 * -2.0)));
    let p = a2.mul_add(2.0, 2.0) / ((a1 - a2) + 1.0);
    let r = (((1.0 - a2).mul_add(p, -1.0) - a2) / (a2 + 1.0)).sqrt();
    let fs4d = fs4 as f64;
    let f = (r.atan() * fs4d / d(0x4009_21fb_5452_4550)) as f32;
    let w = (f as f64) * d(0x4009_21fb_5452_4550) / fs4d;
    let sum = b2 + (b0 + b1);
    let nn0 = sum * sum;
    let nn1 = b1.mul_add(b2, b0.mul_add(b1, b2 * (b0 * 4.0))) * -4.0;
    let nn2 = b2 * (b0 * 16.0);
    let one = (1.0 - a1) - a2;
    let dd0 = one * one;
    let dd1 = (-a1).mul_add(a2, a2.mul_add(4.0, a1)) * 4.0;
    let dd2 = a2 * -16.0;
    let s = w.sin();
    let s2 = s * s;
    let numv = (nn2 * s2).mul_add(s2, nn1.mul_add(s2, nn0));
    let denv = (dd2 * s2).mul_add(s2, dd1.mul_add(s2, dd0));
    let mag = ((numv / denv).sqrt() + -1.0) + 1.0;
    let g = if mag > d(0x3ea0_c6f7_a0b5_ed8d) {
        (mag.log10() * 20.0) as f32
    } else {
        -127.0
    };
    let q = (w / p / if b0 > 1.0 { 1.0 } else { mag }) as f32;
    (f, g, q)
}

/// Mix ratio of a 0..10 control: `(x, ratio)` with `x = min(v, 10)`, `ratio = v/x` (1 when x <= 0.01).
fn knob10(v: f32) -> (f32, f32) {
    let x = v.min(10.0);
    let r = if (x as f64) > 0.01 { v / x } else { 1.0 };
    (x, r)
}

pub(super) fn design(ctx: &Ctx, p: &[f32], m: f32, out: &mut [Bq]) -> Layout {
    let eq1 = p[0] != 0.0;
    let (lb1, latt1) = (p[1], p[2]);
    let lf1 = (p[3] as i32).clamp(0, 3) as usize;
    let (hw1, hb1) = (p[4], p[5]);
    let hf1 = (p[6] as i32).clamp(0, 6) as usize;
    let hatt1 = p[7];
    let hattf1 = (p[8] as i32).clamp(0, 2) as usize;
    let eq5 = p[9] != 0.0;
    let lb5 = p[10];
    let lf5 = (p[11] as i32).clamp(0, 4) as usize;
    let md5 = p[12];
    let mf5 = (p[13] as i32).clamp(0, 10) as usize;
    let hb5 = p[14];
    let hf5 = (p[15] as i32).clamp(0, 4) as usize;
    let fs = ctx.fs;
    let fs4 = fs * 4.0;
    let kb = f32::from_bits(0x4187_b439);
    let kd = k_den();

    // MEQ-5 low-mid boost.
    let (ca, rb) = (PUL_LOWBOOSTTAB[2 * lf5], PUL_LOWBOOSTTAB[2 * lf5 + 1]);
    let v5lb = lb5 * m;
    let (x, r) = knob10(v5lb);
    let g0 = r.max(1.0);
    let rp =
        ((x as f64).mul_add(d(0x3fb9_1687_2b02_0c4a), d(0x3f60_624d_d2f1_a9fc)) * 2500.0) as f32;
    let s4 = 2500.0 - rp;
    let carp = ca * rp;
    let carb = ca * rb;
    let num = [K_NUM, carp * K_NUM, carb * K_NUM];
    let s2 = fma(s4, 1510.0, 510_000.0);
    let s3 = (s4 + 1000.0) * 2500.0 * 14800.0;
    let den = [kd, fma(s2, 17300.0, s3) * carp, carb * kd];
    let (f, g, q) = bilin_par(kb, fs4, num, den);
    out[0] = hqpeq(ctx, f, if eq5 { g0 * g } else { 0.0 }, g0 * q);

    // MEQ-5 mid dip (ratio uses the low-boost amount, see module doc).
    let (cm, rm) = (PUL_MIDCUTTAB[2 * mf5], PUL_MIDCUTTAB[2 * mf5 + 1]);
    let x = (md5 * m).min(10.0);
    let r = if (x as f64) > 0.01 { v5lb / x } else { 1.0 };
    let g1 = r.max(1.0);
    let xs = ((x as f64) * d(0x3fee_147a_e147_ae14)) as f32;
    let x2 = (xs * xs) as f64;
    let xd = xs as f64;
    let (poly, c4) = if xs > 5.0 {
        let t = xd.mul_add(d(0x3fe7_870f_433e_eefe), d(0xbfe7_c7e4_2a61_7d73));
        let t = x2.mul_add(d(0xbfbe_4444_5295_4e30), t);
        (
            (x2 * d(0x3f81_6605_4cd8_e730)).mul_add(xd, t),
            d(0xbf2e_098e_b4ba_3799),
        )
    } else {
        let t = xd.mul_add(d(0x3fbe_f2d3_71ac_95a0), x2 * d(0x3fbd_c65b_ef6a_05ad));
        (
            (x2 * d(0xbfa2_ba9d_ed88_a67a)).mul_add(xd, t),
            d(0x3f69_a8eb_8ca4_9555),
        )
    };
    let rp = (((x2 * c4).mul_add(x2, poly)) as f32) * 2500.0;
    let s3 = 2500.0 - rp;
    let s4 = fma(s3, rp, 675_000.0);
    let cmrm = cm * rm;
    let num = [K_NUM, cm * 1000.0 * 4800.0 * s4, cmrm * K_NUM];
    let t = (rp + 270.0) * fma(s3, 3500.0, 4_285_000.0);
    let t = fma(t, 14800.0, s4 * 4_285_000.0);
    let t = fma(rp * 270.0 * 3500.0, 14800.0, t);
    let den = [kd, cm * t, cmrm * kd];
    let (f, g, q) = bilin_par(kb, fs4, num, den);
    out[1] = hqpeq(ctx, f, if eq5 { g1 * g } else { 0.0 }, g1 * q);

    // MEQ-5 high-mid boost.
    let (ch, rh) = (PUL_HIBOOSTTAB[2 * hf5], PUL_HIBOOSTTAB[2 * hf5 + 1]);
    let (x, r) = knob10(hb5 * m);
    let rp = ((x as f64).mul_add(d(0x3fb9_5810_624d_d2f2), 0.01) * 10000.0) as f32;
    let s3 = 10000.0 - rp;
    let chrp = ch * rp;
    let chrh = ch * rh;
    let num = [K_NUM, chrp * K_NUM, chrh * K_NUM];
    let s1 = (s3 + 2500.0) + 4800.0;
    let t = (s3 + 4800.0) * 8_750_000.0;
    let den = [kd, fma(s1, 4_285_000.0, t) * chrp, chrh * kd];
    let (f, g, q) = bilin_par(kb, fs4, num, den);
    let g2 = r.max(1.0);
    out[2] = hqpeq(ctx, f, if eq5 { g2 * g } else { 0.0 }, g2 * q);

    // EQP-1A low network: boost and attenuation pots in one passive biquad (bilinear at 2 fs).
    let (la, lb) = (PUL_LOSHVTAB[2 * lf1], PUL_LOSHVTAB[2 * lf1 + 1]);
    let vb = lb1 * m;
    let xb = if eq1 { vb } else { 0.0 }.min(10.0);
    let rb_ratio = if (xb as f64) > 0.01 { vb / xb } else { 1.0 };
    let va = latt1 * m;
    let xa_in = if eq1 { va } else { 0.0 };
    let e = d(0x4005_bf0a_8b04_919b);
    let t = e.powf((xb as f64) * d(0xbfef_915b_b7e1_ae30));
    let t = d(0x3ff0_2e38_fdcd_75bf) / t.mul_add(d(0x4089_f89f_b82c_2bd8), 1.0)
        + d(0xbf53_e814_50ef_dc9c);
    let r_lb = ((t as f32) * 10000.0) as f64;
    let rbst = (3f64.powf(((rb_ratio.max(1.0) + -1.0) * 2.0) as f64) * r_lb) as f32;
    let xa = xa_in.min(10.0);
    let ra_ratio = if (xa as f64) > 0.01 { va / xa } else { 1.0 };
    let t = e.powf((xa as f64) * d(0xbfec_c9d0_4de0_b38a));
    let t = d(0x3ff0_45ce_5893_1419) / t.mul_add(d(0x4079_c857_b203_1ceb), 1.0)
        + d(0xbf63_fd0d_0678_c005);
    let r_la = ((t as f32) * 100_000.0) as f64;
    let ratt = (5f64.powf(((ra_ratio.max(1.0) + -1.0) * 2.0) as f64) * r_la) as f32;
    {
        let (ra, rb) = (ratt, rbst);
        let s2a = ra + 12000.0;
        let s4 = fma(rb, s2a, 1.0e7);
        let s5 = la * ra;
        let s1 = fma(s5, fma(rb, 12000.0, 1.0e7), lb * 1000.0 * rb * 10000.0);
        let s6 = la * lb;
        let s7 = s6 * 1000.0 * rb * ra * 10000.0;
        let s16 = rb + 10000.0;
        let s2 = fma(s16, s2a, ra * 1000.0) + 1.0e6 + 1.0e7;
        let s16b = fma(rb + 11000.0, 11000.0, s16 * 1000.0);
        let s3 = fma(s5, s16b, fma(ra + 11000.0, 11000.0, 1.0e7) * (lb * rb));
        let s0 = s6 * rb * ra * 131_000_000.0;
        let c = fs + fs;
        let c2 = c * c;
        let s17 = c2 * (s0 * -2.0);
        let n = 1.0 / fma(s0, c2, fma(s3, c, s2));
        let a2m = n * (-s0).mul_add(c2, -(-s3).mul_add(c, s2));
        let b1 = n * (fma(s4, 2.0, c2 * (s7 * -2.0)) * 13.11);
        let b0 = n * (fma(s7, c2, fma(s1, c, s4)) * 13.11);
        let b2 = n * (fma(s7, c2, (-s1).mul_add(c, s4)) * 13.11);
        let a1m = n * (-s2).mul_add(2.0, -s17);
        out[3] = [b0, b1, b2, -a1m, -a2m];
    }

    // EQP-1A HF attenuation: 1st-order high shelf from a (f, g) table over the knob (16 points).
    let hm = hatt1 * m;
    let i = (hm as i32).min(14).max(0) as usize;
    let tab: &[f32] = if fs > 46_000.0 {
        &PUL_HISHVTAB48
    } else {
        &PUL_HISHVTAB44
    };
    let xr = m.min(1.0) * (hatt1 * 100.0);
    let r_sh = (1.0 / (1.0 / (xr as f64).max(d(0x3f1a_36e2_eb1c_432d)) + d(0x3f8b_4e81_c000_0000))
        + (1000.0 - xr) as f64) as f32;
    let t = hatt1.mul_add(m, -(i as f32));
    let rec = &tab[hattf1 * 32 + i * 2..];
    let ff = fma(1.0 - t, rec[0], t * rec[2]);
    let gg = if eq1 {
        fma(1.0 - t, rec[1], t * rec[3])
    } else {
        0.0
    };
    let k = tanf(ctx.pi_fs * ff);
    let (a, num, den) = if gg > -98.0 {
        let a = db_lin(gg);
        if a >= 1.0 {
            (a, k + -1.0, k + 1.0)
        } else {
            (a, fma(a, k, -1.0), fma(a, k, 1.0))
        }
    } else {
        (0.0, -1.0, 1.0)
    };
    let c = num / den;
    let am1 = a + -1.0;
    let b0 = (((1.0 - c) * am1) as f64).mul_add(0.5, 1.0) as f32;
    let b1 = (((c + -1.0) * am1) as f64).mul_add(0.5, c as f64) as f32;
    out[4] = [b0, b1, 0.0, c, 0.0];

    // EQP-1A HF boost bell (bandwidth pot `1hw`), loaded by the attenuator's shunt resistance.
    let s2 = r_sh + 10000.0;
    let s4 = s2 + 101_000.0;
    let s5 = r_sh * 10000.0;
    let kk = 1.0 / (s5 / (s4 * ((r_sh * 101_000.0) / s4 + 10000.0)));
    let (x, r) = knob10(hb1 * m);
    let g5 = r.max(1.0);
    let rb = x * 1000.0;
    let s4b = 10000.0 - rb;
    let rw = ((((hw1 as f64).max(0.02)) * 0.1) * 2200.0) as f32;
    let (cp, lp) = (PUL_PARATAB[2 * hf1], PUL_PARATAB[2 * hf1 + 1]);
    let s17 = rb + rw;
    let num = [s5, s5 * (s17 * cp), s5 * cp * lp];
    let s19 = r_sh + 111_000.0;
    let d0 = fma(s19, 10000.0, r_sh * 111_000.0);
    let d1 = fma(fma(s4b, rb, rw * 10000.0), s19, s17 * r_sh * 111_000.0) * cp;
    let d2 = fma(s2, 111_000.0, s5) * (cp * lp);
    let (f, g, q) = bilin_par(kk, fs4, num, [d0, d1, d2]);
    let gb = g5.sqrt() * g;
    out[5] = hqpeq(ctx, f, if eq1 { gb } else { 0.0 }, (g5 * q).min(50.0));
    Layout::cascade(6)
}
