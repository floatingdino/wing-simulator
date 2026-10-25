//! `MACH4` ("MACH EQ4", Maag EQ4, channel only): port of `ae::eq_parchg_maag4`.
//!
//! Slots: `sub 40 160 650 2k5` (-5..+5), `air` (0..10), `airm` (OFF/2k5/5k/10k/20k/40k), `again` (auto gain).
//! A **parallel** bank of six fixed band shapes (constant poles per sample-rate set); each band's output level
//! is `10^((L0 + s * L(k)) / 20)` with a per-band 5-anchor dB law `L` over the knob, an extra slope `s` beyond
//! the knob range (mix > 100 %), and the dry share `(1 - min(mix,1))/6` added to every band so the bank sums
//! to unity when flat. `again` normalises the six band levels to a constant total power. Air: one of five
//! fixed shelf/bell shapes chosen by `airm` (OFF uses the first shape at zero level).

use super::prims::*;
use super::Layout;

pub(super) const DEFAULTS: [f32; 8] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 5.0, 1.0];

/// Per band: L0 (base level dB), then the dB law at knob anchors -5, -2.5, 0, +2.5, +5 (air: 0, 2.5, .., 10).
const LEVELS: [[u32; 6]; 6] = [
    [
        0xc1ad47ae, 0xc181999a, 0xc154cccd, 0x0, 0x41680000, 0x41bd999a,
    ],
    [
        0xc1ac7ae1, 0xc1826666, 0xc1433333, 0x0, 0x41500000, 0x41accccd,
    ],
    [
        0xc1b028f6, 0xc17ccccd, 0xc1433333, 0x0, 0x41680000, 0x41ba6666,
    ],
    [
        0xc1b428f6, 0xc174cccd, 0xc139999a, 0x0, 0x415b3333, 0x41b40000,
    ],
    [
        0xc1b2b852, 0xc1700000, 0xc1380000, 0x0, 0x41666666, 0x41bccccd,
    ],
    [
        0xc21c6666, 0x0, 0x40533333, 0x41780000, 0x41ed999a, 0x4218cccd,
    ],
];

/// Fixed bands 0..4 per rate set: (P, Q, R, S, T, A2, A1) with
/// `b2 = P m g + Q d`, `b0 = R m g + d`, `b1 = S m g + T d`, `a1 = -A1`, `a2 = -A2` (d = dry share).
const BANDS_48: [[u32; 7]; 5] = [
    [
        0x3f7d_b6a9,
        0x3f7f_58b6,
        0x3f80_d107,
        0xbfff_ac4f,
        0xbfff_ac4f,
        0xbf7f_58b6,
        0x3fff_ac4f,
    ],
    [
        0x3f76_cfdb,
        0x3f7d_62f0,
        0x3f83_498c,
        0xbffe_b096,
        0xbffe_b096,
        0xbf7d_62f0,
        0x3ffe_b096,
    ],
    [
        0x3f5b_b849,
        0x3f75_9f7b,
        0x3f8c_f399,
        0xbffa_c20d,
        0xbffa_c20d,
        0xbf75_9f7b,
        0x3ffa_c20d,
    ],
    [
        0x3f00_b791,
        0x3f58_4bce,
        0x3fb7_2767,
        0xbff6_b39a,
        0xbfeb_5654,
        0xbf58_4bce,
        0x3feb_5654,
    ],
    [
        0x0000_0000,
        0x8000_0000,
        0x4104_5c96,
        0xc101_f713,
        0xbf59_a7d1,
        0x0000_0000,
        0x3f59_a7d1,
    ],
];
const BANDS_44: [[u32; 7]; 5] = [
    [
        0x3f7d_82f7,
        0x3f7f_49ef,
        0x3f80_e37a,
        0xbfff_a4e8,
        0xbfff_a4e8,
        0xbf7f_49ef,
        0x3fff_a4e8,
    ],
    [
        0x3f76_0101,
        0x3f7d_2819,
        0x3f83_938a,
        0xbffe_9308,
        0xbffe_9308,
        0xbf7d_2819,
        0x3ffe_9308,
    ],
    [
        0x3f58_9522,
        0x3f74_b9c1,
        0x3f8e_1251,
        0xbffa_4cb6,
        0xbffa_4cb6,
        0xbf74_b9c1,
        0x3ffa_4cb6,
    ],
    [
        0x3eee_a392,
        0x3f55_0c73,
        0x3fbc_2bd8,
        0xbff6_e0a0,
        0xbfe9_9219,
        0xbf55_0c73,
        0x3fe9_9219,
    ],
    [
        0x0000_0000,
        0x8000_0000,
        0x4103_97a0,
        0xc101_0002,
        0xbf56_8615,
        0x0000_0000,
        0x3f56_8615,
    ],
];

/// Air shapes per `airm` (b2, b0, b1 coefficients of the level part, A2, A1): `Tab3_48` / `Tab3_44`.
const AIR_48: [[u32; 5]; 6] = [
    [
        0x4110_1690,
        0x411d_3da9,
        0xc196_8cec,
        0xbf38_c190,
        0x3fda_8da0,
    ],
    [
        0x4110_1690,
        0x411d_3da9,
        0xc196_8cec,
        0xbf38_c190,
        0x3fda_8da0,
    ],
    [
        0x40eb_dc7b,
        0x410c_7368,
        0xc180_cce2,
        0xbf04_1fc6,
        0x3fbb_d0e1,
    ],
    [
        0x4073_8e7e,
        0x40e6_5c33,
        0xc12b_ae49,
        0xbe46_67f2,
        0x3f6b_62c6,
    ],
    [
        0x3f91_83e8,
        0x40a0_82eb,
        0xc0af_a7e7,
        0xbb8c_d8ea,
        0x3eae_7399,
    ],
    [
        0xbe48_f290,
        0x4042_e15d,
        0xbfd6_8041,
        0x3c63_9c47,
        0xbe3f_5ad3,
    ],
];
const AIR_44: [[u32; 5]; 6] = [
    [
        0x410d_ac41,
        0x411b_ceff,
        0xc194_9b77,
        0xbf33_6e79,
        0x3fd7_949f,
    ],
    [
        0x410d_ac41,
        0x411b_ceff,
        0xc194_9b77,
        0xbf33_6e79,
        0x3fd7_949f,
    ],
    [
        0x40e4_7655,
        0x410a_1149,
        0xc17b_6758,
        0xbef9_9e37,
        0x3fb7_3e9a,
    ],
    [
        0x405f_f1cc,
        0x40de_f9ea,
        0xc122_9aaa,
        0xbe2a_213c,
        0x3f5c_9c4e,
    ],
    [
        0x3f78_ef76,
        0x4098_ca6d,
        0xc0a1_8179,
        0x3bb3_ea08,
        0x3e96_c223,
    ],
    [
        0xbea4_1373,
        0x4039_d61b,
        0xbfa6_e182,
        0x3b85_670e,
        0xbe91_2cf4,
    ],
];

/// Level multiplier `s` beyond the knob range (|u| > 1 only with mix > 100 %).
fn slope(b: usize, u: f32) -> f32 {
    let a = u.abs();
    if a <= 1.0 {
        return 1.0;
    }
    match b {
        0 => {
            if u > 0.0 {
                ((u + -1.0) as f64).mul_add(0.4, 1.0) as f32
            } else {
                u * u
            }
        }
        4 => {
            if u <= 0.0 {
                u * u
            } else {
                ((u + -1.0) as f64).mul_add(0.5, 1.0) as f32
            }
        }
        5 => ((u + -1.0) as f64).mul_add(0.25, 1.0) as f32,
        _ => {
            if u <= 0.0 {
                u * u
            } else {
                u.sqrt()
            }
        }
    }
}

pub(super) fn design(ctx: &Ctx, p: &[f32], m: f32, out: &mut [Bq]) -> Layout {
    let airm = (p[6] as i32).clamp(0, 5) as usize;
    let again = p[7] != 0.0;
    let mmax = m.max(1.0) as f64;
    let m1 = m.min(1.0);
    let mut lv = [0.0f32; 6];
    for (b, l) in lv.iter_mut().enumerate() {
        let scale = if b == 5 { 0.1 } else { 0.2 };
        let u = (scale * (p[b] as f64) * mmax) as f32;
        let s = slope(b, u);
        let uc = u.max(-1.0).min(1.0);
        let pos = if b == 5 {
            uc
        } else {
            (uc as f64).mul_add(0.5, 0.5) as f32
        };
        let k = (pos * 4.0).max(0.0).min(4.0);
        let t = LEVELS[b].map(f32::from_bits);
        let db = fma(s, lerp_anchor(&t[1..], k), t[0]);
        *l = if db > -98.0 { db_lin(db) } else { 0.0 };
    }
    if airm == 0 {
        lv[5] = 0.0;
    }
    let dry = (1.0 - m1) / 6.0;
    if again {
        let sum = lv.iter().fold(0.0f32, |a, &x| a + x);
        let c = f32::from_bits(if airm == 0 { 0x3ecb_1118 } else { 0x3ed0_bee8 });
        let n = (c / sum).sqrt();
        for l in &mut lv {
            *l *= n;
        }
    }
    let fs48 = ctx.fs > 46_000.0;
    let bands = if fs48 { &BANDS_48 } else { &BANDS_44 };
    for (b, c) in bands.iter().enumerate() {
        let c = c.map(f32::from_bits);
        let g = lv[b];
        let b2 = fma(m1 * c[0], g, dry * c[1]);
        let b0 = fma(m1 * c[2], g, dry);
        let b1 = fma(m1 * c[3], g, dry * c[4]);
        out[b] = [b0, b1, b2, -c[6], -c[5]];
    }
    let e = if fs48 { AIR_48[airm] } else { AIR_44[airm] }.map(f32::from_bits);
    let g = lv[5];
    let nd = -dry;
    let b2 = fma(e[0] * m1, g, e[3] * nd);
    let b0 = fma(e[1] * m1, g, dry);
    let b1 = fma(m1 * e[2], g, e[4] * nd);
    out[5] = [b0, b1, b2, -e[4], -e[3]];
    Layout::parallel(6)
}
