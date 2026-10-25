//! WING-Edit's `Cbiquad` design primitives used by the alternate EQ models, and the table-interpolation
//! helpers shared by the `eq_parchg_*` ports.
//!
//! Every function mirrors the arm64 code of WING-Edit 3.3.4 operation by operation (float32 arithmetic with
//! fused multiply-adds where the original uses `fmadd`, float64 where it converts to double, the same constants
//! by bit pattern), so the coefficients match WING-Edit's to float32 rounding. Output sections are
//! `[b0, b1, b2, a1, a2]` with `H(z) = (b0 + b1 z^-1 + b2 z^-2) / (1 + a1 z^-1 + a2 z^-2)`.

/// One second-order section `[b0, b1, b2, a1, a2]`.
pub type Bq = [f32; 5];

/// Unity section.
pub const UNITY: Bq = [1.0, 0.0, 0.0, 0.0, 0.0];

/// Design context: `pi_fs = π/fs` as float32 (WING-Edit's `_AEDRV+0x10`) and the 44.1 kHz table switch.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Ctx {
    pub pi_fs: f32,
    pub fs: f32,
    /// `fs < 46 kHz`: models with sample-rate-specific tables use their 44.1 kHz set.
    pub fs44: bool,
}

impl Ctx {
    pub fn new(fs: f64) -> Self {
        Self {
            pi_fs: (core::f64::consts::PI / fs) as f32,
            fs: fs as f32,
            fs44: fs < 46_000.0,
        }
    }
}

/// ln(10)/20 as used by WING-Edit (`exp(g * c)` = 10^(g/20)).
pub const DB_TO_LN: f64 = f64::from_bits(0x3fbd_791c_6109_b02d);

/// `(float) exp((double) g * 0.115129255)` = 10^(g/20); the WING-Edit idiom for dB -> linear.
#[inline]
pub fn db_lin(g: f32) -> f32 {
    ((g as f64) * DB_TO_LN).exp() as f32
}

/// `db_lin` with WING-Edit's `> -98 dB` guard (else 0).
#[inline]
pub fn db_lin_guard(g: f32) -> f32 {
    if g > -98.0 {
        db_lin(g)
    } else {
        0.0
    }
}

/// `tanf(pi_fs * f)` (float32 pre-warp).
#[inline]
pub fn tan_w(ctx: &Ctx, f: f32) -> f32 {
    (ctx.pi_fs * f).tan()
}

/// `fmadd`: `a * b + c` with a single rounding.
#[inline]
pub fn fma(a: f32, b: f32, c: f32) -> f32 {
    a.mul_add(b, c)
}

/// Gain-column interpolation `(1-t)*row[i] + t*row[i+1]` as WING-Edit computes it.
#[inline]
pub fn lerp_col(row: &[f32], i: usize, t: f32) -> f32 {
    fma(1.0 - t, row[i], t * row[i + 1])
}

/// Knob-anchor interpolation between rows `r[j]` and `r[j+1]` at position `k` (`k` in `[0, n-1]`,
/// `j = min(floor(k), n-2)`): `r[j+1]*(k-j) + (j+1-k)*r[j]`.
#[inline]
pub fn lerp_anchor(r: &[f32], k: f32) -> f32 {
    let n = r.len();
    let mut j = 0usize;
    while j + 2 < n && k >= (j + 1) as f32 {
        j += 1;
    }
    let jf = j as f32;
    if j == 0 {
        fma(r[1], k, (1.0 - k) * r[0])
    } else {
        fma(r[j + 1], k - jf, (jf + 1.0 - k) * r[j])
    }
}

/// Value of a `rows x ncols` table: gain-column interpolation (`i`, `t`) of every row from `row0` on, then
/// knob-anchor interpolation over `nrows` rows at position `k`.
#[inline]
pub fn tab_k(
    tab: &[f32],
    ncols: usize,
    row0: usize,
    nrows: usize,
    k: f32,
    i: usize,
    t: f32,
) -> f32 {
    let mut r = [0.0f32; 5];
    for (j, v) in r[..nrows].iter_mut().enumerate() {
        *v = lerp_col(&tab[(row0 + j) * ncols..], i, t);
    }
    lerp_anchor(&r[..nrows], k)
}

/// Knob position 0..10 mapped to `0..anchors-1` and clamped (`clamp(knob*(n-1)/10, 0, n-1)`).
#[inline]
pub fn knob_pos(knob: f32, anchors: usize) -> f32 {
    let m = (anchors - 1) as f32;
    let k = if anchors == 3 {
        (knob + knob) / 10.0
    } else {
        (knob * m) / 10.0
    };
    k.max(0.0).min(m)
}

/// Integer column and fraction of a gain position (`i = min(trunc(x), max_i)`, `t = x - i`).
#[inline]
pub fn col(x: f32, max_i: i32) -> (usize, f32) {
    let i = (x as i32).min(max_i).max(0);
    (i as usize, x - i as f32)
}

/// `Cbiquad::hqpeq(f, g, q)`: Orfanidis peaking EQ (prescribed Nyquist gain, G0 = 1) with WING's Q -> bandwidth
/// map (Q relative to the peak gain), a Q stretch above 13.9 kHz and a band-edge fold near Nyquist.
pub fn hqpeq(ctx: &Ctx, f: f32, g: f32, q: f32) -> Bq {
    const W_HI: f64 = f64::from_bits(0x3ffd_276b_3909_0b17);
    const Q_A: f64 = f64::from_bits(0x3fd5_88c2_d97b_5355);
    const Q_B: f64 = f64::from_bits(0xbfe3_9e77_69c3_e8f8);
    const INV_LOG10_2: f64 = f64::from_bits(0x400a_93fc_9999_18c0);
    const LN2_HALF: f64 = f64::from_bits(0x3fd6_2e42_fead_c49c);
    const LN2: f64 = f64::from_bits(0x3fe6_2e42_ff36_b4fb);
    const HALF_PI: f64 = f64::from_bits(0x3ff9_21fb_5452_4550);
    const PI_D: f64 = f64::from_bits(0x4009_21fb_5452_4550);
    const PI07: f64 = f64::from_bits(0x4001_97c9_87d3_3084);
    const PI02: f64 = f64::from_bits(0x3fe4_1b2f_76a8_3774);
    const LOG2E: f64 = f64::from_bits(0x3ff7_1547_652b_720b);
    let pi2: f32 = f32::from_bits(0x411d_e9e6);

    let w0 = (ctx.pi_fs + ctx.pi_fs) * f;
    let a = db_lin_guard(g);
    let w0d = w0 as f64;
    let mut q = q;
    if w0d > W_HI {
        q = ((w0d.mul_add(Q_A, Q_B)).exp() * q as f64) as f32;
    }
    let gm = if g < 0.0 { 1.0 / a } else { a };
    let r = q / gm.sqrt();
    let r = r * r;
    let s = fma(r, 2.0, 1.0) / r;
    let x = (s as f64).mul_add(0.5, ((s * s) as f64).mul_add(0.25, -1.0).sqrt());
    let bw = x.log10() * INV_LOG10_2;

    let gp = if (g.abs() as f64) < 0.001 {
        f32::from_bits(0xba83_126f)
    } else {
        g
    };
    let gl = db_lin_guard(gp);
    let bwf = bw as f32;
    let gb = db_lin_guard(gp * 0.5);
    let w0sq = w0 * w0;
    let g2 = gl * gl;
    let mut bwd = bwf as f64;
    let mut e = (bwd * LN2_HALF).exp();
    let s12 = e as f32;
    let up = w0 * s12;
    let upd = up as f64;
    // Below the fold WING-Edit keeps G (not GB^2 = sqrt(G)^2) in the register it later uses as GB^2 for
    // F, F00, F01 and F11; only B uses the squared value. Mirrored for bit-faithfulness.
    let (gb2, gb2_b) = if upd < HALF_PI {
        (gl, gb * gb)
    } else {
        let t: f32 = if upd < PI07 {
            ((upd + -HALF_PI) / PI02) as f32
        } else {
            1.0
        };
        let s0 = up * (1.0 - t);
        let wu = ((t as f64) * 0.5).mul_add(w0d + PI_D, s0 as f64) as f32;
        let wu2 = wu * wu;
        let nbw = (((wu2 / w0sq).ln() as f64) * LOG2E) as f32;
        let dw = w0 * (s12 + (-1.0 / s12));
        let s1 = ((gl + -1.0) / (g2 - gl)).sqrt() * dw;
        let s2 = gl * s1;
        let s3 = wu2 - w0sq;
        let num = fma(s3, s3, s2 * s2 * wu2);
        let den = fma(s3, s3, s1 * s1 * wu2);
        let v = (num / den).sqrt();
        bwd = nbw as f64;
        e = (bwd * LN2_HALF).exp();
        (v * v, v * v)
    };
    let tn = (ctx.pi_fs * f).tan();
    let mut fden = (g2 - gb2).abs();
    if (fden as f64) < f64::from_bits(0x3ee4_f8b5_88e3_68f1) {
        fden = f32::from_bits(0x3727_c5ac);
    }
    let qq = (e / ((bwd * LN2).exp() + -1.0)) as f32;
    let dw = w0 / qq;
    let dw2 = dw * dw;
    let g00 = (g2 + -1.0).abs();
    let f00 = (gb2 + -1.0).abs();
    let p = w0sq + -pi2;
    let num = fma(p, p, (g2 * f00 * pi2 * dw2) / fden);
    let den = fma(p, p, (f00 * pi2 * dw2) / fden);
    let g1 = (num / den).sqrt();
    let g01 = (g2 - g1).abs();
    let g11 = (-g1).mul_add(g1, g2).abs();
    let f01 = (gb2 - g1).abs();
    let f11 = (-g1).mul_add(g1, gb2).abs();
    let w2 = tn * (tn * (g11 / g00).sqrt());
    let dwt = ((((dw as f64) * 0.5).tan()) * (fma((f00 / f11).sqrt(), w2, 1.0) as f64)) as f32;
    let w22 = w2 + w2;
    let c = fma(f11 * dwt, dwt, (f01 - (f00 * f11).sqrt()) * -w22);
    let d = (g01 - (g00 * g11).sqrt()) * w22;
    let aa = ((d + c) / fden).sqrt();
    let bb = (fma(g2, c, gb2_b * d) / fden).sqrt();
    let opw = w2 + 1.0;
    let norm = 1.0 / (opw + aa);
    let b0 = (g1 + w2 + bb) * norm;
    let b1 = ((g1 - w2) * -2.0) * norm;
    let b2 = (w2 + (g1 - bb)) * norm;
    let a1n = ((1.0 - w2) + (1.0 - w2)) * norm;
    let a2n = norm * -(opw - aa);
    [b0, b1, b2, -a1n, -a2n]
}

/// `Cbiquad::ls12q2(f, g, q1, q2)`: bilinear low shelf, `q1` on the gain-scaled polynomial, `q2` on the other.
pub fn ls12q2(ctx: &Ctx, f: f32, g: f32, q1: f32, q2: f32) -> Bq {
    let k = tan_w(ctx, f);
    let k2 = k * k;
    let a = if g > -98.0 { db_lin(g) } else { 0.0 };
    if g > -98.0 && a >= 1.0 {
        let s3 = k / q1;
        let n = 1.0 / (s3 + 1.0 + k2);
        let s4 = a.sqrt() * k / q2;
        let b1 = fma(a + a, k2, -2.0) * n;
        let b0 = fma(a, k2, s4 + 1.0) * n;
        let b2 = fma(a, k2, 1.0 - s4) * n;
        let a1n = (-(k2 * 2.0) - -2.0) * n;
        let a2n = n * -(k2 + (1.0 - s3));
        [b0, b1, b2, -a1n, -a2n]
    } else {
        let v = if g < 98.0 { db_lin(-g) } else { 0.0 };
        let s2 = k * v.sqrt() / q1;
        let n = 1.0 / fma(v, k2, s2 + 1.0);
        let s4 = k / q2;
        let b1 = fma(k2, 2.0, -2.0) * n;
        let b0 = (k2 + (s4 + 1.0)) * n;
        let b2 = (k2 + (1.0 - s4)) * n;
        let a1n = (-((v + v) * k2) - -2.0) * n;
        let a2n = n * -fma(v, k2, 1.0 - s2);
        [b0, b1, b2, -a1n, -a2n]
    }
}

/// `Cbiquad::hs12q2(f, g, q1, q2)`: bilinear high shelf (mirror of [`ls12q2`]).
pub fn hs12q2(ctx: &Ctx, f: f32, g: f32, q1: f32, q2: f32) -> Bq {
    let k = tan_w(ctx, f);
    let k2 = k * k;
    let a = if g > -98.0 { db_lin(g) } else { 0.0 };
    if g > -98.0 && a >= 1.0 {
        let s0 = k / q1;
        let n = 1.0 / (k2 + (s0 + 1.0));
        let s3 = a.sqrt() * k / q2;
        let b1 = fma(k2, 2.0, a * -2.0) * n;
        let b0 = (k2 + (s3 + a)) * n;
        let b2 = (k2 + (a - s3)) * n;
        let a1n = -fma(k2, 2.0, -2.0) * n;
        let a2n = -(k2 + (1.0 - s0)) * n;
        [b0, b1, b2, -a1n, -a2n]
    } else {
        let v = if g < 98.0 { db_lin(-g) } else { 0.0 };
        let sv = v.sqrt();
        let n = 1.0 / (k2 + (v + k * sv / q1));
        let s4 = k / q2;
        let b1 = fma(k2, 2.0, -2.0) * n;
        let b0 = (k2 + (s4 + 1.0)) * n;
        let b2 = (k2 + (1.0 - s4)) * n;
        let s0 = k * (1.0 / sv) / q1;
        let k2v = k2 / v;
        let n2 = 1.0 / (k2v + (s0 + 1.0));
        let a1n = -((k2 + k2) / v + -2.0) * n2;
        let a2n = -(k2v + (1.0 - s0)) * n2;
        [b0, b1, b2, -a1n, -a2n]
    }
}

/// First-order low shelf written inline by `eq_parchg_ssl_g` (LF band, second section): pole/zero at the
/// bilinear corner `f`, DC gain `g` dB.
pub fn ls6_ssl(ctx: &Ctx, f: f32, g: f32) -> Bq {
    let k = tan_w(ctx, f);
    let (a, m) = if g > -98.0 {
        let a = db_lin(g);
        if a >= 1.0 {
            (a, 1.0)
        } else {
            (a, a)
        }
    } else {
        (0.0, 0.0)
    };
    let num = if m == 1.0 && a >= 1.0 {
        k + -1.0
    } else {
        k - m
    };
    let c = num / (k + m);
    let h = ((a + -1.0) * (c + 1.0)) as f64;
    let b0 = h.mul_add(0.5, 1.0) as f32;
    let b1 = h.mul_add(0.5, c as f64) as f32;
    [b0, b1, 0.0, c, 0.0]
}

#[cfg(test)]
mod tests {
    use super::*;

    fn mag_db(q: &Bq, fs: f64, f: f64) -> f64 {
        crate::fidelity::cascade_db(&[q.map(f64::from)], fs, f)
    }

    #[test]
    fn peq_hits_gain_and_shelves_settle() {
        let c = Ctx::new(48_000.0);
        let p = hqpeq(&c, 1000.0, 6.0, 1.0);
        assert!((mag_db(&p, 48e3, 1000.0) - 6.0).abs() < 0.01);
        assert!(mag_db(&p, 48e3, 20.0).abs() < 0.05);
        let l = ls12q2(&c, 100.0, -6.0, 0.7, 0.7);
        assert!((mag_db(&l, 48e3, 5.0) + 6.0).abs() < 0.05);
        assert!(mag_db(&l, 48e3, 10_000.0).abs() < 0.05);
        let h = hs12q2(&c, 5000.0, 9.0, 0.7, 0.7);
        assert!((mag_db(&h, 48e3, 23_000.0) - 9.0).abs() < 0.1);
        assert!(mag_db(&h, 48e3, 50.0).abs() < 0.05);
        let s = ls6_ssl(&c, 200.0, -4.0);
        assert!((mag_db(&s, 48e3, 1.0) + 4.0).abs() < 0.05);
        assert!(lerp_anchor(&[0.0, 10.0, 30.0], 1.5) == 20.0);
        assert_eq!(col(4.5, 4), (4, 0.5));
    }
}
