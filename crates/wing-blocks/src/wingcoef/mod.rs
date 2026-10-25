//! Exact WING coefficient design (T03C): the biquad routines WING-Edit 3.3.4 uses for the channel filter, the
//! STD EQ (ch/aux/bus), the dynamics key filter (`dynsc`/`gatesc`) and the dynamics crossover (`dynxo`).
//!
//! Every function is a line-by-line port of the arm64 code (`Cbiquad::*`, `Cxover48::Cxover48`,
//! `ae::eq_parchg_xseries[_bus]`, `ae::Cchfilter::update`, `ae::Cdynsidechain::update`,
//! `ae::Ccompxover::update`; addresses in `docs/research/reference-curves.md` and the T03C log), including
//! its float32 arithmetic, fused multiply-adds, double-precision `exp`/`tan` calls and its constants (for
//! example `1/0.301` instead of `1/log10(2)`), so the coefficients match the `biquads` in
//! `data/reference-curves/*.json` to float32 rounding. These coefficients are what WING-Edit draws and posts to
//! the engine; that the console computes the same is an assumption (same `parchg` code by name).
//!
//! Design families (all evaluated at `fs`, tables switch at 46 kHz like WING-Edit):
//! - [`peq`]: `hqpeq`, Orfanidis (JAES 1997) peaking EQ with prescribed Nyquist gain. Q is applied to the
//!   un-boosted side: bandwidth from `Q' = Q / sqrt(G)` (`G` = linear boost >= 1) in octaves, the band-edge
//!   gain is the dB midpoint, edges past pi/2 are pulled toward Nyquist, and above w0 = 1.822 rad
//!   (13.9 kHz @ 48k) Q is widened by `1.4^(w0 - 1.822)`.
//! - [`bp`]: `hqbp`, the same design at +40 dB and Q/10, scaled by -40 dB (unity-peak band-pass).
//! - [`ls12`]/[`hs12`]: 2nd-order bilinear shelves. The `f` corner belongs to the unity side; the boosted side
//!   is the same polynomial scaled by `G` (zeros at `f*sqrt(G)`), so +6 dB is +3.9 dB at `f` for Q = 0.707.
//! - [`hqhs12`], [`hqls12`], [`hqhs6`], [`hqls6`]: the same shelves with (f, Q, gain) remapped through
//!   measured-looking tables above 2.5 kHz (200 Hz for `hqls6`) to undo bilinear cramping.
//! - [`lp12`]: `hqlp12`, Massberg (AES 2011) analog-matched 2nd-order low-pass; [`lp6`] its 1st-order form.
//! - [`hp12`]: `hqhp12`, bilinear 2nd-order high-pass whose (gain, f, Q) are remapped by a table above 2.5 kHz;
//!   [`hp6`] the table-remapped 1st-order high-pass (`hqhp6`, inlined in WING-Edit); [`hp12_bilinear`] plain.
//! - [`xover`]: `Cxover48`, the BW/BS/LR 6..48 dB/oct cut shapes of the bus EQ.

mod tables;

use core::f64::consts::PI;

/// One biquad `[b0, b1, b2, a1, a2]`, `H(z) = (b0 + b1 z^-1 + b2 z^-2) / (1 + a1 z^-1 + a2 z^-2)`.
pub type Bq = [f32; 5];
/// Identity section.
pub const IDENTITY: Bq = [1.0, 0.0, 0.0, 0.0, 0.0];

/// Sample-rate context (`_AEDRV`: fs, pi/fs as float).
#[derive(Clone, Copy, Debug)]
pub struct Ctx {
    pub fs: f64,
    pifs: f32,
    /// 48 kHz table set (fs >= 46 kHz).
    t: usize,
}

impl Ctx {
    pub fn new(fs: f64) -> Self {
        Self {
            fs,
            pifs: (PI / fs) as f32,
            t: usize::from(fs as f32 >= 46_000.0),
        }
    }

    /// `pi/fs` as the float WING-Edit stores.
    pub fn pi_over_fs(&self) -> f32 {
        self.pifs
    }
}

const LN10_20: f64 = f64_bits(0x3fbd791c6109b02d); // 0.115129255
const W_HI: f64 = f64_bits(0x3ffd276b39090b17); // 1.82212374 rad
const Q_HI_A: f64 = f64_bits(0x3fd588c2d97b5355); // ln 1.4
const Q_HI_B: f64 = f64_bits(0xbfe39e7769c3e8f8); // -ln 1.4 * W_HI
const INV_0301: f64 = f64_bits(0x400a93fc999918c0); // 1/0.301
const LN2_2: f64 = f64_bits(0x3fd62e42fead449c); // ln2 / 2
const LN2: f64 = f64_bits(0x3fe62e42ff36b4fb);
const INV_LN2: f64 = f64_bits(0x3ff71547652b720b);
const HALF_PI: f64 = f64_bits(0x3ff921fb54524550);
const EDGE_MAX: f64 = f64_bits(0x400197c987d33084); // pi/2 + pi/5
const PI_5: f64 = f64_bits(0x3fe41b2f76a83774); // pi/5
const PI_C: f64 = f64_bits(0x400921fb54524550);
const TWO_PI: f64 = f64_bits(0x401921fb54524550);
const PI_SQRT2: f64 = f64_bits(0x4011c5831ad33748); // pi*sqrt(2)
const PI2_F: f32 = 9.869_604; // 0x411de9e6
const SQRT1_2: f32 = core::f32::consts::FRAC_1_SQRT_2; // 0x3f3504f3

const fn f64_bits(b: u64) -> f64 {
    f64::from_bits(b)
}

/// `exp(g * ln10/20)` as float, 0 for `g <= -98` (the WING `dB -> lin` idiom).
#[inline]
pub fn db2lin(g: f32) -> f32 {
    if g > -98.0 {
        (g as f64 * LN10_20).exp() as f32
    } else {
        0.0
    }
}

#[inline]
fn fms(a: f32, b: f32, c: f32) -> f32 {
    // c - a*b, fused (AArch64 FMSUB)
    (-a).mul_add(b, c)
}

/// Biquad store order of WING-Edit (b2, b0, b1, A2, A1 with `y += A1 y[-1] + A2 y[-2]`) -> [`Bq`].
#[inline]
fn bq(b0: f32, b1: f32, b2: f32, big_a1: f32, big_a2: f32) -> Bq {
    [b0, b1, b2, -big_a1, -big_a2]
}

// ------------------------------------------------------------------------------------------------------------
// Orfanidis peaking EQ (hqpeq / hqbp)
// ------------------------------------------------------------------------------------------------------------

/// Q widening above 13.9 kHz (shared by `hqpeq` and `hqbp`).
#[inline]
fn hf_q(w0: f32, q: f32) -> f32 {
    if (w0 as f64) > W_HI {
        ((w0 as f64).mul_add(Q_HI_A, Q_HI_B).exp() * q as f64) as f32
    } else {
        q
    }
}

/// Bandwidth in octaves for Q (WING: `log10(x)/0.301`).
#[inline]
fn bw_oct(q: f32) -> f32 {
    let s1 = q * q;
    let s0 = s1.mul_add(2.0, 1.0) / s1;
    let d1 = s0 as f64;
    let sq = s0 * s0;
    let d0 = (sq as f64).mul_add(0.25, -1.0).sqrt();
    let d0 = d1.mul_add(0.5, d0);
    (d0.log10() * INV_0301) as f32
}

/// Upper band edge pulled toward Nyquist when it passes pi/2; returns (w2'^2, new bandwidth in octaves).
#[inline]
fn pull_edge(w0: f32, w2: f32, w02: f32) -> (f32, f32) {
    let mut s3 = 1.0f32;
    if (w2 as f64) < EDGE_MAX {
        s3 = ((w2 as f64 - HALF_PI) / PI_5) as f32;
    }
    let s0 = w2 * (1.0 - s3);
    let s0 = (s3 as f64 * 0.5).mul_add(PI_C + w0 as f64, s0 as f64) as f32;
    let s8 = s0 * s0;
    let nb = ((s8 / w02).ln() as f64 * INV_LN2) as f32;
    (s8, nb)
}

/// `Cbiquad::hqpeq(f, gain_db, q)`.
pub fn peq(c: &Ctx, f: f32, g: f32, q: f32) -> Bq {
    let w0 = (c.pifs + c.pifs) * f;
    let g_in = db2lin(g);
    let q = hf_q(w0, q);
    let gb = if g < 0.0 { 1.0 / g_in } else { g_in };
    let qp = q / gb.sqrt();
    let bw = bw_oct(qp);
    let gc = if ((g.abs()) as f64) < 0.001 {
        -0.001f32
    } else {
        g
    };
    let gg = db2lin(gc);
    let gbb = db2lin(gc * 0.5);
    let w02 = w0 * w0;
    let g2 = gg * gg;
    let mut d15 = bw as f64;
    let mut d9 = (d15 * LN2_2).exp();
    let s12 = d9 as f32;
    let w2 = w0 * s12;
    let (gb2, s13);
    if (w2 as f64) < HALF_PI {
        gb2 = gbb * gbb;
        s13 = gg;
    } else {
        let (s8, nb) = pull_edge(w0, w2, w02);
        let s2 = w0 * (s12 + (-1.0 / s12));
        let a = s2 * ((gg + -1.0) / (g2 - gg)).sqrt();
        let b = gg * a;
        let s3 = s8 - w02;
        let t2 = s3.mul_add(s3, b * b * s8);
        let t1 = s3.mul_add(s3, a * a * s8);
        let r = (t2 / t1).sqrt();
        s13 = r * r;
        gb2 = s13;
        d15 = nb as f64;
        d9 = (d15 * LN2_2).exp();
    }
    let t = (c.pifs * f).tan();
    let mut fv = (g2 - s13).abs();
    if (fv as f64) < 1e-5 {
        fv = f32::from_bits(0x3727c5ac);
    }
    orf_tail(OrfIn {
        w0,
        w02,
        t,
        g2,
        gb2_sq: s13,
        gb2_b: gb2,
        fv,
        d15,
        d9,
        g00_fixed: None,
        out_scale: None,
    })
}

struct OrfIn {
    w0: f32,
    w02: f32,
    t: f32,
    g2: f32,
    /// GB^2 used in F00/F01/F11.
    gb2_sq: f32,
    /// GB^2 used in the B term.
    gb2_b: f32,
    fv: f32,
    d15: f64,
    d9: f64,
    /// `hqbp` uses the constant 9999 for G00.
    g00_fixed: Option<f32>,
    /// `hqbp` scales the numerator by 0.01 (b0, b2) and -0.02 (b1).
    out_scale: Option<(f32, f32)>,
}

fn orf_tail(i: OrfIn) -> Bq {
    let qbw = (i.d9 / ((i.d15 * LN2).exp() - 1.0)) as f32;
    let dw = i.w0 / qbw;
    let dw2 = dw * dw;
    let g00 = i.g00_fixed.unwrap_or_else(|| (i.g2 + -1.0).abs());
    let f00 = (i.gb2_sq + -1.0).abs();
    let s2 = i.w02 + -PI2_F;
    let num = s2.mul_add(s2, i.g2 * f00 * PI2_F * dw2 / i.fv);
    let den = s2.mul_add(s2, f00 * PI2_F * dw2 / i.fv);
    let g1 = (num / den).sqrt();
    let g01 = (i.g2 - g1).abs();
    let g11 = fms(g1, g1, i.g2).abs();
    let f01 = (i.gb2_sq - g1).abs();
    let f11 = fms(g1, g1, i.gb2_sq).abs();
    let w2 = i.t * (i.t * (g11 / g00).sqrt());
    let d9b = (f00 / f11).sqrt().mul_add(w2, 1.0) as f64;
    let big_dw = ((dw as f64 * 0.5).tan() * d9b) as f32;
    let s2 = w2 + w2;
    let cc = (f11 * big_dw).mul_add(big_dw, (f01 - (f00 * f11).sqrt()) * -s2);
    let dd = (g01 - (g00 * g11).sqrt()) * s2;
    let aa = ((dd + cc) / i.fv).sqrt();
    let bb = (i.g2.mul_add(cc, i.gb2_b * dd) / i.fv).sqrt();
    let s1 = w2 + 1.0;
    let n = 1.0 / (s1 + aa);
    let (b0, b1, b2) = match i.out_scale {
        None => (
            ((g1 + w2) + bb) * n,
            ((g1 - w2) * -2.0) * n,
            (w2 + (g1 - bb)) * n,
        ),
        Some((k, k2)) => (
            ((g1 + w2) + bb) * k * n,
            (g1 - w2) * k2 * n,
            n * ((w2 + (g1 - bb)) * k),
        ),
    };
    let big_a1 = ((1.0 - w2) + (1.0 - w2)) * n;
    let big_a2 = n * -(s1 - aa);
    bq(b0, b1, b2, big_a1, big_a2)
}

/// `Cbiquad::hqbp(f, q)`: unity-peak band-pass (Orfanidis at +40 dB, Q/10, output -40 dB).
pub fn bp(c: &Ctx, f: f32, q: f32) -> Bq {
    let w0 = (c.pifs + c.pifs) * f;
    let q = hf_q(w0, q);
    let bw = bw_oct(q / 10.0);
    let w02 = w0 * w0;
    let mut d12 = bw as f64;
    let mut d9 = (d12 * LN2_2).exp();
    let s8 = d9 as f32;
    let w2 = w0 * s8;
    let (gb, gb2);
    if (w2 as f64) < HALF_PI {
        gb2 = 100.0f32;
        gb = 10.0f32;
    } else {
        let (s11, nb) = pull_edge(w0, w2, w02);
        let a = w0 * (s8 - 1.0 / s8) * 0.1;
        let b = a * 100.0;
        let s3 = s11 - w02;
        let t2 = s3.mul_add(s3, b * b * s11);
        let t1 = s3.mul_add(s3, a * a * s11);
        gb = (t2 / t1).sqrt();
        gb2 = gb * gb;
        d12 = nb as f64;
        d9 = (d12 * LN2_2).exp();
    }
    let t = (c.pifs * f).tan();
    let g2 = 10_000.0f32;
    let mut fv = (g2 - gb2).abs();
    if (fv as f64) < 1e-5 {
        fv = f32::from_bits(0x3727c5ac);
    }
    orf_tail(OrfIn {
        w0,
        w02,
        t,
        g2,
        gb2_sq: gb2,
        gb2_b: gb * gb,
        fv,
        d15: d12,
        d9,
        g00_fixed: Some(9999.0),
        out_scale: Some((0.01, -0.02)),
    })
}

// ------------------------------------------------------------------------------------------------------------
// Bilinear 2nd-order shelves (ls12 / hs12 and the table-driven hq variants)
// ------------------------------------------------------------------------------------------------------------

/// Low-shelf core with a numerator scale `m` (`hqls12` make-up; 1 for `ls12`).
fn ls_core(k: f32, g: f32, q: f32, m: Option<f32>) -> Bq {
    let k2 = k * k;
    let gl = db2lin(g);
    if g > -98.0 && gl >= 1.0 {
        let s3 = k / q;
        let n = 1.0 / (k2 + (s3 + 1.0));
        let s4 = gl.sqrt() * k / q;
        let b1 = (gl + gl).mul_add(k2, -2.0);
        let b0 = gl.mul_add(k2, s4 + 1.0);
        let b2 = gl.mul_add(k2, 1.0 - s4);
        let (b0, b1, b2) = match m {
            None => (b0 * n, b1 * n, b2 * n),
            Some(m) => (n * (m * b0), (m * b1) * n, n * (m * b2)),
        };
        let a1 = n * -(k2.mul_add(2.0, -2.0));
        let a2 = n * -(k2 + (1.0 - s3));
        bq(b0, b1, b2, a1, a2)
    } else {
        let gi = if g < 98.0 { db2lin(-g) } else { 0.0 };
        let s2 = gi.sqrt() * k / q;
        let n = 1.0 / gi.mul_add(k2, s2 + 1.0);
        let s4 = k / q;
        let b1 = k2.mul_add(2.0, -2.0);
        let b0 = k2 + (s4 + 1.0);
        let b2 = k2 + (1.0 - s4);
        let (b0, b1, b2) = match m {
            None => (b0 * n, b1 * n, b2 * n),
            Some(m) => ((b0 * m) * n, (m * b1) * n, (b2 * m) * n),
        };
        let a1 = n * -((gi + gi).mul_add(k2, -2.0));
        let a2 = n * -gi.mul_add(k2, 1.0 - s2);
        bq(b0, b1, b2, a1, a2)
    }
}

/// High-shelf core (`hs12`, and `hqhs12` after its table step).
fn hs_core(k: f32, g: f32, q: f32) -> Bq {
    let k2 = k * k;
    let gl = db2lin(g);
    if g > -98.0 && gl >= 1.0 {
        let s1 = k / q;
        let n = 1.0 / (k2 + (s1 + 1.0));
        let s3 = gl.sqrt() * k / q;
        let b1 = k2.mul_add(2.0, gl * -2.0) * n;
        let b0 = n * (k2 + (s3 + gl));
        let b2 = n * (k2 + (gl - s3));
        let a1 = n * -(k2.mul_add(2.0, -2.0));
        let a2 = n * -(k2 + (1.0 - s1));
        bq(b0, b1, b2, a1, a2)
    } else {
        let gi = if g < 98.0 { db2lin(-g) } else { 0.0 };
        let sg = gi.sqrt();
        let n = 1.0 / (k2 + (gi + k * sg / q));
        let s4 = k / q;
        let b1 = k2.mul_add(2.0, -2.0) * n;
        let b0 = (k2 + (s4 + 1.0)) * n;
        let b2 = (k2 + (1.0 - s4)) * n;
        let n2 = 1.0 / (k2 / gi + (1.0 / q / sg).mul_add(k, 1.0));
        let a1 = n2 * -((k2 + k2) / gi + -2.0);
        let s1 = k * (1.0 / sg) / q;
        let a2 = n2 * -(k2 / gi + (1.0 - s1));
        bq(b0, b1, b2, a1, a2)
    }
}

/// `Cbiquad::ls12(f, g, q)`: bilinear 2nd-order low shelf, corner `f` on the unity side.
pub fn ls12(c: &Ctx, f: f32, g: f32, q: f32) -> Bq {
    ls_core((c.pifs * f).tan(), g, q, None)
}

/// `Cbiquad::hs12(f, g, q)`: bilinear 2nd-order high shelf, corner `f` on the unity side.
pub fn hs12(c: &Ctx, f: f32, g: f32, q: f32) -> Bq {
    hs_core((c.pifs * f).tan(), g, q)
}

/// Linear (f-row, |g|-column) table interpolation used by `hqhs12`/`hqls12`/`hqhs6` above 2.5 kHz.
/// Returns (row index i in 1..=7, f fraction, column j, g fraction).
#[inline]
fn grid_2k5(f: f32, g: f32) -> (usize, f32, usize, f32) {
    let i = ((f as f64 * 0.0004) as i32).clamp(1, 7);
    let fr = (((i as f32).mul_add(-2500.0, f)) as f64 * 0.0004) as f32;
    let ga = g.abs();
    let j = ((ga * 4.0 / 15.0) as i32).clamp(0, 5) as usize;
    let gf = ((j as f32 as f64).mul_add(-3.75, ga as f64) * 4.0 / 15.0) as f32;
    (i as usize, fr, j, gf)
}

/// Bilinear interpolation in a 7-column table: rows i-1/i (weights 1-fr / fr), columns j/j+1 (1-gf / gf).
#[inline]
fn interp7<const R: usize>(t: &[[f32; 7]; R], i: usize, fr: f32, j: usize, gf: f32) -> f32 {
    let s4 = 1.0 - fr;
    let c0 = s4.mul_add(t[i - 1][j], t[i][j] * fr);
    let c1 = s4.mul_add(t[i - 1][j + 1], t[i][j + 1] * fr);
    (1.0 - gf).mul_add(c0, c1 * gf)
}

/// `Cbiquad::hqhs12(f, g)`: high shelf, Q 0.707 below 2.5 kHz, table-remapped (f, Q, g) above.
pub fn hqhs12(c: &Ctx, f: f32, g: f32) -> Bq {
    let (f, g, q) = if f > 2500.0 {
        let (i, fr, j, gf) = grid_2k5(f, g);
        let ft = interp7(&tables::HS12_F[c.t], i, fr, j, gf);
        let q = 1.0 / interp7(&tables::HS12_Q[c.t], i, fr, j, gf);
        let gm = interp7(&tables::HS12_G[c.t], i, fr, j, gf);
        (ft, if g > 0.0 { gm } else { -gm }, q)
    } else {
        (f, g, SQRT1_2)
    };
    hs_core((f * c.pifs).tan(), g, q)
}

/// `Cbiquad::hqhs12q25(f, g)` (T05Q, SPKMAN tilt): the `hqhs12` design with Q 0.4 below 2.5 kHz, `f` clamped
/// to <= 15 kHz, `g` to +-12 dB, and its own (f, 1/Q, |g|) tables above 2.5 kHz (6 rows of 2.5 kHz x 5
/// columns of 3 dB).
#[allow(clippy::manual_clamp)] // fmax/fmin order of the original
pub fn hqhs12q25(c: &Ctx, f: f32, g: f32) -> Bq {
    let f = f.min(15_000.0);
    let g = g.max(-12.0).min(12.0);
    let (f, g, q) = if f > 2500.0 {
        let i = ((f as f64 * 0.0004) as i32).clamp(1, 5) as usize;
        let fr = (((i as f32).mul_add(-2500.0, f)) as f64 * 0.0004) as f32;
        let ga = g.abs();
        let j = ((ga / 3.0) as i32).min(3) as usize;
        let gf = fms(j as f32, 3.0, ga) / 3.0;
        let ip = |t: &[[f32; 5]; 6]| {
            let s4 = 1.0 - fr;
            let c0 = s4.mul_add(t[i - 1][j], t[i][j] * fr);
            let c1 = s4.mul_add(t[i - 1][j + 1], t[i][j + 1] * fr);
            (1.0 - gf).mul_add(c0, gf * c1)
        };
        let ft = ip(&tables::HS12Q25_F[c.t]);
        let q = 1.0 / ip(&tables::HS12Q25_Q[c.t]);
        let gm = ip(&tables::HS12Q25_G[c.t]);
        (ft, if g > 0.0 { gm } else { -gm }, q)
    } else {
        (f, g, f32::from_bits(0x3ecccccd))
    };
    hs_core((f * c.pifs).tan(), g, q)
}

/// `Cbiquad::hqls12(f, g)`: low shelf, Q 0.707 below 2.5 kHz, table-remapped above with make-up gain.
pub fn hqls12(c: &Ctx, f: f32, g: f32) -> Bq {
    let (ft, gt, q) = if f > 2500.0 {
        let (i, fr, j, gf) = grid_2k5(f, g);
        let ft = interp7(&tables::LS12_F[c.t], i, fr, j, gf);
        let q = 1.0 / interp7(&tables::LS12_Q[c.t], i, fr, j, gf);
        let gm = interp7(&tables::LS12_G[c.t], i, fr, j, gf);
        (ft, if g > 0.0 { gm } else { -gm }, q)
    } else {
        (f, g, SQRT1_2)
    };
    let m = db2lin(g - gt);
    ls_core((ft * c.pifs).tan(), gt, q, Some(m))
}

/// First-order shelf core shared by `hqls6`/`hqhs6`. `low`: low shelf with make-up `m`.
fn shelf1(k: f32, g: f32, low: bool, m: f32) -> Bq {
    let gl = db2lin(g);
    let ge1 = g > -98.0 && gl >= 1.0;
    let p = if low {
        if ge1 {
            (k + -1.0) / (k + 1.0)
        } else {
            (k - gl) / (k + gl)
        }
    } else if ge1 {
        (k + -1.0) / (k + 1.0)
    } else {
        gl.mul_add(k, -1.0) / gl.mul_add(k, 1.0)
    };
    let (b0, b1) = if low {
        let d0 = ((gl + -1.0) * (p + 1.0)) as f64;
        let d2 = d0.mul_add(0.5, 1.0);
        let d1 = d0.mul_add(0.5, p as f64);
        ((m as f64 * d2) as f32, (m as f64 * d1) as f32)
    } else {
        let b0 = ((gl + -1.0) * (1.0 - p)) as f64;
        let b0 = b0.mul_add(0.5, 1.0) as f32;
        let b1 = ((gl + -1.0) * (p + -1.0)) as f64;
        let b1 = b1.mul_add(0.5, p as f64) as f32;
        (b0, b1)
    };
    [b0, b1, 0.0, p, 0.0]
}

/// `Cbiquad::hqls6(f, g)`: 1st-order low shelf, table-remapped (geometric in f) above 200 Hz, with make-up.
pub fn hqls6(c: &Ctx, f: f32, g: f32) -> Bq {
    let f = f.min(20_000.0);
    let (ft, gt) = if f > 200.0 {
        let tt = ((f.log10() as f64 + f64_bits(0xc00268826a1f7ae7)) * 4.0) as f32;
        let i = (tt as i32).clamp(0, 7) as usize;
        let fr = tt - i as f32;
        let ga = g.abs();
        let j = ((ga * 4.0 / 15.0) as i32).clamp(0, 5) as usize;
        let gf = ((j as f32 as f64).mul_add(-3.75, ga as f64) * 4.0 / 15.0) as f32;
        let tf = &tables::LS6_F[c.t];
        let tg = &tables::LS6_G[c.t];
        let geo = |t: &[[f32; 7]; 9], col: usize| t[i][col] * (t[i + 1][col] / t[i][col]).powf(fr);
        let f0 = geo(tf, j);
        let f1 = geo(tf, j + 1);
        let s12 = 1.0 - gf;
        let ft = s12.mul_add(f0, f1 * gf);
        let g0 = if j != 0 { geo(tg, j) } else { 0.0 };
        let g1 = geo(tg, j + 1) * gf;
        let gm = s12.mul_add(g0, g1);
        (ft, if g > 0.0 { gm } else { -gm })
    } else {
        (f, g)
    };
    let m = db2lin(g - gt);
    shelf1((ft * c.pifs).tan(), gt, true, m)
}

/// `Cbiquad::hqhs6(f, g)`: 1st-order high shelf, table-remapped above 2.5 kHz.
pub fn hqhs6(c: &Ctx, f: f32, g: f32) -> Bq {
    let f = f.min(20_000.0);
    let (ft, gt) = if f > 2500.0 {
        let (i, fr, j, gf) = grid_2k5(f, g);
        let ft = interp7(&tables::HS6_F[c.t], i, fr, j, gf);
        let gm = interp7(&tables::HS6_G[c.t], i, fr, j, gf);
        (ft, if g > 0.0 { gm } else { -gm })
    } else {
        (f, g)
    };
    shelf1((ft * c.pifs).tan(), gt, false, 1.0)
}

// ------------------------------------------------------------------------------------------------------------
// Cuts
// ------------------------------------------------------------------------------------------------------------

/// Plain bilinear 2nd-order high-pass (`K = tan(pi f/fs)`), as inlined for CUT (aux), BS12/24/BW24/48/LR24/48
/// low cuts and the channel 24 dB/oct low cut.
pub fn hp12_bilinear(c: &Ctx, f: f32, q: f32) -> Bq {
    let k = (c.pifs * f).tan();
    let k2 = k * k;
    let s0 = k / q;
    let n = 1.0 / (k2 + (s0 + 1.0));
    let a1 = (k2 + -1.0) * -2.0 * n;
    let a2 = n * -(k2 + (1.0 - s0));
    bq(n, n * -2.0, n, a1, a2)
}

/// Row lookup of the `hqhp6`/`hqhp12` tables: (row a, row b, fraction) for `f` in (2500, 50000].
#[inline]
fn hp_rows(f: f32) -> (usize, usize, f32) {
    if f >= 20_000.0 {
        let i = ((f as f64 * 0.0002) as i32).clamp(4, 9);
        let fr = ((i as f32).mul_add(-5000.0, f) as f64 * 0.0002) as f32;
        (i as usize + 3, i as usize + 4, fr)
    } else {
        let i = ((f as f64 * 0.0004) as i32).clamp(1, 7);
        let fr = ((i as f32).mul_add(-2500.0, f) as f64 * 0.0004) as f32;
        (i as usize - 1, i as usize, fr)
    }
}

/// `hqhp6` (inlined in WING-Edit): 1st-order bilinear high-pass, (gain, f) remapped by a table above 2.5 kHz.
pub fn hp6(c: &Ctx, f: f32) -> Bq {
    let f = f.min(50_000.0);
    let (gdb, ft) = if f > 2500.0 {
        let (ra, rb, fr) = hp_rows(f);
        let t = &tables::HP1[c.t];
        let s1 = 1.0 - fr;
        (
            t[ra][0].mul_add(s1, t[rb][0] * fr),
            t[ra][1].mul_add(s1, t[rb][1] * fr),
        )
    } else {
        (0.0, f)
    };
    let gl = if gdb > -98.0 {
        (gdb as f64 * LN10_20).exp() as f32
    } else {
        0.0
    };
    let t = (ft * c.pifs).tan();
    let s1 = t + 1.0;
    let b0 = gl / s1;
    let a1 = -(t + -1.0) / s1;
    bq(b0, -b0, 0.0, a1, 0.0)
}

/// `Cbiquad::hqhp12(f)`: 2nd-order bilinear high-pass with (gain, f, Q) remapped by a table above 2.5 kHz
/// (Q 0.707 below).
pub fn hp12(c: &Ctx, f: f32) -> Bq {
    let f = f.min(50_000.0);
    let (gdb, ft, q) = if f > 2500.0 {
        let (ra, rb, fr) = hp_rows(f);
        let t = &tables::HP2[c.t];
        let s2 = 1.0 - fr;
        let gdb = s2.mul_add(t[ra][0], t[rb][0] * fr);
        let ft = t[ra][1].mul_add(s2, t[rb][1] * fr);
        let iq = t[ra][2].mul_add(s2, t[rb][2] * fr);
        (gdb, ft, 1.0 / iq)
    } else {
        (0.0, f, SQRT1_2)
    };
    let gl = if gdb > -98.0 {
        (gdb as f64 * LN10_20).exp() as f32
    } else {
        0.0
    };
    let k = (ft * c.pifs).tan();
    let k2 = k * k;
    let s0 = k / q;
    let n = 1.0 / (k2 + (s0 + 1.0));
    let b0 = gl * n;
    let b1 = gl * (n * -2.0);
    let a1 = (k2 + -1.0) * -2.0 * n;
    let a2 = n * -(k2 + (1.0 - s0));
    bq(b0, b1, b0, a1, a2)
}

/// `Cbiquad::hqlp12(f, q)`: Massberg analog-matched 2nd-order low-pass.
pub fn lp12(c: &Ctx, f: f32, q: f32) -> Bq {
    let w0 = (c.pifs + c.pifs) * f;
    let d0 = PI_SQRT2 / w0 as f64;
    let d0 = (-d0).mul_add(d0, 2.0);
    let s11 = w0 * q;
    let d3 = TWO_PI / s11 as f64;
    let d0 = d0.mul_add(d0, d3 * d3).sqrt();
    let g1 = (2.0 / d0) as f32;
    let s9 = if ((1.0 - g1).abs() as f64) < 1e-6 {
        (g1 as f64 + -1e-6) as f32
    } else {
        g1
    };
    let os = if (q as f64) > f64_bits(0x3fe6a09e6665983e) {
        let s0 = (q + q) * q;
        let s1 = (q * 4.0).mul_add(q, -1.0).sqrt();
        let gp = ((s0 / s1) as f64).max(f64_bits(0x3ff000010c6f7a0b)) as f32;
        let s0 = ((-1.0 / s0 + 1.0).sqrt() * w0 * 0.5).tan().abs();
        let op = (s0 as f64).clamp(1e-6, 1e6) as f32;
        let s1 = gp.mul_add(gp, s9 * -s9);
        let s2 = gp.mul_add(gp, -1.0);
        (s1 / s2).sqrt().sqrt() * op
    } else {
        let q2 = q * q;
        let s2 = 2.0 - 1.0 / q2;
        let s3 = (q * -4.0).mul_add(q, 1.0);
        let s0 = s3 / (q2 * q * q) + 4.0 / s9;
        let s0 = ((s2 + s0.sqrt()) * 0.5).sqrt();
        let or = (w0 * s0 * 0.5).tan();
        let s1 = w0 * fms(s9, s9, 1.0).sqrt().sqrt() * 0.5;
        or.min(s1)
    };
    let mag = |w: f32| {
        let s1 = fms(w / w0, w / w0, 1.0);
        let s0 = w / s11;
        1.0 / s1.mul_add(s1, s0 * s0).sqrt()
    };
    let gs = mag(os.atan() + os.atan());
    let sg1 = s9.sqrt();
    let wm = (os / sg1).atan();
    let gm = mag(wm + wm);
    let s1 = gm * gm;
    let s2 = gs.mul_add(gs, -s1);
    let s5 = s9 + -1.0;
    let x = ((s9 * s2) / (s5 * (s5 * gm.mul_add(gm, s9)))).sqrt();
    let y = ((s9 * s9 * s2) / (s5 * (s5 * (gs.mul_add(gs, s9) * s1)))).sqrt();
    let nb0 = s9 + os.mul_add(os, sg1 * os / y);
    let nb1 = os.mul_add(os, -s9);
    let nb1 = nb1 + nb1;
    let nb2 = s9 + os.mul_add(os, (os * -sg1) / y);
    let s0 = os / x;
    let a0 = os.mul_add(os, s0) + 1.0;
    let s4 = os.mul_add(os, -1.0);
    let s0b = os.mul_add(os, -s0);
    bq(
        nb0 / a0,
        nb1 / a0,
        nb2 / a0,
        (s4 * -2.0) / a0,
        -(s0b + 1.0) / a0,
    )
}

/// Massberg analog-matched 1st-order low-pass (inlined in `Cchfilter`/`Cxover48`).
pub fn lp6(c: &Ctx, f: f32) -> Bq {
    let w0 = (c.pifs + c.pifs) * f;
    let d1 = TWO_PI / w0 as f64;
    let g1 = (2.0 / d1.mul_add(d1, 4.0).sqrt()) as f32;
    let g = if g1 > 0.5 { g1.sqrt() } else { SQRT1_2 };
    let s11 = fms(g, g, 1.0);
    let om = (w0 * s11.sqrt() / (g + g)).tan();
    let s1 = g.mul_add(g, -g1 * g1);
    let os = om * (s1 * s11).sqrt() / s11;
    let d = os + 1.0;
    bq((os + g1) / d, (os - g1) / d, 0.0, -(os + -1.0) / d, 0.0)
}

/// `Cxover48(type, hi, f)`: bus EQ cut shapes. `ty` 1..=10 = BW6, BW12, BS12, LR12, BW18, BW24, BS24, LR24,
/// BW48, LR48; `hi` = low-pass (high band), else high-pass (low band). Returns the sections and their count.
pub fn xover(c: &Ctx, ty: u8, hi: bool, f: f32) -> ([Bq; 4], usize) {
    let mut s = [IDENTITY; 4];
    const BW4: [f32; 2] = [f32::from_bits(0x3f0a8bd4), f32::from_bits(0x3fa73d75)];
    const LR8: [f32; 4] = [
        f32::from_bits(0x3fa73d74),
        f32::from_bits(0x3f0a8bd4),
        f32::from_bits(0x3fa73d74),
        f32::from_bits(0x3f0a8bd4),
    ];
    const BW8: [f32; 4] = [
        f32::from_bits(0x3f6664d7),
        f32::from_bits(0x402406cf),
        f32::from_bits(0x3f0281f7),
        f32::from_bits(0x3f19f1bd),
    ];
    let q2 = |f: f32, q: f32| {
        if hi {
            lp12(c, f, q)
        } else {
            hp12_bilinear(c, f, q)
        }
    };
    let one = |f: f32| if hi { lp6(c, f) } else { hp6(c, f) };
    let n = match ty {
        1 => {
            s[0] = one(f);
            1
        }
        2 => {
            s[0] = if hi { lp12(c, f, SQRT1_2) } else { hp12(c, f) };
            1
        }
        3 => {
            s[0] = q2(f, f32::from_bits(0x3f13cd3a));
            1
        }
        4 => {
            s[0] = one(f);
            s[1] = s[0];
            2
        }
        5 => {
            s[0] = one(f);
            s[1] = q2(f, 1.0);
            2
        }
        6 => {
            s[0] = q2(f, BW4[0]);
            s[1] = q2(f, BW4[1]);
            2
        }
        7 => {
            let d = f as f64;
            s[0] = q2(
                (d * f64_bits(0x3ff0f0eaa2ba6e6a)) as f32,
                f32::from_bits(0x3f4e37c2),
            );
            s[1] = q2(
                (d * f64_bits(0x3fee38eecdc259f1)) as f32,
                f32::from_bits(0x3f059d81),
            );
            2
        }
        8 => {
            s[0] = q2(f, SQRT1_2);
            s[1] = s[0];
            2
        }
        9 => {
            for (k, q) in BW8.iter().enumerate() {
                s[k] = q2(f, *q);
            }
            4
        }
        10 => {
            for (k, q) in LR8.iter().enumerate() {
                s[k] = q2(f, *q);
            }
            4
        }
        _ => 0,
    };
    (s, n)
}

/// Bus EQ tilt section: `hs12(pivot, -2 tilt, 0.2)` times `10^(tilt/20)` (positive tilt = more low end).
pub fn tilt(c: &Ctx, tilt_db: f32) -> Bq {
    let fp = tilt_pivot_hz(tilt_db);
    let mut s = hs12(c, fp, tilt_db * -2.0, f32::from_bits(0x3e4ccccd));
    let g = db2lin(tilt_db);
    s[0] *= g;
    s[1] *= g;
    s[2] *= g;
    s
}

/// Tilt pivot: 650 Hz at 0 dB, 780 Hz at |tilt| 3 dB, 925 Hz at 6 dB (piecewise linear in |tilt|).
pub fn tilt_pivot_hz(tilt_db: f32) -> f32 {
    let u = ((tilt_db.abs() + tilt_db.abs()) / 6.0).min(2.0);
    if u >= 1.0 {
        (u + -1.0).mul_add(925.0, (2.0 - u) * 780.0)
    } else {
        u.mul_add(780.0, (1.0 - u) * 650.0)
    }
}

/// Complex response of a section list at `f` Hz (double precision).
pub fn response(fs: f64, secs: &[Bq], f: f64) -> wing_dsp::Complex {
    let w = 2.0 * PI * f / fs;
    let z1 = wing_dsp::Complex::expj(-w);
    let z2 = z1 * z1;
    let one = wing_dsp::Complex::new(1.0, 0.0);
    let mut h = one;
    for s in secs {
        let num = one * s[0] as f64 + z1 * s[1] as f64 + z2 * s[2] as f64;
        let den = one + z1 * s[3] as f64 + z2 * s[4] as f64;
        h = h * (num / den);
    }
    h
}
