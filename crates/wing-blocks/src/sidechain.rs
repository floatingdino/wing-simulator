//! Dynamics/gate side-chain key filter (`dynsc`, `gatesc`) and the dynamics crossover `dynxo` (T03C), with
//! WING-Edit's exact coefficients (`ae::Cdynsidechain::update`, `ae::Ccompxover::update`), verified against
//! `data/reference-curves/dyn-sidechain-xover.json`.
//!
//! Key filter ([`key_filter_coeffs`], one biquad on the detector input):
//! - LP12 = `hqlp12(f, 0.707)` (Q ignored), HP12 = `hqhp12(f)` (Q ignored), BP = `hqbp(f, q)` (unity peak),
//!   NOTCH = `hqpeq(f, -30 dB, q)` (a 30 dB deep cut, not an infinite notch).
//!
//! Crossover ([`xover_coeffs`], one biquad `H`): `type` OFF / LO / HI / S6 / S12 / PEAK, `f`, `q` (PEAK only),
//! `depth` (-30..+30 dB, default 6):
//! - LO = `hqlp12(f, 0.707)`, HI = `hqhp12(f)` (depth unused).
//! - S6 / S12: depth >= 0: high shelf of `-depth` (`hqhs6` / `hqhs12`); depth < 0: low shelf of `depth`
//!   (`hqls6` / `hqls12`).
//! - PEAK: `hqpeq(f, -depth, q)`; for depth < 0 the whole section is scaled by `10^(depth/20)`.
//!
//! How the console applies `H` is not in WING-Edit (it only draws `|H|` and posts the biquad to the engine);
//! [`DynXover`] applies the dynamics model's change through it: `out = x + H * (model(x) - x)`. For a gain
//! model that is `x + (g - 1) * H x`: the full gain change where `|H| = 1`, none where `|H| = 0` (LO: only the
//! low band is compressed; S6 with +6 dB depth: the top is compressed 6 dB less). **[I]** inference.
//!
//! Real-time: coefficient updates are block-rate; processing is a stereo f64 TDF-II biquad, no allocation.

use crate::wingcoef::{self, Bq, Ctx};
use core::f32::consts::FRAC_1_SQRT_2;
use wing_dsp::{BiquadF64, Coeffs};

pub use crate::dynamics::KeyFilterKind;

/// Key filter biquad for `dynsc`/`gatesc` `type f q`.
pub fn key_filter_coeffs(c: &Ctx, kind: KeyFilterKind, f: f32, q: f32) -> Bq {
    match kind {
        KeyFilterKind::Off => wingcoef::IDENTITY,
        KeyFilterKind::Lp12 => wingcoef::lp12(c, f, FRAC_1_SQRT_2),
        KeyFilterKind::Hp12 => wingcoef::hp12(c, f),
        KeyFilterKind::Bp => wingcoef::bp(c, f, q),
        KeyFilterKind::Notch => wingcoef::peq(c, f, -30.0, q),
    }
}

fn coeffs(b: &Bq) -> Coeffs {
    Coeffs {
        b0: b[0] as f64,
        b1: b[1] as f64,
        b2: b[2] as f64,
        a1: b[3] as f64,
        a2: b[4] as f64,
    }
}

/// Stereo key filter with WING coefficients.
#[derive(Clone, Copy, Debug, Default)]
pub struct KeyFilter {
    on: bool,
    f: [BiquadF64; 2],
}

impl KeyFilter {
    /// Set type/frequency/Q at sample rate `fs` (state is kept; block-rate call).
    pub fn set(&mut self, kind: KeyFilterKind, freq: f32, q: f32, fs: f64) {
        self.on = kind != KeyFilterKind::Off;
        let c = coeffs(&key_filter_coeffs(&Ctx::new(fs), kind, freq, q));
        for b in &mut self.f {
            b.set_coeffs(c);
        }
    }

    pub fn is_on(&self) -> bool {
        self.on
    }

    /// Filter one sample of channel `ch` (0/1). Pass-through when off.
    #[inline]
    pub fn tick(&mut self, ch: usize, x: f32) -> f32 {
        if self.on {
            self.f[ch & 1].process(x)
        } else {
            x
        }
    }

    pub fn process_block(&mut self, ch: usize, buf: &mut [f32]) {
        if self.on {
            self.f[ch & 1].process_block(buf);
        }
    }

    pub fn reset(&mut self) {
        for b in &mut self.f {
            b.reset();
        }
    }

    pub fn flush_denormals(&mut self) {
        for b in &mut self.f {
            b.flush_denormals();
        }
    }
}

/// `dynxo.type`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum XoverKind {
    /// FLAT: full-band dynamics.
    #[default]
    Off,
    /// Low band only (12 dB/oct split).
    Lo,
    /// High band only (12 dB/oct split).
    Hi,
    /// 1st-order shelf weighting by `depth`.
    S6,
    /// 2nd-order shelf weighting by `depth`.
    S12,
    /// Peak weighting by `depth` at `f`/`q`.
    Peak,
}

impl XoverKind {
    /// Snapshot token (fw 3.x).
    pub fn from_snap(s: &str) -> Option<Self> {
        Some(match s {
            "OFF" => XoverKind::Off,
            "LO" => XoverKind::Lo,
            "HI" => XoverKind::Hi,
            "S6" => XoverKind::S6,
            "S12" => XoverKind::S12,
            "PEAK" => XoverKind::Peak,
            _ => return None,
        })
    }
}

/// `dynxo` parameters.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct XoverParams {
    pub kind: XoverKind,
    /// 20 .. 20000 Hz.
    pub freq: f32,
    /// PEAK Q.
    pub q: f32,
    /// -30 .. +30 dB.
    pub depth_db: f32,
}

impl Default for XoverParams {
    /// fw 3.x scene defaults: OFF, 1002.37 Hz, Q 0.4, depth 6 dB.
    fn default() -> Self {
        Self {
            kind: XoverKind::Off,
            freq: 1_002.374_4,
            q: 0.4,
            depth_db: 6.0,
        }
    }
}

impl XoverParams {
    pub fn clamped(mut self) -> Self {
        self.freq = self.freq.clamp(20.0, 20_000.0);
        self.q = self.q.clamp(0.4, 10.0);
        self.depth_db = self.depth_db.clamp(-30.0, 30.0);
        self
    }
}

/// The `dynxo` biquad (`Ccompxover::update`).
pub fn xover_coeffs(c: &Ctx, p: &XoverParams) -> Bq {
    let (f, d) = (p.freq, p.depth_db);
    match p.kind {
        XoverKind::Off => wingcoef::IDENTITY,
        XoverKind::Lo => wingcoef::lp12(c, f, FRAC_1_SQRT_2),
        XoverKind::Hi => wingcoef::hp12(c, f),
        XoverKind::S6 if d < 0.0 => wingcoef::hqls6(c, f, d),
        XoverKind::S6 => wingcoef::hqhs6(c, f, -d),
        XoverKind::S12 if d < 0.0 => wingcoef::hqls12(c, f, d),
        XoverKind::S12 => wingcoef::hqhs12(c, f, -d),
        XoverKind::Peak => {
            let mut s = wingcoef::peq(c, f, -d, p.q);
            if d < 0.0 {
                let g = wingcoef::db2lin(d);
                s[0] *= g;
                s[1] *= g;
                s[2] *= g;
            }
            s
        }
    }
}

/// Crossover solo (`dynxo/$solo`): the band the dynamics act on, `H` applied to the slot input (identity for
/// FLAT) [I: WING's XO listen is not documented].
pub fn xover_listen_coeffs(p: &XoverParams, fs: f64) -> Coeffs {
    coeffs(&xover_coeffs(&Ctx::new(fs), &p.clamped()))
}

/// Stereo dynamics crossover: routes a model's change through `H` (see module doc).
#[derive(Clone, Copy, Debug, Default)]
pub struct DynXover {
    on: bool,
    p: XoverParams,
    f: [BiquadF64; 2],
}

impl DynXover {
    pub fn set(&mut self, p: &XoverParams, fs: f64) {
        let p = p.clamped();
        let was = self.on;
        self.on = p.kind != XoverKind::Off;
        self.p = p;
        let c = coeffs(&xover_coeffs(&Ctx::new(fs), &p));
        for b in &mut self.f {
            b.set_coeffs(c);
            if !was {
                b.reset();
            }
        }
    }

    pub fn params(&self) -> &XoverParams {
        &self.p
    }

    pub fn is_on(&self) -> bool {
        self.on
    }

    /// One sample: `dry` input, `wet` model output -> crossover output.
    #[inline]
    pub fn tick(&mut self, ch: usize, dry: f32, wet: f32) -> f32 {
        if self.on {
            dry + self.f[ch & 1].process(wet - dry)
        } else {
            wet
        }
    }

    /// Block form: `wet[i] = dry[i] + H(wet[i] - dry[i])`.
    pub fn apply(&mut self, ch: usize, dry: &[f32], wet: &mut [f32]) {
        if !self.on {
            return;
        }
        for (w, &d) in wet.iter_mut().zip(dry) {
            *w = d + self.f[ch & 1].process(*w - d);
        }
    }

    pub fn reset(&mut self) {
        for b in &mut self.f {
            b.reset();
        }
    }

    pub fn flush_denormals(&mut self) {
        for b in &mut self.f {
            b.flush_denormals();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const FS: f64 = 48_000.0;

    fn db(b: &Bq, f: f64) -> f64 {
        wingcoef::response(FS, &[*b], f).db()
    }

    #[test]
    fn key_filter_shapes() {
        let c = Ctx::new(FS);
        let bp = key_filter_coeffs(&c, KeyFilterKind::Bp, 1000.0, 2.0);
        assert!(db(&bp, 1000.0).abs() < 0.01, "BP unity peak");
        assert!(db(&bp, 100.0) < -15.0);
        let n = key_filter_coeffs(&c, KeyFilterKind::Notch, 1000.0, 2.0);
        assert!((db(&n, 1000.0) + 30.0).abs() < 0.01, "notch = -30 dB PEQ");
        let lp = key_filter_coeffs(&c, KeyFilterKind::Lp12, 1000.0, 5.0);
        assert!((db(&lp, 1000.0) + 3.01).abs() < 0.05, "LP ignores Q");
        assert_eq!(
            key_filter_coeffs(&c, KeyFilterKind::Off, 1.0, 1.0),
            wingcoef::IDENTITY
        );
    }

    #[test]
    fn xover_depth_sign_and_shapes() {
        let c = Ctx::new(FS);
        let mk = |kind, depth_db| XoverParams {
            kind,
            freq: 1000.0,
            q: 1.0,
            depth_db,
        };
        let s = xover_coeffs(&c, &mk(XoverKind::S12, 6.0));
        assert!(db(&s, 30.0).abs() < 0.05 && (db(&s, 20_000.0) + 6.0).abs() < 0.3);
        let s = xover_coeffs(&c, &mk(XoverKind::S6, -6.0));
        assert!((db(&s, 20.0) + 6.0).abs() < 0.2 && db(&s, 20_000.0).abs() < 0.2);
        let s = xover_coeffs(&c, &mk(XoverKind::Peak, 6.0));
        assert!((db(&s, 1000.0) + 6.0).abs() < 0.01 && db(&s, 20.0).abs() < 0.05);
        let s = xover_coeffs(&c, &mk(XoverKind::Peak, -6.0));
        assert!(db(&s, 1000.0).abs() < 0.01 && (db(&s, 20.0) + 6.0).abs() < 0.05);
    }

    #[test]
    fn dynxo_routes_gain_change_through_h() {
        // A fixed -12 dB "model" with LO crossover at 1 kHz: lows get -12 dB, highs stay at 0 dB.
        let mut xo = DynXover::default();
        xo.set(
            &XoverParams {
                kind: XoverKind::Lo,
                freq: 1000.0,
                ..Default::default()
            },
            FS,
        );
        let g = 10f32.powf(-12.0 / 20.0);
        let run = |xo: &mut DynXover, f: f64| {
            xo.reset();
            let n = 48_000;
            let x: Vec<f32> = (0..n)
                .map(|i| (2.0 * core::f64::consts::PI * f * i as f64 / FS).sin() as f32)
                .collect();
            let mut y: Vec<f32> = x.iter().map(|v| v * g).collect();
            xo.apply(0, &x, &mut y);
            let p = y[n / 2..].iter().fold(0.0f32, |m, v| m.max(v.abs()));
            20.0 * (p as f64).log10()
        };
        // Small residual from the split's phase (H is not linear-phase): within 0.5 dB well below f.
        assert!((run(&mut xo, 30.0) + 12.0).abs() < 0.5);
        assert!(run(&mut xo, 15_000.0).abs() < 0.2);
    }
}
