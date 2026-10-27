//! WING meter-word laws for model slots (T082): how a gate/dynamics model's gain reduction and an FX model's
//! state become the words of a meter report (protocol.md §4.2). Schema-free and allocation-free; the
//! simulator's meter sender resolves the law once per model id and applies it to the engine's telemetry.
//!
//! # Gate / dynamics (`gate gain` / `dyn gain` words, V1 and V2)
//!
//! WING-Edit 3.3.4 `nrp::MeterHandler::rxMeters` (0x1000d72d4) reads the gain word `w` of a channel /
//! aux / bus / main / matrix V2 block as a **meter position** `m = w / 256` (the GR meter's full deflection
//! is `m = 1`), except for the model at `ae::f_table` index 7 (**DEQ2**): there it takes the gain word and
//! the following LED/state word as two band gains `g_k = 1 + w_k / 256`, draws `20 log10(g1 g2) / 15`
//! (clamped to ±1) and keeps the two values for the dynamic-EQ graph. The console therefore applies the
//! model's GR-meter law before sending; the laws are WING-Edit's `ae::<kind>_gain_<model>` functions,
//! exported as `gr_meter` tables in `data/reference-curves/dyn-models.json` (reference-curves.md §4):
//!
//! | law | models |
//! |---|---|
//! | `GR / 60`, cap 1 | GATE, DUCK |
//! | `GR / 20`, cap 1 | 9000G, DS902, PSE, CMB, COMP, EXP, ECL33, 9000C, RED3, LA, F670, BLISS, 2250, E88C, LMT |
//! | `GR / 20`, cap 2.5 / 2 / 1.5 / 1.3 | E88 / B560 / D241C / NSTR |
//! | `(GR - 1) / 11` in 0..1 | D241G |
//! | `-GR / 20`, cap -1 (needle deflects the other way) | 76LA, L100 |
//! | `-GR / 15`, cap -1 | RIDE |
//! | `-0.3 g` (VU needle, `g` = linear gain) | B160, SBUS |
//! | `g / ln 10` | ONEC |
//! | constant 1 / constant 0 | WAVE / WARM |
//! | `band dB / 15` (signed, ±1) | DEQ: `gui_modules::DynEqGraph::updateCurve` draws the dynamic band at `m * 15` dB |
//! | two band words `256 (g_k - 1)` | DEQ2 (see above) |
//!
//! The DEQ entries of `dyn-models.json` (`(4/3) log10(1 + g)`) are the same law: WING-Edit's
//! `ae::dyn_gain_dyneq(_, g)` is `clamp(20 log10(1 + g) / 15, -1, 1)` with `1 + g` the band's linear gain
//! (T086). Its `dyn_led_dyneq` / `dyn_state_dyneq` are constant 0, so the DEQ LED/state word is 0.
//!
//! # FX (`A6` blocks: in L/R, out L/R, 6 state words)
//!
//! `rxMeters` stores words 0-3 as levels (1/256 dB) and words 4-9 verbatim in the FX unit; the FX pages read
//! them as follows (WING-Edit 3.3.4 static analysis, T082):
//!
//! | model | page | state words |
//! |---|---|---|
//! | *EVEN*, *SOUL*, *VINTAGE*, *BUS* | `Fx{Even,Soul,Vintage,Bus}Channel` | words 4, 6 = `2048 g` of the 1st / 2nd dynamics stage (LED ladder at 20 log10 g <= -3/-6/-10/-14/-19 dB) |
//! | DEQ3 | `DynEqGraph::updateCurve` | words 4, 6, 8 = `2048 (g_k - 1)` of band k (dynamic gain drawn, ±15 dB) |
//! | C5-CMB | `FxCombinator::Ccurves::paint` | words 4..8 = band k gain change `dB * 2048 / 6` (protocol.md "FX band GR") |
//! | LIMITER, *MASTER* | `FxPrecisionLimiter` / `FxMastering` | word 4 = GR; scale **[H]** `GR * 2048 / 6` (the pages scale it by 320/2048 px; the dB unit is not visible statically) |
//!
//! Raw state words ([`FxMeterLaw::Raw`], the model writes words 4..9 itself through
//! [`crate::model::Model::fx_meter_words`]; T087 static analysis, `lin2048` = linear amplitude/gain, 2048 = 1):
//!
//! | model | page | state words |
//! |---|---|---|
//! | PCORR | `FxPitchFix` | 4 = pitch ratio out/in (lin2048), 5 = `2048 (pitch class + 1)` of the target note (C = 1), 6 = detected period `64 * 48000 / f0` (WING-Edit assumes 48 kHz; 0 = unvoiced, hides the dial) |
//! | V-IMG | `FxVelvetImg` | 6 / 7 = output MID / SIDE peaks `2048 |L+R|`, `2048 |L-R|` (drawn 6 dB lower), 8 = correlation `1024 (1 + rho)` |
//! | SUB-M | `FxSubMonster` | 4..8 = band 1..5 level (lin2048, bars -72..0 dB): the synthesised sub of each band [I] |
//! | SPKMAN | `FxSpeakerPro` | 4 = dynamic-EQ gain `2048 (g - 1)` (with `dyneq` on), 6 = limiter input peak (lin2048), 8 = limiter gain (lin2048, 2048 idle) |
//! | DE-S2 | `FxDeEsser` | 4..7 = GR of LOW, HIGH, LOW (SIDE), HIGH (SIDE): LED brightness ladder on the raw word (10..1000); scale `GR * 2048 / 6` [H] |
//! | others | - | no state words read (P-BASS reads only the output levels) |

use std::f32::consts::LN_10;

/// Gain (dB, <= 0 for reduction) to linear.
#[inline]
fn lin(db: f32) -> f32 {
    10f32.powf(db * 0.05)
}

/// Most band values a model reports through [`crate::model::Model::meter_bands`].
pub const MAX_METER_BANDS: usize = 6;

/// GR-meter law of a gate / dynamics model (see the module doc).
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum DynMeterLaw {
    /// `clamp((GR - offset) / full_scale, 0, cap)`.
    Linear {
        full_scale: f32,
        cap: f32,
        offset: f32,
    },
    /// `-min(GR / full_scale, 1)`.
    Negative { full_scale: f32 },
    /// `-0.3 g`.
    Vu,
    /// `g / ln 10`.
    OneKnob,
    /// A constant position.
    Const(f32),
    /// `band 0 dB / 15`, signed.
    DynEq,
    /// Two band words `256 (g_k - 1)` (gain word + LED/state word).
    DynEq2,
}

const fn linear(full_scale: f32, cap: f32) -> DynMeterLaw {
    DynMeterLaw::Linear {
        full_scale,
        cap,
        offset: 0.0,
    }
}

impl DynMeterLaw {
    /// Law of gate/dynamics model `id` (`mdl` token). Unknown ids use the compressor law `GR / 20`.
    pub fn for_model(id: &str) -> Self {
        match id {
            "GATE" | "DUCK" => linear(60.0, 1.0),
            "E88" => linear(20.0, 2.5),
            "B560" => linear(20.0, 2.0),
            "D241C" => linear(20.0, 1.5),
            "NSTR" => linear(20.0, 1.3),
            "D241G" => DynMeterLaw::Linear {
                full_scale: 11.0,
                cap: 1.0,
                offset: 1.0,
            },
            "76LA" | "L100" => DynMeterLaw::Negative { full_scale: 20.0 },
            "RIDE" => DynMeterLaw::Negative { full_scale: 15.0 },
            "B160" | "SBUS" => DynMeterLaw::Vu,
            "ONEC" => DynMeterLaw::OneKnob,
            "WAVE" => DynMeterLaw::Const(1.0),
            "WARM" => DynMeterLaw::Const(0.0),
            "DEQ" => DynMeterLaw::DynEq,
            "DEQ2" => DynMeterLaw::DynEq2,
            _ => linear(20.0, 1.0),
        }
    }

    /// Meter position for `gr_db` dB of gain reduction (>= 0); `bands` = the model's signed band gains (dB,
    /// [`crate::model::Model::meter_bands`]) for the dynamic-EQ laws.
    pub fn position(self, gr_db: f32, bands: &[f32]) -> f32 {
        let gr = gr_db.max(0.0);
        match self {
            DynMeterLaw::Linear {
                full_scale,
                cap,
                offset,
            } => ((gr - offset) / full_scale).clamp(0.0, cap),
            DynMeterLaw::Negative { full_scale } => -(gr / full_scale).min(1.0),
            DynMeterLaw::Vu => -0.3 * lin(-gr),
            DynMeterLaw::OneKnob => lin(-gr) / LN_10,
            DynMeterLaw::Const(c) => c,
            DynMeterLaw::DynEq => (bands.first().copied().unwrap_or(0.0) / 15.0).clamp(-1.0, 1.0),
            DynMeterLaw::DynEq2 => {
                let b = |k: usize| bands.get(k).copied().unwrap_or(0.0);
                ((b(0) + b(1)) / 15.0).clamp(-1.0, 1.0)
            }
        }
    }

    /// The (gain, LED/state) words of a V2 block, or the gain word alone for V1 (`.0`). `active` is the
    /// LED/state flag for every law but DEQ (always 0) and DEQ2, whose second word carries band 2.
    pub fn words(self, gr_db: f32, bands: &[f32], active: bool) -> (i16, i16) {
        match self {
            DynMeterLaw::DynEq2 => {
                let w = |k: usize| sat(256.0 * (lin(bands.get(k).copied().unwrap_or(0.0)) - 1.0));
                (w(0), w(1))
            }
            // DEQ: `ae::dyn_led_dyneq` / `dyn_state_dyneq` are constant 0 (T086)
            DynMeterLaw::DynEq => (sat(256.0 * self.position(gr_db, bands)), 0),
            _ => (sat(256.0 * self.position(gr_db, bands)), active as i16),
        }
    }
}

/// State-word law of an FX model (see the module doc).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FxMeterLaw {
    /// No state words.
    None,
    /// Channel-strip FX: words 4, 6 = `2048 g` of dynamics stage 1 / 2.
    StripLeds,
    /// DEQ3: words 4, 6, 8 = `2048 (g_k - 1)`.
    TripleDeq,
    /// C5-CMB: words 4..8 = band gain change `dB * 2048 / 6`.
    Combinator,
    /// LIMITER / *MASTER*: word 4 = `GR * 2048 / 6` [H].
    Gr,
    /// The model's own words 4..9 ([`crate::model::Model::fx_meter_words`]), saturated.
    Raw,
}

impl FxMeterLaw {
    /// Law of FX model `id` (`mdl` token).
    pub fn for_model(id: &str) -> Self {
        match id {
            "*EVEN*" | "*SOUL*" | "*VINTAGE*" | "*BUS*" => FxMeterLaw::StripLeds,
            "DEQ3" => FxMeterLaw::TripleDeq,
            "C5-CMB" => FxMeterLaw::Combinator,
            "LIMITER" | "*MASTER*" => FxMeterLaw::Gr,
            "PCORR" | "V-IMG" | "SUB-M" | "SPKMAN" | "DE-S2" => FxMeterLaw::Raw,
            _ => FxMeterLaw::None,
        }
    }

    /// The 6 state words (FX block words 4..9) from the model's GR (dB, >= 0) and signed band gains (dB).
    pub fn words(self, gr_db: f32, bands: &[f32]) -> [i16; 6] {
        let b = |k: usize| bands.get(k).copied().unwrap_or(0.0);
        let mut w = [0i16; 6];
        match self {
            FxMeterLaw::None => {}
            FxMeterLaw::StripLeds => {
                w[0] = sat(2048.0 * lin(b(0).min(0.0)));
                w[2] = sat(2048.0 * lin(b(1).min(0.0)));
            }
            FxMeterLaw::TripleDeq => {
                for k in 0..3 {
                    w[2 * k] = sat(2048.0 * (lin(b(k)) - 1.0));
                }
            }
            FxMeterLaw::Combinator => {
                for (k, v) in w.iter_mut().take(5).enumerate() {
                    *v = sat(b(k) * 2048.0 / 6.0);
                }
            }
            FxMeterLaw::Gr => w[0] = sat(gr_db.max(0.0) * 2048.0 / 6.0),
            FxMeterLaw::Raw => {
                for (k, v) in w.iter_mut().enumerate() {
                    *v = sat(b(k));
                }
            }
        }
        w
    }
}

/// Peak hold for level-type FX state words: the previous value decays with a 100 ms time constant over the
/// `n` samples of a block (at `fs`), then takes the block's `peak`.
#[inline]
pub fn hold_peak(prev: f32, peak: f32, n: usize, fs: f64) -> f32 {
    let decay = (-(n as f64) / (0.1 * fs)).exp() as f32;
    (prev * decay).max(peak)
}

/// Round and saturate to an i16 word.
#[inline]
fn sat(v: f32) -> i16 {
    v.round().clamp(i16::MIN as f32, i16::MAX as f32) as i16
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn dyn_laws_match_wing_edit_points() {
        let p = |id: &str, gr: f32| DynMeterLaw::for_model(id).position(gr, &[]);
        assert_eq!(p("GATE", 30.0), 0.5);
        assert_eq!(p("COMP", 10.0), 0.5);
        assert_eq!(p("COMP", 40.0), 1.0);
        assert_eq!(p("E88", 80.0), 2.5);
        assert!((p("D241G", 6.0) - 0.45455).abs() < 1e-4);
        assert_eq!(p("76LA", 10.0), -0.5);
        assert!((p("RIDE", 6.0) + 0.4).abs() < 1e-6);
        assert!((p("B160", 6.0) + 0.15036).abs() < 1e-4);
        assert!((p("ONEC", 6.0) - 0.21766).abs() < 1e-4);
        assert_eq!(p("WAVE", 0.0), 1.0);
        assert_eq!(p("WARM", 20.0), 0.0);
        let deq = DynMeterLaw::for_model("DEQ");
        assert_eq!(deq.words(0.0, &[-7.5], true), (-128, 0));
        assert_eq!(deq.words(0.0, &[30.0], true), (256, 0));
        let deq2 = DynMeterLaw::for_model("DEQ2");
        let (a, b) = deq2.words(6.0, &[-6.0206, 0.0], true);
        assert_eq!((a, b), (-128, 0));
    }

    #[test]
    fn fx_laws() {
        let s = FxMeterLaw::for_model("*SOUL*").words(6.0, &[-6.0206, 0.0]);
        assert_eq!((s[0], s[2]), (1024, 2048));
        let d = FxMeterLaw::for_model("DEQ3").words(0.0, &[6.0206, 0.0, -6.0206]);
        assert_eq!((d[0], d[2], d[4]), (2048, 0, -1024));
        let c = FxMeterLaw::for_model("C5-CMB").words(3.0, &[-3.0, 0.0, 0.0, 0.0, -6.0]);
        assert_eq!((c[0], c[4]), (-1024, -2048));
        assert_eq!(FxMeterLaw::for_model("LIMITER").words(3.0, &[])[0], 1024);
        assert_eq!(FxMeterLaw::for_model("HALL").words(3.0, &[1.0]), [0; 6]);
        let r = FxMeterLaw::for_model("PCORR").words(0.0, &[2048.4, 4096.0, 40_000.0]);
        assert_eq!(r, [2048, 4096, 32767, 0, 0, 0]);
        assert_eq!(FxMeterLaw::for_model("DE-S2"), FxMeterLaw::Raw);
    }
}
