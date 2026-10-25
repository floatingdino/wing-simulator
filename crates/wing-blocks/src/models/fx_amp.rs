//! FX rack amp / cab simulators (T05A, models.md §6): `RACKAMP` (RACK AMP, SansAmp PSA-1 style), `UKROCK`
//! (UK ROCK AMP, Marshall JCM800 2203), `ANGEL` (ANGEL AMP, ENGL high gain), `JAZZC` (JAZZ CLEAN AMP, Roland
//! JC-120), `DELUXE` (DELUXE AMP, Fender Deluxe Reverb AB763) and `BODY` (BODYREZ, TC BodyRez acoustic piezo
//! body-resonance restorer).
//!
//! Own grey-box models (no third-party code, no captured IRs or neural weights; see docs/research/dsp-reuse.md).
//!
//! ## Amp signal flow (per channel; WING FX slots are stereo, both sides run the same amp independently)
//!
//! ```text
//! base rate: sanitize -> pre-EQ (RACKAMP buzz/punch/crunch, ANGEL mid boost)
//! 4x       : [stage 1] -> ... -> [stage n] -> power coupling HP -> presence / depth shelves -> 2-pole
//!            band limit -> master -> power amp (push-pull, rail sag) -> align delay
//!            where at stage position `ts_pos` the tone stack and at `gain_pos` the gain/volume pot (+ bright
//!            cap shelf) are inserted
//! base rate: post-EQ (RACKAMP low/high EQ) -> cab sim (cross-faded on/off) -> out gain -> DC blocker
//! ```
//!
//! * **Gain stages**: coupling-cap high-pass -> gain -> asymmetric cubic soft clipper `f(u) = L c(u/L)`,
//!   `c(v) = v - 4v^3/27` up to the knee `|v| = 3/2`, flat beyond; `L = lp` for `u >= 0` (grid-conduction
//!   side, low limit) and `L = ln` for `u < 0` (cut-off side, more headroom) -> 2-pole Miller/plate-cap
//!   low-pass (7..15 kHz; 2-pole so the next stage's ADAA sees little energy near the 4x Nyquist).
//!   The asymmetry gives the even harmonics of a triode; the solid-state JAZZC stages are symmetric. Every
//!   curve (preamp and power) runs with **second-order antiderivative anti-aliasing** (ADAA2, Bilbao,
//!   Esqueda, Parker & Valimaki 2017) using the curve's polynomial antiderivatives, with the *linear*
//!   part passed untouched (only the nonlinear remainder `f(u) - u` is ADAA-filtered: small signals get no
//!   ADAA droop). ADAA2 suppresses an alias that lands `d` Hz from a multiple of the 4x rate by
//!   `~(d / 192 kHz)^2` on top of the oversampling (first-order ADAA only gives `d / 192 kHz`: measured
//!   -62 dBc at 4 kHz high gain, not enough). Measured worst alias in 0..20 kHz, 4 kHz at -6 dBFS, every
//!   amp at full gain (test `aliasing_below_minus_80_dbc_at_4khz_high_gain`): -84 dBc with CAB SIM off
//!   (UKROCK), <= -96 dBc with the cab on.
//! * **4x oversampling** (`wing_dsp::oversample::Oversampler4x`, linear-phase half-band FIRs, 45.5 samples
//!   round trip) around everything nonlinear plus the tone stack, presence and power stage. Each ADAA2
//!   curve delays by exactly one 4x sample; a pure delay tops the oversampled section up to [`OS_DELAY`] =
//!   6 samples (1.5 base samples), so the latency is an integer: **[`AMP_LATENCY`] = 47 samples** (0.98 ms
//!   at 48 kHz) for every amp model; BODY has 0.
//! * **Tone stacks**: exact component-value third-order transfer function of the passive
//!   treble/middle/bass network (the Fender/Marshall "FMV" topology analysed by Yeh & Smith, DAFx-06),
//!   derived here by nodal analysis with the middle control as a rheostat `m R3` to ground (which is how
//!   the Bassman/JCM800/AB763 wire it), bilinear-transformed at the 4x rate (warping < 1 % below 10 kHz).
//!   Component sets: [`ToneStackParts`]. Bass pot audio taper, treble/middle linear. DELUXE (no middle
//!   knob) has the AB763 fixed 6.8 k resistor in the middle position (`m = 1`).
//! * **Power amp**: push-pull = symmetric curve. **Sag**: a feed-forward detector (anti-aliased clipped
//!   rectifier `min(|u|, 1)` of the power-stage drive, i.e. the current draw; 10 ms attack,
//!   voice-dependent release) drops the supply rail `V = 1 / (1 + sag_k * env)`; the stage then clips at
//!   `V`: `y = V f(u / V)` -> compression and "bloom" on note decay. (A plain `|x|` detector at the 4x
//!   rate folds rectifier harmonics to near DC and modulates them onto every harmonic: -60 dBc.)
//!   PRESENCE = first-order high shelf (negative-feedback presence network), ANGEL BOTTOM = depth low
//!   shelf, both ahead of the power stage.
//! * **CPU** (Apple M-series, criterion `fx_amp` bench, 64-sample stereo block): ANGEL ~18 us, UKROCK
//!   ~16 us, DELUXE ~13 us, RACKAMP ~12 us, JAZZC ~10 us (oversampler ~3 us of each), BODY ~0.4 us. The
//!   4x section runs stage by stage over the chunk with both channels interleaved ([`os_section`]).
//! * **Cab sim**: IR-free, 6-section biquad cascades per cabinet (closed 4x12 Greenback, 4x12 V30, open
//!   1x12, open 2x12 JC, SansAmp speaker emulation) at the base rate: low resonance, cone break-up peaks
//!   and steep top roll-off. CAB SIM off/on cross-fades over 10 ms.
//!
//! ## Parameters (every WING key mapped; knob values 0..10, DELUXE 1..10)
//!
//! Items marked [I] are inferred (no console measurement or schematic-exact source yet).
//!
//! * RACKAMP: PREAMP = drive into the first stage (-12..+24 dB), BUZZ = pre-distortion low shelf 180 Hz
//!   +/-12 dB, PUNCH = pre-distortion peak 400 Hz +/-10 dB, CRUNCH = pre-distortion peak 2.8 kHz +/-12 dB,
//!   DRIVE = power-section drive (-12..+24 dB), LOW EQ = post shelf 120 Hz +/-12 dB, HIGH EQ = post shelf
//!   3.3 kHz +/-12 dB [I].
//! * UKROCK / ANGEL: GAIN = log pot between V1a and V1b (with the JCM800's 470 pF bright cap: treble bleed
//!   falling with gain; ANGEL: only with BRIGHT on) [I], BASS/MIDDLE/TREBLE = tone stack, PRESENCE =
//!   -3..+9 dB shelf at 3.5 kHz [I], MASTER = log pot into the power amp, SAG = rail droop depth. ANGEL:
//!   four preamp stages, MID BOOST = +7 dB at 650 Hz before the preamp [I], BOTTOM = +6 dB depth shelf at
//!   110 Hz [I].
//! * JAZZC: VOLUME before the (high-headroom, symmetric) preamp, BRIGHT = +6 dB shelf at 3 kHz [I].
//! * DELUXE: VOLUME after the tone stack (AB763 order: V1a -> tone stack -> volume -> V1b), no master: the
//!   power amp breaks up with VOLUME; GZ34 tube-rectifier sag.
//! * OUT GAIN (all): `(out / default)^2` linear (default = unity, 0 = mute) on top of a per-model make-up
//!   gain that puts the defaults near the input level [I].
//! * BODY: BODY 0..100 = amount of a fixed body-resonance EQ (air cavity 95 Hz, top plate 210 Hz, piezo
//!   "quack" cut 1.3 kHz, string-zing shelf 7 kHz); 0 = exact passthrough [I].

use crate::model::{Model, ParamValue};
use core::f64::consts::PI;
use wing_dsp::biquad::{Biquad, BiquadF64, Coeffs};
use wing_dsp::oversample::Oversampler4x;
use wing_dsp::smooth::{one_pole_coeff, OnePoleSmoother};

/// Oversampling factor of the nonlinear section.
pub const OS: usize = 4;
/// Total delay (4x-rate samples) inside the oversampled section: one per ADAA2 curve (preamp stages +
/// power amp) plus a pure alignment delay making up the rest. 6 samples = 1.5 base samples, which turns the
/// 45.5-sample oversampler round trip into an integer.
pub const OS_DELAY: usize = 6;
/// Latency of every amp model (base-rate samples): 45.5 + 6/4.
pub const AMP_LATENCY: usize = 47;
/// Internal processing chunk (base-rate samples); parameters are smoothed / designed per chunk.
const CHUNK: usize = 64;
/// Parameter smoothing time constant.
const SMOOTH_S: f64 = 0.02;
/// ADAA2 ill-conditioning threshold, relative to |u| (f64).
const ADAA_EPS: f64 = 2e-5;
/// Input sanitising limit (|x| in FS); non-finite input samples become 0.
const IN_LIMIT: f32 = 16.0;
/// Output DC blocker corner.
const DC_HZ: f64 = 8.0;
/// CAB SIM on/off cross-fade time constant.
const CAB_XFADE_S: f64 = 0.01;
/// Sag envelope attack.
const SAG_ATTACK_S: f64 = 0.01;
// every curve (MAX_STAGES preamp + power) fits in the oversampled-section delay
const _: () = assert!(MAX_STAGES < OS_DELAY);
/// Q of the 2-pole inter-stage (Miller / plate cap) low-passes.
const LP_Q: f64 = 0.6;
/// Max preamp stages.
const MAX_STAGES: usize = 4;
/// Threshold below which filter states are flushed (denormal guard, per chunk).
const FLUSH: f64 = 1e-30;

// ---------------------------------------------------------------------------------------------------------
// Building blocks
// ---------------------------------------------------------------------------------------------------------

#[inline]
fn db(x: f64) -> f64 {
    10f64.powf(x / 20.0)
}

/// Audio (log) pot taper: 0 -> 0, 1 -> 1, `(e^(k p) - 1) / (e^k - 1)`.
#[inline]
pub fn audio_taper(p: f64, k: f64) -> f64 {
    let p = p.clamp(0.0, 1.0);
    (k * p).exp_m1() / k.exp_m1()
}

/// First-order section `H(s) = g0 (1 + s/wz) / (1 + s/wp)` (or LP/HP), bilinear with per-corner prewarp,
/// f64 TDF-II.
#[derive(Clone, Copy, Debug)]
pub struct Fo {
    b0: f64,
    b1: f64,
    a1: f64,
    z: f64,
}

impl Default for Fo {
    fn default() -> Self {
        Self::IDENTITY
    }
}

impl Fo {
    pub const IDENTITY: Self = Self {
        b0: 1.0,
        b1: 0.0,
        a1: 0.0,
        z: 0.0,
    };

    #[inline]
    fn k(fs: f64, f: f64) -> f64 {
        (PI * f.clamp(1e-3, 0.49 * fs) / fs).tan()
    }

    pub fn lowpass(fs: f64, fc: f64) -> Self {
        let k = Self::k(fs, fc);
        let n = 1.0 / (1.0 + k);
        Self {
            b0: k * n,
            b1: k * n,
            a1: (k - 1.0) * n,
            z: 0.0,
        }
    }

    pub fn highpass(fs: f64, fc: f64) -> Self {
        let k = Self::k(fs, fc);
        let n = 1.0 / (1.0 + k);
        Self {
            b0: n,
            b1: -n,
            a1: (k - 1.0) * n,
            z: 0.0,
        }
    }

    /// `g0 (1 + s/wz) / (1 + s/wp)`.
    pub fn zero_pole(fs: f64, g0: f64, fz: f64, fp: f64) -> Self {
        let iz = 1.0 / Self::k(fs, fz);
        let ip = 1.0 / Self::k(fs, fp);
        let a0 = 1.0 + ip;
        Self {
            b0: g0 * (1.0 + iz) / a0,
            b1: g0 * (1.0 - iz) / a0,
            a1: (1.0 - ip) / a0,
            z: 0.0,
        }
    }

    /// High shelf: unity at DC, `gain_db` at HF, geometric-mean corner `fc`.
    pub fn high_shelf(fs: f64, fc: f64, gain_db: f64) -> Self {
        let r = db(gain_db).sqrt();
        Self::zero_pole(fs, 1.0, fc / r, fc * r)
    }

    /// Low shelf: `gain_db` at DC, unity at HF.
    pub fn low_shelf(fs: f64, fc: f64, gain_db: f64) -> Self {
        let g = db(gain_db);
        let r = g.sqrt();
        Self::zero_pole(fs, g, fc * r, fc / r)
    }

    /// Copy coefficients, keep state.
    #[inline]
    pub fn set(&mut self, o: Fo) {
        self.b0 = o.b0;
        self.b1 = o.b1;
        self.a1 = o.a1;
    }

    #[inline(always)]
    pub fn tick(&mut self, x: f64) -> f64 {
        let y = self.b0 * x + self.z;
        self.z = self.b1 * x - self.a1 * y;
        y
    }

    pub fn reset(&mut self) {
        self.z = 0.0;
    }

    #[inline]
    fn flush(&mut self) {
        if self.z.abs() < FLUSH {
            self.z = 0.0;
        }
    }

    /// Magnitude response (dB) at `f` for sample rate `fs` (tests / docs).
    pub fn magnitude_db(&self, fs: f64, f: f64) -> f64 {
        let w = 2.0 * PI * f / fs;
        let (c, s) = (w.cos(), -w.sin());
        let nr = self.b0 + self.b1 * c;
        let ni = self.b1 * s;
        let dr = 1.0 + self.a1 * c;
        let di = self.a1 * s;
        10.0 * ((nr * nr + ni * ni) / (dr * dr + di * di)).log10()
    }
}

/// Knee end of the normalised cubic clipper `c(v) = v - (4/27) v^3` (`c(3/2) = 1`, `c'(3/2) = 0`).
const KNEE: f64 = 1.5;

/// Asymmetric cubic soft clipper `f(u) = L c(u / L)` with `c(v) = v - (4/27) v^3` for `|v| <= 3/2` and
/// `+/-1` beyond; `L = lp` for `u >= 0` (output limit `+lp`), `L = ln` for `u < 0` (limit `-ln`). Unity
/// slope at 0, C1-continuous, and polynomial first/second antiderivatives (cheap, exact ADAA2).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Curve {
    pub lp: f64,
    pub ln: f64,
    ip: f64,
    ineg: f64,
}

impl Curve {
    pub const fn new(lp: f64, ln: f64) -> Self {
        Self {
            lp,
            ln,
            ip: 1.0 / lp,
            ineg: 1.0 / ln,
        }
    }

    pub const fn sym(l: f64) -> Self {
        Self::new(l, l)
    }

    /// `(L, 1/L)` for the side of `u`.
    #[inline(always)]
    fn side(self, u: f64) -> (f64, f64) {
        if u >= 0.0 {
            (self.lp, self.ip)
        } else {
            (self.ln, self.ineg)
        }
    }

    #[inline(always)]
    pub fn f(self, u: f64) -> f64 {
        let (l, il) = self.side(u);
        let v = (u * il).clamp(-KNEE, KNEE);
        l * (v - (4.0 / 27.0) * v * v * v)
    }

    /// First antiderivative (`F1(0) = 0`): `L^2 C1(u/L)`, `C1 = v^2/2 - v^4/27` | `|v| - 9/16`.
    #[inline(always)]
    pub fn f1(self, u: f64) -> f64 {
        let (l, il) = self.side(u);
        let v = u * il;
        let a = v.abs();
        let c = if a <= KNEE {
            let v2 = v * v;
            v2 * (0.5 - v2 * (1.0 / 27.0))
        } else {
            a - 9.0 / 16.0
        };
        l * l * c
    }

    /// Second antiderivative (`F2(0) = 0`): `L^3 C2(u/L)`, `C2 = v^3/6 - v^5/135` |
    /// `sign(v) (v^2/2 - 9|v|/16 + 9/40)`.
    #[inline(always)]
    pub fn f2(self, u: f64) -> f64 {
        let (l, il) = self.side(u);
        let v = u * il;
        let a = v.abs();
        let c = if a <= KNEE {
            let v2 = v * v;
            v * v2 * (1.0 / 6.0 - v2 * (1.0 / 135.0))
        } else {
            (0.5 * v * v - (9.0 / 16.0) * a + 0.225).copysign(v)
        };
        l * l * l * c
    }
}

/// First-order ADAA of the clipped rectifier `h(x) = min(|x|, 1)` (`H(x) = x |x| / 2` inside, `sign(x)
/// (|x| - 1/2)` beyond): the sag detector. A plain `|x|` would fold its high harmonics to near DC, where
/// the envelope passes them on as gain modulation (sidebands of every harmonic).
#[derive(Clone, Copy, Debug, Default)]
pub struct RectAdaa {
    x1: f64,
    h1: f64,
}

impl RectAdaa {
    #[inline(always)]
    fn h(x: f64) -> f64 {
        let a = x.abs();
        if a <= 1.0 {
            0.5 * x * a
        } else {
            (a - 0.5).copysign(x)
        }
    }

    #[inline(always)]
    pub fn tick(&mut self, x: f64) -> f64 {
        let h = Self::h(x);
        let dx = x - self.x1;
        let y = if dx.abs() > 1e-9 {
            (h - self.h1) / dx
        } else {
            (0.5 * (x + self.x1)).abs().min(1.0)
        };
        self.x1 = x;
        self.h1 = h;
        y
    }
}

/// Second-order ADAA (Bilbao, Esqueda, Parker, Valimaki 2017) of one [`Curve`], with the *linear* part
/// passed untouched: `y_n = u_(n-1) + ADAA2[f - id]`, i.e. ADAA2 of `f` plus `(2 u_(n-1) - u_n -
/// u_(n-2)) / 3` (ADAA2 of the identity is the 3-tap mean). Delay: exactly one sample for every signal.
/// The common path costs one division: `d0 = dF2 dx2 r`, `y = 2 (dF2 - d1 dx) r`, `r = 1 / (dx dx2)`.
#[derive(Clone, Copy, Debug, Default)]
pub struct Adaa2 {
    x1: f64,
    x2: f64,
    f2_1: f64,
    d1: f64,
}

impl Adaa2 {
    #[inline(always)]
    pub fn tick(&mut self, c: Curve, x0: f64) -> f64 {
        // ill-conditioning threshold: rounding error of the F2 differences ~ eps |F2| / dx^2
        let tol = ADAA_EPS * x0.abs() + 1e-12;
        let f2_0 = c.f2(x0);
        let dx = x0 - self.x1;
        let dx2 = x0 - self.x2;
        let df = f2_0 - self.f2_1;
        let (d0, y) = if dx.abs() > tol && dx2.abs() > tol {
            let r = 1.0 / (dx * dx2);
            (df * dx2 * r, 2.0 * (df - self.d1 * dx) * r)
        } else {
            self.slow(c, x0, dx, dx2, df, tol)
        };
        let lin = (2.0 * self.x1 - x0 - self.x2) * (1.0 / 3.0);
        self.x2 = self.x1;
        self.x1 = x0;
        self.f2_1 = f2_0;
        self.d1 = d0;
        y + lin
    }

    /// Ill-conditioned cases (published fallbacks).
    #[cold]
    #[inline(never)]
    fn slow(&self, c: Curve, x0: f64, dx: f64, dx2: f64, df: f64, tol: f64) -> (f64, f64) {
        let d0 = if dx.abs() > tol {
            df / dx
        } else {
            c.f1(0.5 * (x0 + self.x1))
        };
        let y = if dx2.abs() > tol {
            2.0 * (d0 - self.d1) / dx2
        } else {
            let xb = 0.5 * (x0 + self.x2);
            let delta = xb - self.x1;
            if delta.abs() > tol {
                2.0 / delta * (c.f1(xb) + (c.f2(self.x1) - c.f2(xb)) / delta)
            } else {
                c.f(0.5 * (xb + self.x1))
            }
        };
        (d0, y)
    }

    pub fn reset(&mut self) {
        *self = Self::default();
    }
}

// ---------------------------------------------------------------------------------------------------------
// Tone stack
// ---------------------------------------------------------------------------------------------------------

/// Component values of the passive TMB ("FMV") tone stack. Topology (Yeh & Smith naming): input -> `C1` ->
/// top of the treble pot `R1` (wiper = output, bottom = node B); input -> slope resistor `R4` -> node A;
/// `C2` A-B; bass pot `R2` (rheostat, `l R2`) B-M; `C3` A-M; middle pot `R3` (rheostat, `m R3`) M-ground.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ToneStackParts {
    pub r1: f64,
    pub r2: f64,
    pub r3: f64,
    pub r4: f64,
    pub c1: f64,
    pub c2: f64,
    pub c3: f64,
}

impl ToneStackParts {
    /// Fender '59 Bassman 5F6-A (reference set of Yeh & Smith).
    pub const BASSMAN: Self = Self {
        r1: 250e3,
        r2: 1e6,
        r3: 25e3,
        r4: 56e3,
        c1: 250e-12,
        c2: 20e-9,
        c3: 20e-9,
    };
    /// Marshall JCM800 2203.
    pub const JCM800: Self = Self {
        r1: 220e3,
        r2: 1e6,
        r3: 22e3,
        r4: 33e3,
        c1: 470e-12,
        c2: 22e-9,
        c3: 22e-9,
    };
    /// ENGL-style high-gain stack [I].
    pub const ANGEL: Self = Self {
        r1: 250e3,
        r2: 1e6,
        r3: 25e3,
        r4: 47e3,
        c1: 330e-12,
        c2: 22e-9,
        c3: 22e-9,
    };
    /// Roland JC-120-style stack [I].
    pub const JC120: Self = Self {
        r1: 250e3,
        r2: 1e6,
        r3: 10e3,
        r4: 47e3,
        c1: 180e-12,
        c2: 22e-9,
        c3: 47e-9,
    };
    /// Fender AB763 Deluxe Reverb (vibrato channel): middle = fixed 6.8 k (use `m = 1`).
    pub const DELUXE: Self = Self {
        r1: 250e3,
        r2: 250e3,
        r3: 6.8e3,
        r4: 100e3,
        c1: 250e-12,
        c2: 100e-9,
        c3: 47e-9,
    };

    /// Analog transfer function `H(s) = (b1 s + b2 s^2 + b3 s^3) / (1 + a1 s + a2 s^2 + a3 s^3)` for treble
    /// `t`, middle `m`, bass `l` pot positions in 0..1 (returned as `[b0..b3]`, `[a0..a3]`, `b0 = 0`,
    /// `a0 = 1`). Exact nodal-analysis result for an unloaded output.
    pub fn analog(&self, t: f64, m: f64, l: f64) -> ([f64; 4], [f64; 4]) {
        let Self {
            r1,
            r2,
            r3,
            r4,
            c1,
            c2,
            c3,
        } = *self;
        let (t, m, l) = (t.clamp(0.0, 1.0), m.clamp(0.0, 1.0), l.clamp(0.0, 1.0));
        let b1 = t * c1 * r1 + l * (c1 + c2) * r2 + m * (c1 + c2 + c3) * r3;
        let mid2 = m
            * (c1 * c2 * r1 * r3
                + c1 * c2 * r3 * r4
                + c1 * c3 * r1 * r3
                + c1 * c3 * r3 * r4
                + l * (c1 + c2) * c3 * r2 * r3);
        let b2 = l * (c1 * c2 * r1 * r2 + c1 * c2 * r2 * r4 + c1 * c3 * r2 * r4)
            + mid2
            + t * (c1 * c2 * r1 * r4 + c1 * c3 * r1 * r4);
        let ccc = c1 * c2 * c3;
        let b3 = ccc * r1 * r2 * r4 * l * t + m * l * ccc * (r1 * r2 * r3 + r2 * r3 * r4);
        let a1 = c1 * r1 + (c2 + c3) * r4 + l * (c1 + c2) * r2 + m * (c1 + c2 + c3) * r3;
        let a2 = (c1 * c2 + c1 * c3) * r1 * r4
            + l * (c1 * c2 * r1 * r2 + c1 * c2 * r2 * r4 + c1 * c3 * r2 * r4 + c2 * c3 * r2 * r4)
            + mid2;
        let a3 = ccc * r1 * r2 * r4 * l + m * l * ccc * (r1 * r2 * r3 + r2 * r3 * r4);
        ([0.0, b1, b2, b3], [1.0, a1, a2, a3])
    }

    /// Digital coefficients (bilinear at `fs`, `a0` normalised to 1).
    pub fn digital(&self, fs: f64, t: f64, m: f64, l: f64) -> ([f64; 4], [f64; 4]) {
        let (b, a) = self.analog(t, m, l);
        let c = 2.0 * fs;
        let bl = |p: [f64; 4]| {
            let (p0, p1, p2, p3) = (p[0], p[1] * c, p[2] * c * c, p[3] * c * c * c);
            [
                p0 + p1 + p2 + p3,
                3.0 * p0 + p1 - p2 - 3.0 * p3,
                3.0 * p0 - p1 - p2 + 3.0 * p3,
                p0 - p1 + p2 - p3,
            ]
        };
        let bd = bl(b);
        let ad = bl(a);
        let n = 1.0 / ad[0];
        (
            [bd[0] * n, bd[1] * n, bd[2] * n, bd[3] * n],
            [1.0, ad[1] * n, ad[2] * n, ad[3] * n],
        )
    }
}

/// Third-order TDF-II filter (f64) running a [`ToneStackParts`] response.
#[derive(Clone, Copy, Debug, Default)]
pub struct ToneStack {
    b: [f64; 4],
    a: [f64; 4],
    z: [f64; 3],
}

impl ToneStack {
    pub fn set(&mut self, coeffs: ([f64; 4], [f64; 4])) {
        self.b = coeffs.0;
        self.a = coeffs.1;
    }

    pub fn reset(&mut self) {
        self.z = [0.0; 3];
    }

    #[inline(always)]
    pub fn tick(&mut self, x: f64) -> f64 {
        let y = self.b[0] * x + self.z[0];
        self.z[0] = self.b[1] * x - self.a[1] * y + self.z[1];
        self.z[1] = self.b[2] * x - self.a[2] * y + self.z[2];
        self.z[2] = self.b[3] * x - self.a[3] * y;
        y
    }

    fn flush(&mut self) {
        for z in &mut self.z {
            if z.abs() < FLUSH {
                *z = 0.0;
            }
        }
    }
}

// ---------------------------------------------------------------------------------------------------------
// Cabinets
// ---------------------------------------------------------------------------------------------------------

/// Cabinet sims (IR-free biquad cascades) [I].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Cab {
    /// Closed-back 4x12, Greenback-style (UKROCK).
    Greenback412,
    /// Closed-back 4x12, V30-style: tighter low end, more upper-mid bite (ANGEL).
    V30412,
    /// Open-back 1x12 (DELUXE).
    Open112,
    /// Open-back 2x12 JC (JAZZC).
    Jc212,
    /// SansAmp-style speaker emulation (RACKAMP).
    SansAmp,
}

#[derive(Clone, Copy, Debug)]
enum Sec {
    Hp(f64, f64),
    Lp(f64, f64),
    Pk(f64, f64, f64),
}

/// Number of biquads per cab.
pub const CAB_SECTIONS: usize = 6;

impl Cab {
    fn sections(self) -> [Sec; CAB_SECTIONS] {
        use Sec::*;
        match self {
            Cab::Greenback412 => [
                Hp(75.0, 0.9),
                Pk(110.0, 1.4, 4.5),
                Pk(450.0, 1.0, -3.0),
                Pk(2300.0, 1.3, 4.0),
                Lp(4800.0, 0.85),
                Lp(6500.0, 0.6),
            ],
            Cab::V30412 => [
                Hp(85.0, 1.0),
                Pk(120.0, 1.5, 3.5),
                Pk(500.0, 1.0, -4.0),
                Pk(3000.0, 1.2, 5.0),
                Lp(5200.0, 0.9),
                Lp(7000.0, 0.6),
            ],
            Cab::Open112 => [
                Hp(90.0, 0.6),
                Pk(140.0, 1.0, 1.5),
                Pk(800.0, 1.0, -2.0),
                Pk(2000.0, 1.0, 3.0),
                Lp(5500.0, 0.8),
                Lp(8000.0, 0.6),
            ],
            Cab::Jc212 => [
                Hp(65.0, 0.7),
                Pk(120.0, 1.2, 2.0),
                Pk(1000.0, 0.8, -1.5),
                Pk(3200.0, 1.0, 2.5),
                Lp(5500.0, 0.8),
                Lp(8000.0, 0.6),
            ],
            Cab::SansAmp => [
                Hp(70.0, 0.8),
                Pk(160.0, 1.0, 2.0),
                Pk(700.0, 0.9, -2.0),
                Pk(2000.0, 1.0, 3.0),
                Lp(4500.0, 0.75),
                Lp(5500.0, 0.6),
            ],
        }
    }

    /// Biquad coefficients at `fs`.
    pub fn coeffs(self, fs: f64) -> [Coeffs; CAB_SECTIONS] {
        self.sections().map(|s| match s {
            Sec::Hp(f, q) => Coeffs::highpass(fs, f, q),
            Sec::Lp(f, q) => Coeffs::lowpass(fs, f, q),
            Sec::Pk(f, q, g) => Coeffs::peak(fs, f, q, g),
        })
    }
}

// ---------------------------------------------------------------------------------------------------------
// Amp voices
// ---------------------------------------------------------------------------------------------------------

/// Which amp.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AmpKind {
    RackAmp,
    UkRock,
    Angel,
    JazzClean,
    Deluxe,
}

/// One preamp gain stage.
#[derive(Clone, Copy, Debug)]
struct StageCfg {
    gain: f64,
    curve: Curve,
    hp_hz: f64,
    lp_hz: f64,
}

const fn st(gain: f64, pos: f64, neg: f64, hp_hz: f64, lp_hz: f64) -> StageCfg {
    StageCfg {
        gain,
        curve: Curve::new(pos, neg),
        hp_hz,
        lp_hz,
    }
}

const NO_STAGE: StageCfg = st(1.0, 1.0, 1.0, 1.0, 20_000.0);
/// "Not present" position.
const NONE: usize = usize::MAX;

/// Constant voicing of an amp model [I].
#[derive(Clone, Copy, Debug)]
struct Voice {
    stages: [StageCfg; MAX_STAGES],
    n_stages: usize,
    /// Gain/volume pot (+ bright shelf) sits in front of this stage (`n_stages` = after the last).
    gain_pos: usize,
    /// Tone stack sits in front of this stage (`n_stages` = after the last, [`NONE`] = no stack).
    ts_pos: usize,
    ts: ToneStackParts,
    /// Bright cap shelf corner, max boost (dB), and how much the boost falls with the gain pot (0..1).
    bright_hz: f64,
    bright_db: f64,
    bright_track: f64,
    /// Power-section input gain at full master (or fixed when the amp has no master).
    power_gain: f64,
    power: Curve,
    power_hp_hz: f64,
    /// Low-pass in front of the power stage (phase inverter / output transformer bandwidth).
    power_lp_hz: f64,
    /// Presence shelf corner (Hz); dB range is `PRES_DB`.
    pres_hz: f64,
    /// Sag depth at SAG = max, and envelope release.
    sag_max: f64,
    sag_release_s: f64,
    cab: Cab,
    makeup_db: f64,
}

/// Gain/volume pot residual at 0 (-60 dB).
const GAIN_FLOOR: f64 = 0.001;
/// PRESENCE knob range (dB at 0 and 10) [I].
pub const PRES_DB: (f64, f64) = (-3.0, 9.0);
/// ANGEL MID BOOST / BOTTOM [I].
pub const MIDB: (f64, f64, f64) = (650.0, 0.8, 7.0);
pub const BOTTOM: (f64, f64) = (110.0, 6.0);

fn voice(kind: AmpKind) -> Voice {
    match kind {
        AmpKind::UkRock => Voice {
            stages: [
                st(2.5, 1.0, 2.0, 30.0, 12_000.0),
                st(60.0, 1.0, 2.2, 90.0, 9_000.0),
                st(4.0, 1.0, 1.7, 20.0, 8_000.0),
                NO_STAGE,
            ],
            n_stages: 3,
            gain_pos: 1,
            ts_pos: 3,
            ts: ToneStackParts::JCM800,
            bright_hz: 2_000.0,
            bright_db: 8.0,
            bright_track: 0.85,
            power_gain: 6.0,
            power: Curve::sym(1.0),
            power_hp_hz: 25.0,
            power_lp_hz: 10_000.0,
            pres_hz: 3_500.0,
            sag_max: 0.8,
            sag_release_s: 0.15,
            cab: Cab::Greenback412,
            makeup_db: -12.0,
        },
        AmpKind::Angel => Voice {
            stages: [
                st(2.5, 1.0, 2.0, 40.0, 12_000.0),
                st(40.0, 1.0, 2.2, 140.0, 8_000.0),
                st(8.0, 1.0, 2.0, 60.0, 7_000.0),
                st(1.5, 1.5, 2.5, 20.0, 7_000.0),
            ],
            n_stages: 4,
            gain_pos: 1,
            ts_pos: 4,
            ts: ToneStackParts::ANGEL,
            bright_hz: 2_000.0,
            bright_db: 9.0,
            bright_track: 0.6,
            power_gain: 6.0,
            power: Curve::sym(1.0),
            power_hp_hz: 30.0,
            power_lp_hz: 9_000.0,
            pres_hz: 3_500.0,
            sag_max: 0.4,
            sag_release_s: 0.1,
            cab: Cab::V30412,
            makeup_db: -12.0,
        },
        AmpKind::Deluxe => Voice {
            stages: [
                st(2.5, 1.0, 2.0, 20.0, 15_000.0),
                st(30.0, 1.0, 2.0, 20.0, 10_000.0),
                NO_STAGE,
                NO_STAGE,
            ],
            n_stages: 2,
            gain_pos: 1,
            ts_pos: 1,
            ts: ToneStackParts::DELUXE,
            bright_hz: 2_000.0,
            bright_db: 0.0,
            bright_track: 0.0,
            power_gain: 1.2,
            power: Curve::sym(1.0),
            power_hp_hz: 20.0,
            power_lp_hz: 14_000.0,
            pres_hz: 3_500.0,
            sag_max: 1.5,
            sag_release_s: 0.25,
            cab: Cab::Open112,
            makeup_db: -5.0,
        },
        AmpKind::JazzClean => Voice {
            stages: [
                st(3.0, 4.0, 4.0, 20.0, 20_000.0),
                NO_STAGE,
                NO_STAGE,
                NO_STAGE,
            ],
            n_stages: 1,
            gain_pos: 0,
            ts_pos: 1,
            ts: ToneStackParts::JC120,
            bright_hz: 3_000.0,
            bright_db: 6.0,
            bright_track: 0.0,
            power_gain: 3.0,
            power: Curve::sym(1.0),
            power_hp_hz: 20.0,
            power_lp_hz: 20_000.0,
            pres_hz: 3_500.0,
            sag_max: 0.0,
            sag_release_s: 0.1,
            cab: Cab::Jc212,
            makeup_db: 8.0,
        },
        AmpKind::RackAmp => Voice {
            stages: [
                st(4.0, 1.0, 1.4, 40.0, 10_000.0),
                st(4.0, 1.0, 1.25, 30.0, 8_000.0),
                NO_STAGE,
                NO_STAGE,
            ],
            n_stages: 2,
            gain_pos: 0,
            ts_pos: NONE,
            ts: ToneStackParts::BASSMAN,
            bright_hz: 2_000.0,
            bright_db: 0.0,
            bright_track: 0.0,
            power_gain: 1.0,
            power: Curve::sym(1.0),
            power_hp_hz: 25.0,
            power_lp_hz: 10_000.0,
            pres_hz: 3_500.0,
            sag_max: 0.0,
            sag_release_s: 0.1,
            cab: Cab::SansAmp,
            makeup_db: -16.0,
        },
    }
}

/// Logical controls (index into the normalised control vector).
mod ctl {
    pub const GAIN: usize = 0;
    pub const BASS: usize = 1;
    pub const MID: usize = 2;
    pub const TREB: usize = 3;
    pub const PRES: usize = 4;
    pub const MSTR: usize = 5;
    pub const OUT: usize = 6;
    pub const SAG: usize = 7;
    pub const CAB: usize = 8;
    pub const BRI: usize = 9;
    pub const MIDB: usize = 10;
    pub const BOT: usize = 11;
    pub const BUZZ: usize = 12;
    pub const PUNCH: usize = 13;
    pub const CRUNCH: usize = 14;
    pub const DRIVE: usize = 15;
    pub const LEQ: usize = 16;
    pub const HEQ: usize = 17;
    pub const N: usize = 18;
}

/// Max schema slots of an amp model.
const MAX_SLOTS: usize = 12;

/// RACKAMP schema slots.
pub mod rack_slot {
    pub const PRE: u16 = 1;
    pub const BUZZ: u16 = 2;
    pub const PUNCH: u16 = 3;
    pub const CRUNCH: u16 = 4;
    pub const DRIVE: u16 = 5;
    pub const OUT: u16 = 6;
    pub const LEQ: u16 = 7;
    pub const HEQ: u16 = 8;
    pub const CAB: u16 = 9;
}

/// UKROCK / ANGEL schema slots (MIDB..BOT: ANGEL only).
pub mod tube_slot {
    pub const GAIN: u16 = 1;
    pub const BASS: u16 = 2;
    pub const MID: u16 = 3;
    pub const TREB: u16 = 4;
    pub const PRES: u16 = 5;
    pub const MSTR: u16 = 6;
    pub const OUT: u16 = 7;
    pub const SAG: u16 = 8;
    pub const CAB: u16 = 9;
    pub const MIDB: u16 = 10;
    pub const BRI: u16 = 11;
    pub const BOT: u16 = 12;
}

/// JAZZC schema slots.
pub mod jazz_slot {
    pub const VOL: u16 = 1;
    pub const BASS: u16 = 2;
    pub const MID: u16 = 3;
    pub const TREB: u16 = 4;
    pub const OUT: u16 = 5;
    pub const BRI: u16 = 6;
    pub const CAB: u16 = 7;
}

/// DELUXE schema slots.
pub mod deluxe_slot {
    pub const VOL: u16 = 1;
    pub const BASS: u16 = 2;
    pub const TREB: u16 = 3;
    pub const OUT: u16 = 4;
    pub const SAG: u16 = 5;
    pub const CAB: u16 = 6;
}

const K10: (f32, f32, f32) = (0.0, 10.0, 5.0);
const ON: (f32, f32, f32) = (0.0, 1.0, 1.0);
const OFF: (f32, f32, f32) = (0.0, 1.0, 0.0);
const K_DLX: (f32, f32, f32) = (1.0, 10.0, 5.5);

/// Per kind: schema slots as (min, max, default) and their logical control.
fn slot_table(kind: AmpKind) -> &'static [((f32, f32, f32), usize)] {
    use ctl::*;
    match kind {
        AmpKind::RackAmp => &[
            (K10, GAIN),
            (K10, BUZZ),
            (K10, PUNCH),
            (K10, CRUNCH),
            (K10, DRIVE),
            (K10, OUT),
            (K10, LEQ),
            (K10, HEQ),
            (ON, CAB),
        ],
        AmpKind::UkRock => &[
            (K10, GAIN),
            (K10, BASS),
            (K10, MID),
            (K10, TREB),
            (K10, PRES),
            (K10, MSTR),
            (K10, OUT),
            (K10, SAG),
            (ON, CAB),
        ],
        AmpKind::Angel => &[
            (K10, GAIN),
            (K10, BASS),
            (K10, MID),
            (K10, TREB),
            (K10, PRES),
            (K10, MSTR),
            (K10, OUT),
            (K10, SAG),
            (ON, CAB),
            (OFF, MIDB),
            (OFF, BRI),
            (OFF, BOT),
        ],
        AmpKind::JazzClean => &[
            (K10, GAIN),
            (K10, BASS),
            (K10, MID),
            (K10, TREB),
            (K10, OUT),
            (OFF, BRI),
            (ON, CAB),
        ],
        AmpKind::Deluxe => &[
            (K_DLX, GAIN),
            (K_DLX, BASS),
            (K_DLX, TREB),
            (K_DLX, OUT),
            (K_DLX, SAG),
            (ON, CAB),
        ],
    }
}

/// Gains derived from the controls (linear), ramped per sample across a chunk.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
struct Gains {
    knob: f64,
    master: f64,
    out: f64,
    sag: f64,
}

/// Per-channel DSP state.
#[derive(Clone, Copy, Debug, Default)]
struct ChanDsp {
    pre_eq: [Biquad; 3],
    post_eq: [Biquad; 2],
    cab: [Biquad; CAB_SECTIONS],
    stages: [(Fo, Adaa2, BiquadF64); MAX_STAGES],
    bright: Fo,
    ts: ToneStack,
    power_hp: Fo,
    power_lp: BiquadF64,
    pres: Fo,
    bottom: Fo,
    power: Adaa2,
    rect: RectAdaa,
    env: f64,
    align_buf: [f64; OS_DELAY],
    align_pos: usize,
    dc: Fo,
}

impl ChanDsp {
    fn reset(&mut self) {
        for b in self
            .pre_eq
            .iter_mut()
            .chain(self.post_eq.iter_mut())
            .chain(self.cab.iter_mut())
        {
            b.reset();
        }
        for (hp, a, lp) in &mut self.stages {
            hp.reset();
            a.reset();
            lp.reset();
        }
        self.bright.reset();
        self.ts.reset();
        self.power_hp.reset();
        self.power_lp.reset();
        self.pres.reset();
        self.bottom.reset();
        self.power.reset();
        self.rect = RectAdaa::default();
        self.env = 0.0;
        self.align_buf = [0.0; OS_DELAY];
        self.align_pos = 0;
        self.dc.reset();
    }

    fn flush(&mut self) {
        for (hp, _, lp) in &mut self.stages {
            hp.flush();
            lp.flush_denormals();
        }
        self.power_lp.flush_denormals();
        self.bright.flush();
        self.ts.flush();
        self.power_hp.flush();
        self.pres.flush();
        self.bottom.flush();
        self.dc.flush();
        for b in self
            .pre_eq
            .iter_mut()
            .chain(self.post_eq.iter_mut())
            .chain(self.cab.iter_mut())
        {
            b.flush_denormals();
        }
    }

    /// Pure delay of `len` (< [`OS_DELAY`]) samples.
    #[inline(always)]
    fn align(&mut self, y: f64, len: usize) -> f64 {
        if len == 0 {
            return y;
        }
        let o = self.align_buf[self.align_pos];
        self.align_buf[self.align_pos] = y;
        self.align_pos += 1;
        if self.align_pos >= len {
            self.align_pos = 0;
        }
        o
    }

    /// Power amp with rail sag: the feed-forward detector follows the (anti-aliased, clipped) drive
    /// level, the rail `V = 1 / (1 + sag env)` sets the clip level: `y = V f(u / V)`.
    #[inline(always)]
    fn power_tick(&mut self, curve: Curve, u: f64, sag: f64, att: f64, rel: f64) -> f64 {
        let a = self.rect.tick(u);
        self.env += if a > self.env { att } else { rel } * (a - self.env);
        if sag > 0.0 {
            let g = 1.0 + sag * self.env;
            self.power.tick(curve, u * g) / g
        } else {
            self.power.tick(curve, u)
        }
    }
}

/// The oversampled section over one chunk (`w0`/`w1` = left/right at the 4x rate, in place). Runs stage by
/// stage over the whole chunk with both channels interleaved: each loop's only loop-carried dependencies
/// are its filter states, so successive samples (and the two channels) overlap in the pipeline instead
/// of waiting on the full per-sample chain.
#[allow(clippy::too_many_arguments)]
fn os_section(
    v: &Voice,
    c0: &mut ChanDsp,
    c1: &mut ChanDsp,
    w0: &mut [f64],
    w1: &mut [f64],
    g0: Gains,
    g1: Gains,
    att: f64,
    rel: f64,
) {
    let n = w0.len().min(w1.len());
    let (w0, w1) = (&mut w0[..n], &mut w1[..n]);
    let inv = 1.0 / n as f64;
    let ramp = |a: f64, b: f64, i: usize| a + (b - a) * ((i + 1) as f64 * inv);
    for s in 0..=v.n_stages {
        if s == v.ts_pos {
            for (a, b) in w0.iter_mut().zip(w1.iter_mut()) {
                *a = c0.ts.tick(*a);
                *b = c1.ts.tick(*b);
            }
        }
        if s == v.gain_pos {
            for (i, (a, b)) in w0.iter_mut().zip(w1.iter_mut()).enumerate() {
                let k = ramp(g0.knob, g1.knob, i);
                *a = c0.bright.tick(*a * k);
                *b = c1.bright.tick(*b * k);
            }
        }
        if s == v.n_stages {
            break;
        }
        let cfg = v.stages[s];
        let (hp0, ad0, lp0) = &mut c0.stages[s];
        let (hp1, ad1, lp1) = &mut c1.stages[s];
        for (a, b) in w0.iter_mut().zip(w1.iter_mut()) {
            let u0 = cfg.gain * hp0.tick(*a);
            let u1 = cfg.gain * hp1.tick(*b);
            *a = lp0.tick(ad0.tick(cfg.curve, u0));
            *b = lp1.tick(ad1.tick(cfg.curve, u1));
        }
    }
    // presence / depth shelves, then the band limit (keeps the presence boost from sharpening ultrasonic
    // edges into the power stage), then the master
    for (i, (a, b)) in w0.iter_mut().zip(w1.iter_mut()).enumerate() {
        let m = ramp(g0.master, g1.master, i);
        *a = c0
            .power_lp
            .tick(c0.bottom.tick(c0.pres.tick(c0.power_hp.tick(*a))))
            * m;
        *b = c1
            .power_lp
            .tick(c1.bottom.tick(c1.pres.tick(c1.power_hp.tick(*b))))
            * m;
    }
    // power amp + pure alignment delay completing the section delay to OS_DELAY samples
    let align_len = OS_DELAY - (v.n_stages + 1);
    for (i, (a, b)) in w0.iter_mut().zip(w1.iter_mut()).enumerate() {
        let sag = ramp(g0.sag, g1.sag, i);
        let ya = c0.power_tick(v.power, *a, sag, att, rel);
        let yb = c1.power_tick(v.power, *b, sag, att, rel);
        *a = c0.align(ya, align_len);
        *b = c1.align(yb, align_len);
    }
}

/// `RACKAMP` / `UKROCK` / `ANGEL` / `JAZZC` / `DELUXE`, see the module docs.
pub struct Amp {
    kind: AmpKind,
    voice: Voice,
    fs: f64,
    max_block: usize,
    /// Staged schema values (engineering units).
    v: [f32; MAX_SLOTS],
    dirty: bool,
    sm: [OnePoleSmoother; MAX_SLOTS],
    /// Smoothed slot values used by the last coefficient design.
    designed: [f32; MAX_SLOTS],
    gains: Gains,
    cab_w: f32,
    cab_c: f32,
    att: f64,
    rel: f64,
    ch: [ChanDsp; 2],
    os: [Oversampler4x; 2],
    hi: [Vec<f32>; 2],
    work: [[f64; OS * CHUNK]; 2],
}

impl Amp {
    pub fn new(kind: AmpKind) -> Self {
        let mut m = Self {
            kind,
            voice: voice(kind),
            fs: 48_000.0,
            max_block: 0,
            v: [0.0; MAX_SLOTS],
            dirty: true,
            sm: [OnePoleSmoother::new(0.0, SMOOTH_S, 48_000.0); MAX_SLOTS],
            designed: [f32::NAN; MAX_SLOTS],
            gains: Gains::default(),
            cab_w: 1.0,
            cab_c: 0.0,
            att: 0.0,
            rel: 0.0,
            ch: [ChanDsp::default(); 2],
            os: [Oversampler4x::new(1), Oversampler4x::new(1)],
            hi: [Vec::new(), Vec::new()],
            work: [[0.0; OS * CHUNK]; 2],
        };
        m.load_defaults();
        m.prepare(48_000.0, 64);
        m
    }

    pub fn kind(&self) -> AmpKind {
        self.kind
    }

    fn slots(&self) -> &'static [((f32, f32, f32), usize)] {
        slot_table(self.kind)
    }

    /// Normalised control vector (knobs 0..1, switches 0..1) and raw values from slot values `cur`.
    fn controls(&self, cur: &[f32; MAX_SLOTS]) -> ([f64; ctl::N], [f64; ctl::N]) {
        let mut k = [0.5f64; ctl::N];
        let mut raw = [5.0f64; ctl::N];
        // controls a model lacks
        k[ctl::MID] = 1.0; // DELUXE: fixed middle resistor fully in circuit
        k[ctl::PRES] = 0.25; // 0 dB presence
        k[ctl::CAB] = 1.0;
        for c in [ctl::BRI, ctl::MIDB, ctl::BOT, ctl::SAG] {
            k[c] = 0.0;
        }
        for (i, &((lo, hi, def), c)) in self.slots().iter().enumerate() {
            let x = cur[i] as f64;
            raw[c] = x / def.max(1e-6) as f64;
            k[c] = ((x - lo as f64) / (hi - lo) as f64).clamp(0.0, 1.0);
        }
        (k, raw)
    }

    fn compute_gains(&self, k: &[f64; ctl::N], raw: &[f64; ctl::N]) -> Gains {
        let v = &self.voice;
        let out = raw[ctl::OUT].max(0.0).powi(2) * db(v.makeup_db);
        let sag = k[ctl::SAG] * v.sag_max;
        match self.kind {
            AmpKind::UkRock | AmpKind::Angel => Gains {
                knob: GAIN_FLOOR + audio_taper(k[ctl::GAIN], 4.6),
                master: v.power_gain * (0.01 + audio_taper(k[ctl::MSTR], 3.0)),
                out,
                sag,
            },
            AmpKind::Deluxe => Gains {
                knob: GAIN_FLOOR + audio_taper(k[ctl::GAIN], 4.0),
                master: v.power_gain,
                out,
                sag,
            },
            AmpKind::JazzClean => Gains {
                knob: GAIN_FLOOR + audio_taper(k[ctl::GAIN], 3.5),
                master: v.power_gain,
                out,
                sag,
            },
            AmpKind::RackAmp => Gains {
                knob: db(-12.0 + 36.0 * k[ctl::GAIN]),
                master: v.power_gain * db(-12.0 + 36.0 * k[ctl::DRIVE]),
                out,
                sag,
            },
        }
    }

    /// Recompute all filter coefficients from the control vector.
    fn design(&mut self, k: &[f64; ctl::N]) {
        let v = self.voice;
        let fs = self.fs;
        let fo = fs * OS as f64;
        // base-rate pre / post EQ
        let mut pre = [Coeffs::IDENTITY; 3];
        let mut post = [Coeffs::IDENTITY; 2];
        match self.kind {
            AmpKind::RackAmp => {
                let bip = |x: f64, r: f64| (x - 0.5) * 2.0 * r;
                pre[0] = Coeffs::low_shelf(fs, 180.0, 0.7, bip(k[ctl::BUZZ], 12.0));
                pre[1] = Coeffs::peak(fs, 400.0, 0.8, bip(k[ctl::PUNCH], 10.0));
                pre[2] = Coeffs::peak(fs, 2_800.0, 0.8, bip(k[ctl::CRUNCH], 12.0));
                post[0] = Coeffs::low_shelf(fs, 120.0, 0.7, bip(k[ctl::LEQ], 12.0));
                post[1] = Coeffs::high_shelf(fs, 3_300.0, 0.7, bip(k[ctl::HEQ], 12.0));
            }
            AmpKind::Angel => {
                pre[0] = Coeffs::peak(fs, MIDB.0, MIDB.1, MIDB.2 * k[ctl::MIDB]);
            }
            _ => {}
        }
        let bright_on = match self.kind {
            AmpKind::Angel | AmpKind::JazzClean => k[ctl::BRI],
            _ => 1.0,
        };
        let bright_db = v.bright_db * bright_on * (1.0 - v.bright_track * k[ctl::GAIN]);
        let bright = Fo::high_shelf(fo, v.bright_hz, bright_db);
        let ts = v.ts.digital(
            fo,
            k[ctl::TREB],
            k[ctl::MID],
            audio_taper(k[ctl::BASS], 3.4),
        );
        let pres = match self.kind {
            AmpKind::UkRock | AmpKind::Angel => Fo::high_shelf(
                fo,
                v.pres_hz,
                PRES_DB.0 + (PRES_DB.1 - PRES_DB.0) * k[ctl::PRES],
            ),
            _ => Fo::IDENTITY,
        };
        let bottom = match self.kind {
            AmpKind::Angel => Fo::low_shelf(fo, BOTTOM.0, BOTTOM.1 * k[ctl::BOT]),
            _ => Fo::IDENTITY,
        };
        for c in &mut self.ch {
            for (b, &co) in c.pre_eq.iter_mut().zip(&pre) {
                b.set_coeffs(co);
            }
            for (b, &co) in c.post_eq.iter_mut().zip(&post) {
                b.set_coeffs(co);
            }
            c.bright.set(bright);
            c.ts.set(ts);
            c.pres.set(pres);
            c.bottom.set(bottom);
        }
    }

    /// Fixed (sample-rate dependent) coefficients.
    fn design_fixed(&mut self) {
        let v = self.voice;
        let fs = self.fs;
        let fo = fs * OS as f64;
        let cab = v.cab.coeffs(fs);
        for c in &mut self.ch {
            for (s, cfg) in c.stages.iter_mut().zip(&v.stages) {
                s.0.set(Fo::highpass(fo, cfg.hp_hz));
                s.2.set_coeffs(Coeffs::lowpass(fo, cfg.lp_hz, LP_Q));
            }
            c.power_hp.set(Fo::highpass(fo, v.power_hp_hz));
            c.power_lp
                .set_coeffs(Coeffs::lowpass(fo, v.power_lp_hz, LP_Q));
            for (b, &co) in c.cab.iter_mut().zip(&cab) {
                b.set_coeffs(co);
            }
            c.dc.set(Fo::highpass(fs, DC_HZ));
        }
        self.att = 1.0 - one_pole_coeff(SAG_ATTACK_S, fo);
        self.rel = 1.0 - one_pole_coeff(v.sag_release_s, fo);
        self.cab_c = 1.0 - one_pole_coeff(CAB_XFADE_S, fs) as f32;
    }

    fn apply(&mut self) {
        if !self.dirty {
            return;
        }
        self.dirty = false;
        let n = self.slots().len();
        for i in 0..n {
            self.sm[i].set_time(SMOOTH_S, self.fs);
            self.sm[i].set_target(self.v[i]);
        }
    }

    fn process_chunk(&mut self, left: &mut [f32], right: &mut [f32]) {
        let n = left.len();
        let ns = self.slots().len();
        let mut cur = [0.0f32; MAX_SLOTS];
        for (c, s) in cur.iter_mut().zip(&mut self.sm).take(ns) {
            *c = s.advance(n);
        }
        let (k, raw) = self.controls(&cur);
        if cur
            .iter()
            .zip(&self.designed)
            .take(ns)
            .any(|(a, b)| (a - b).abs() >= 1e-5 || b.is_nan())
        {
            self.design(&k);
            self.designed = cur;
        }
        let g0 = self.gains;
        let g1 = self.compute_gains(&k, &raw);
        self.gains = g1;
        let cab_target = cur[self
            .slots()
            .iter()
            .position(|s| s.1 == ctl::CAB)
            .unwrap_or(0)]
        .round()
        .clamp(0.0, 1.0);

        // 1. sanitise + pre-EQ (base rate)
        let has_pre = matches!(self.kind, AmpKind::RackAmp | AmpKind::Angel);
        for (c, buf) in [&mut *left, &mut *right].into_iter().enumerate() {
            let ch = &mut self.ch[c];
            for x in buf.iter_mut() {
                let s = if x.is_finite() {
                    x.clamp(-IN_LIMIT, IN_LIMIT)
                } else {
                    0.0
                };
                *x = if has_pre {
                    let mut y = s;
                    for b in &mut ch.pre_eq {
                        y = b.tick(y);
                    }
                    y
                } else {
                    s
                };
            }
        }

        // 2. upsample
        let nh = OS * n;
        self.os[0].upsample(left, &mut self.hi[0][..nh]);
        self.os[1].upsample(right, &mut self.hi[1][..nh]);

        // 3. oversampled section (f64 work buffers)
        {
            let [h0, h1] = &mut self.hi;
            let [w0, w1] = &mut self.work;
            for (w, h) in w0[..nh].iter_mut().zip(&h0[..nh]) {
                *w = *h as f64;
            }
            for (w, h) in w1[..nh].iter_mut().zip(&h1[..nh]) {
                *w = *h as f64;
            }
            let [c0, c1] = &mut self.ch;
            os_section(
                &self.voice,
                c0,
                c1,
                &mut w0[..nh],
                &mut w1[..nh],
                g0,
                g1,
                self.att,
                self.rel,
            );
            for (h, w) in h0[..nh].iter_mut().zip(&w0[..nh]) {
                *h = *w as f32;
            }
            for (h, w) in h1[..nh].iter_mut().zip(&w1[..nh]) {
                *h = *w as f32;
            }
        }

        // 4. downsample
        self.os[0].downsample(&self.hi[0][..nh], left);
        self.os[1].downsample(&self.hi[1][..nh], right);

        // 5. post-EQ, cab (cross-faded), out gain, DC blocker
        let has_post = self.kind == AmpKind::RackAmp;
        let w_start = self.cab_w;
        let invn = 1.0 / n as f64;
        let cab_c = self.cab_c;
        let mut w_end = w_start;
        for (c, buf) in [&mut *left, &mut *right].into_iter().enumerate() {
            let ch = &mut self.ch[c];
            let mut w = w_start;
            let cab_active = w_start > 0.0 || cab_target > 0.0;
            for (i, x) in buf.iter_mut().enumerate() {
                let mut y = *x;
                if has_post {
                    for b in &mut ch.post_eq {
                        y = b.tick(y);
                    }
                }
                if cab_active {
                    w += cab_c * (cab_target - w);
                    if (w - cab_target).abs() < 1e-4 {
                        w = cab_target;
                    }
                    let mut yc = y;
                    for b in &mut ch.cab {
                        yc = b.tick(yc);
                    }
                    y += w * (yc - y);
                }
                let fr = (i + 1) as f64 * invn;
                let out = g0.out + (g1.out - g0.out) * fr;
                *x = (ch.dc.tick(y as f64) * out) as f32;
            }
            if !cab_active || w == 0.0 {
                for b in &mut ch.cab {
                    b.reset();
                }
            }
            w_end = w;
            ch.flush();
        }
        self.cab_w = w_end;
    }
}

impl Model for Amp {
    fn id(&self) -> &'static str {
        match self.kind {
            AmpKind::RackAmp => "RACKAMP",
            AmpKind::UkRock => "UKROCK",
            AmpKind::Angel => "ANGEL",
            AmpKind::JazzClean => "JAZZC",
            AmpKind::Deluxe => "DELUXE",
        }
    }

    fn prepare(&mut self, sample_rate: f64, max_block: usize) {
        self.fs = sample_rate;
        self.max_block = max_block.clamp(1, CHUNK);
        let mb = self.max_block;
        self.os = [Oversampler4x::new(mb), Oversampler4x::new(mb)];
        self.hi = [vec![0.0; OS * mb], vec![0.0; OS * mb]];
        self.design_fixed();
        self.dirty = true;
        self.reset();
    }

    fn process(
        &mut self,
        left: &mut [f32],
        right: &mut [f32],
        _sidechain: Option<(&[f32], &[f32])>,
    ) {
        self.apply();
        let n = left.len().min(right.len());
        let mut i = 0;
        while i < n {
            let e = (i + self.max_block).min(n);
            self.process_chunk(&mut left[i..e], &mut right[i..e]);
            i = e;
        }
    }

    fn latency(&self) -> usize {
        AMP_LATENCY
    }

    fn reset(&mut self) {
        self.apply();
        for s in &mut self.sm {
            s.reset(s.target());
        }
        for c in &mut self.ch {
            c.reset();
        }
        for o in &mut self.os {
            o.reset();
        }
        let mut cur = [0.0f32; MAX_SLOTS];
        for (c, s) in cur.iter_mut().zip(&self.sm) {
            *c = s.value();
        }
        let (k, raw) = self.controls(&cur);
        self.design(&k);
        self.designed = cur;
        self.gains = self.compute_gains(&k, &raw);
        self.cab_w = k[ctl::CAB].round() as f32;
    }

    fn set_param(&mut self, slot: u16, value: ParamValue) {
        let Some(i) = (slot as usize).checked_sub(1) else {
            return;
        };
        let Some(&((lo, hi, _), c)) = self.slots().get(i) else {
            return;
        };
        let mut x = value.as_f32();
        if !x.is_finite() {
            return;
        }
        if matches!(c, ctl::CAB | ctl::BRI | ctl::MIDB | ctl::BOT) {
            x = x.round();
        }
        self.v[i] = x.clamp(lo, hi);
        self.dirty = true;
    }

    fn param(&self, slot: u16) -> Option<f32> {
        let i = (slot as usize).checked_sub(1)?;
        self.slots().get(i).map(|_| self.v[i])
    }

    fn load_defaults(&mut self) {
        for (d, s) in self.v.iter_mut().zip(slot_table(self.kind)) {
            *d = s.0 .2;
        }
        self.dirty = true;
    }

    fn commit(&mut self) {
        self.apply();
    }
}

// ---------------------------------------------------------------------------------------------------------
// BODY (BODYREZ)
// ---------------------------------------------------------------------------------------------------------

/// BODY schema slot.
pub const BODY_SLOT: u16 = 1;
/// BODY default (schema).
pub const BODY_DEFAULT: f32 = 25.0;
/// Number of body EQ sections.
pub const BODY_SECTIONS: usize = 4;

/// Body EQ at amount `a` (0..1) [I]: air-cavity (Helmholtz) resonance, top-plate resonance, piezo "quack"
/// cut, string-zing shelf.
pub fn body_coeffs(fs: f64, a: f64) -> [Coeffs; BODY_SECTIONS] {
    let a = a.clamp(0.0, 1.0);
    [
        Coeffs::peak(fs, 95.0, 3.0, 12.0 * a),
        Coeffs::peak(fs, 210.0, 2.0, 6.0 * a),
        Coeffs::peak(fs, 1_300.0, 0.9, -5.0 * a),
        Coeffs::high_shelf(fs, 7_000.0, 0.7, -4.0 * a),
    ]
}

/// `BODY`: BODYREZ, see the module docs. Linear, zero latency.
pub struct BodyRez {
    fs: f64,
    v: f32,
    sm: OnePoleSmoother,
    designed: f32,
    eq: [[Biquad; BODY_SECTIONS]; 2],
}

impl BodyRez {
    pub fn new() -> Self {
        Self {
            fs: 48_000.0,
            v: BODY_DEFAULT,
            sm: OnePoleSmoother::new(BODY_DEFAULT, SMOOTH_S, 48_000.0),
            designed: f32::NAN,
            eq: [[Biquad::default(); BODY_SECTIONS]; 2],
        }
    }

    fn design(&mut self, v: f32) {
        let c = body_coeffs(self.fs, v as f64 / 100.0);
        for ch in &mut self.eq {
            for (b, &co) in ch.iter_mut().zip(&c) {
                b.set_coeffs(co);
            }
        }
        self.designed = v;
    }
}

impl Default for BodyRez {
    fn default() -> Self {
        Self::new()
    }
}

impl Model for BodyRez {
    fn id(&self) -> &'static str {
        "BODY"
    }

    fn prepare(&mut self, sample_rate: f64, _max_block: usize) {
        self.fs = sample_rate;
        self.sm.set_time(SMOOTH_S, sample_rate);
        self.reset();
    }

    fn process(
        &mut self,
        left: &mut [f32],
        right: &mut [f32],
        _sidechain: Option<(&[f32], &[f32])>,
    ) {
        self.sm.set_target(self.v);
        let n = left.len().min(right.len());
        let mut i = 0;
        while i < n {
            let e = (i + CHUNK).min(n);
            let v = self.sm.advance(e - i);
            if self.designed.is_nan() || (v - self.designed).abs() >= 0.01 {
                self.design(v);
            }
            for (c, buf) in [&mut left[i..e], &mut right[i..e]].into_iter().enumerate() {
                let eq = &mut self.eq[c];
                for x in buf.iter_mut() {
                    let mut y = if x.is_finite() {
                        x.clamp(-IN_LIMIT, IN_LIMIT)
                    } else {
                        0.0
                    };
                    for b in eq.iter_mut() {
                        y = b.tick(y);
                    }
                    *x = y;
                }
                for b in eq.iter_mut() {
                    b.flush_denormals();
                }
            }
            i = e;
        }
    }

    fn reset(&mut self) {
        self.sm.reset(self.v);
        for ch in &mut self.eq {
            for b in ch.iter_mut() {
                b.reset();
            }
        }
        self.design(self.v);
    }

    fn set_param(&mut self, slot: u16, value: ParamValue) {
        let x = value.as_f32();
        if slot == BODY_SLOT && x.is_finite() {
            self.v = x.clamp(0.0, 100.0);
        }
    }

    fn param(&self, slot: u16) -> Option<f32> {
        (slot == BODY_SLOT).then_some(self.v)
    }

    fn load_defaults(&mut self) {
        self.v = BODY_DEFAULT;
    }
}

#[cfg(test)]
mod tests;
