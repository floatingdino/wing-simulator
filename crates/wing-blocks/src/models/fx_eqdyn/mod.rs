//! FX-rack EQ and dynamics models (T05Q): `GEQ`, `PIA`, `LIMITER`, `DE-S2`, `DEQ3`, `C5-CMB`, `SPKMAN` and the
//! FX-rack versions of the channel EQs `SOUL`, `E88`, `E84`, `F110`, `PULSAR`, `MACH4`
//! (models.md §6, slot order and defaults from `data/schema/wing-tree.json`).
//!
//! Like every FX rack model these output the **processed** signal; the FX slot host blends it with the dry
//! signal by `fxmix` (wing-engine `FxSlot`). Every model is stereo in / stereo out.
//!
//! # Sources: what is exact and what is designed
//!
//! WING-Edit 3.3.4 contains the FX parameter code the console runs (`ae::fx_parchg_<model>`: the block of
//! coefficients and constants it posts to the DSP). `tools/curve-extract/fx_eqdyn.py` runs these functions in
//! the Unicorn emulator (never natively) and records what they post (`data/reference-curves/fx-eqdyn.json`):
//!
//! * **FX EQs** ([`FxEq`]): `fx_parchg_{ssl_g,neve88rs,neve84,pultec_duo}` call the channel `eq_parchg_*` with
//!   `mix/100` as the gain scale (disassembly 0x100076a54...), `isa110`/`maag4`/`api560` inline the same code.
//!   The FX models are therefore the channel models of [`crate::models::eq_alt`] behind one extra slot
//!   (slot 1 = `mix` 0..125 %, the same gain-scale law as the EQ block's `mix`); the oracle test checks every
//!   recorded FX coefficient set against [`crate::models::eq_alt::AltKind::design`].
//! * **GEQ** ([`geq`]): exact port of `fx_parchg_geq` (31 `hqpeq` bands, proportional Q, TRU neighbour
//!   compensation, TRIM on the last section).
//! * **SPKMAN** ([`spkman`]): the linear part is an exact port of `fx_parchg_spkpro` (`Cxover48` HP/LP, the
//!   `hqhs12q25` tilt shelf, the phase all-pass with polarity, the distance/position delays). Its dynamic EQ
//!   reuses the channel DEQ band ([`crate::models::dyn_eq::DeqBand`]); its limiter follows the posted
//!   constants (RMS detector scaled by sqrt 2, release `T*1000/rel`).
//! * **DEQ3** ([`deq3`]): three channel DEQ bands ([`crate::models::dyn_eq::DeqBand`]) in series (same
//!   slot layout per band as `DEQ`).
//! * **LIMITER**, **DE-S2**, **C5-CMB**: the console's detector/gain-computer code is SHARC firmware, not in
//!   WING-Edit; only the parameter mapping is visible (e.g. LIMITER `gin-18`/`gout+18` dB staging, attack and
//!   release as `T*1000/ms` rates; C5-CMB's fixed crossovers at `20 kHz / 10^(0.6 m)`, m = 1..4, LR24/LR48).
//!   The DSP is designed here from those facts and the hardware behaviour; every such choice is marked [I] in
//!   the submodule docs.
//!
//! Real-time: all buffers are allocated in `new`/`prepare`; `process`, `set_param`, `commit` and `reset`
//! never allocate. Coefficient changes ramp linearly per sample in direct form I (f64), which is
//! click-free and cannot blow up (a linear blend of two stable sections stays stable).

// Ports keep WING's fmin/fmax order (NaN behaviour); DSP loops index several parallel arrays.
#![allow(clippy::manual_clamp, clippy::needless_range_loop)]

pub mod combinator;
pub mod deesser;
pub mod deq3;
pub mod geq;
pub mod limiter;
pub mod spkman;

use crate::model::{Model, ModelTelemetry, ParamValue};
use crate::models::eq_alt::{AltEq, AltKind};
use crate::wingcoef::Bq;

pub use combinator::Combinator;
pub use deesser::DeEsser2;
pub use deq3::TripleDeq;
pub use geq::GraphicEq;
pub use limiter::PrecisionLimiter;
pub use spkman::SpeakerManager;

/// Filter states below this are flushed (denormal guard).
pub(crate) const DENORMAL: f64 = 1e-30;

/// `[b0, b1, b2, a1, a2]` as f64.
#[inline]
pub(crate) fn coef(b: &Bq) -> [f64; 5] {
    [
        b[0] as f64,
        b[1] as f64,
        b[2] as f64,
        b[3] as f64,
        b[4] as f64,
    ]
}

/// One stereo direct-form-I section (f64) whose coefficients can ramp linearly to a new target.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Section {
    c: [f64; 5],
    step: [f64; 5],
    target: [f64; 5],
    left: u32,
    /// `[x1, x2, y1, y2]` per channel.
    s: [[f64; 4]; 2],
}

impl Default for Section {
    fn default() -> Self {
        let id = [1.0, 0.0, 0.0, 0.0, 0.0];
        Self {
            c: id,
            step: [0.0; 5],
            target: id,
            left: 0,
            s: [[0.0; 4]; 2],
        }
    }
}

impl Section {
    /// Jump to `b` (no ramp).
    pub fn set(&mut self, b: &Bq) {
        self.c = coef(b);
        self.target = self.c;
        self.left = 0;
    }

    /// Jump to f64 coefficients `[b0, b1, b2, a1, a2]` (no ramp).
    pub fn set64(&mut self, c: [f64; 5]) {
        self.c = c;
        self.target = c;
        self.left = 0;
    }

    /// Ramp to `b` over `n` samples (`n = 0` jumps).
    pub fn ramp_to(&mut self, b: &Bq, n: u32) {
        let t = coef(b);
        if t == self.target && self.left == 0 {
            return;
        }
        self.target = t;
        if n == 0 {
            self.c = t;
            self.left = 0;
            return;
        }
        let inv = 1.0 / n as f64;
        for i in 0..5 {
            self.step[i] = (t[i] - self.c[i]) * inv;
        }
        self.left = n;
    }

    /// True while unity and not ramping (may be skipped).
    #[inline]
    pub fn is_identity(&self) -> bool {
        self.left == 0 && self.c == [1.0, 0.0, 0.0, 0.0, 0.0]
    }

    pub fn clear(&mut self) {
        self.s = [[0.0; 4]; 2];
    }

    /// Settle the coefficients at the target.
    pub fn settle(&mut self) {
        self.c = self.target;
        self.left = 0;
    }

    #[inline(always)]
    fn advance(&mut self) {
        if self.left > 0 {
            self.left -= 1;
            if self.left == 0 {
                self.c = self.target;
            } else {
                for i in 0..5 {
                    self.c[i] += self.step[i];
                }
            }
        }
    }

    #[inline(always)]
    fn tick(c: &[f64; 5], s: &mut [f64; 4], x: f64) -> f64 {
        let y = c[0] * x + c[1] * s[0] + c[2] * s[1] - c[3] * s[2] - c[4] * s[3];
        s[1] = s[0];
        s[0] = x;
        s[3] = s[2];
        s[2] = y;
        y
    }

    /// One stereo sample.
    #[inline(always)]
    pub fn process(&mut self, l: f64, r: f64) -> (f64, f64) {
        self.advance();
        let c = self.c;
        (
            Self::tick(&c, &mut self.s[0], l),
            Self::tick(&c, &mut self.s[1], r),
        )
    }

    /// One mono sample on channel `ch` (the caller advances the ramp once per sample with [`Self::step`]).
    #[inline(always)]
    pub fn process_ch(&mut self, ch: usize, x: f64) -> f64 {
        let c = self.c;
        Self::tick(&c, &mut self.s[ch], x)
    }

    /// Advance the coefficient ramp by one sample (for [`Self::process_ch`] users).
    #[inline(always)]
    pub fn step(&mut self) {
        self.advance();
    }

    pub fn flush(&mut self) {
        for v in self.s.iter_mut().flatten() {
            if v.abs() < DENORMAL {
                *v = 0.0;
            }
        }
    }
}

/// Complex response of f64 sections at `f` Hz.
pub(crate) fn response64(fs: f64, secs: &[[f64; 5]], f: f64) -> wing_dsp::Complex {
    let w = 2.0 * core::f64::consts::PI * f / fs;
    let z1 = wing_dsp::Complex::expj(-w);
    let z2 = z1 * z1;
    let one = wing_dsp::Complex::new(1.0, 0.0);
    let mut h = one;
    for s in secs {
        let num = one * s[0] + z1 * s[1] + z2 * s[2];
        let den = one + z1 * s[3] + z2 * s[4];
        h = h * (num / den);
    }
    h
}

/// `20 log10 |x|` with a floor.
#[inline]
pub(crate) fn lin2db(x: f64) -> f64 {
    20.0 * x.max(1e-12).log10()
}

// ---------------------------------------------------------------------------------------------------------
// FX-rack versions of the channel EQs
// ---------------------------------------------------------------------------------------------------------

/// FX-rack `SOUL`/`E88`/`E84`/`F110`/`PULSAR`/`MACH4`/`PIA`: the channel model ([`AltEq`]) with slot 1 =
/// `mix` (0..125 %, WING's gain-scale law; fixed at 100 % for PULSAR and MACH4 in the schema) and the channel
/// slots shifted by one. Defaults equal the channel defaults (the FX `init` strings match) plus `mix` 100.
pub struct FxEq {
    eq: AltEq,
    mix: f32,
}

impl FxEq {
    pub fn new(kind: AltKind) -> Self {
        Self {
            eq: AltEq::new(kind),
            mix: 100.0,
        }
    }

    pub fn kind(&self) -> AltKind {
        self.eq.kind()
    }

    /// The wrapped channel EQ.
    pub fn eq(&self) -> &AltEq {
        &self.eq
    }

    /// Snapshot id of an FX EQ kind.
    pub fn id_of(kind: AltKind) -> &'static str {
        kind.id()
    }
}

impl Model for FxEq {
    fn id(&self) -> &'static str {
        self.eq.kind().id()
    }

    fn prepare(&mut self, sample_rate: f64, max_block: usize) {
        self.eq.prepare(sample_rate, max_block);
    }

    fn process(&mut self, left: &mut [f32], right: &mut [f32], _sc: Option<(&[f32], &[f32])>) {
        self.eq.process(left, right, None);
    }

    fn reset(&mut self) {
        self.eq.reset();
    }

    fn set_param(&mut self, slot: u16, value: ParamValue) {
        match slot {
            0 => {}
            1 => {
                let fixed = matches!(self.eq.kind(), AltKind::Pulsar | AltKind::Mach4);
                let m = if fixed {
                    100.0
                } else {
                    value.as_f32().clamp(0.0, 125.0)
                };
                if m.is_finite() {
                    self.mix = m;
                    self.eq.set_gain_scale(m);
                }
            }
            s => self.eq.set_param(s - 1, value),
        }
    }

    fn param(&self, slot: u16) -> Option<f32> {
        match slot {
            0 => None,
            1 => Some(self.mix),
            s => self.eq.param(s - 1),
        }
    }

    fn load_defaults(&mut self) {
        self.eq.load_defaults();
        self.mix = 100.0;
        self.eq.set_gain_scale(100.0);
    }

    fn commit(&mut self) {
        self.eq.commit();
    }

    fn telemetry(&self) -> ModelTelemetry {
        ModelTelemetry::default()
    }
}

#[cfg(test)]
mod tests;
