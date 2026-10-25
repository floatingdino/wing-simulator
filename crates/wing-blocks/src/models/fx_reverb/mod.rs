//! FX rack algorithmic reverbs (T05R): `HALL`, `ROOM`, `CHAMBER`, `CONCERT`, `AMBI` (models.md §6), all on the
//! shared FDN core in [`core`] (design, sources and RT notes there).
//!
//! Every model outputs the **wet** signal only (the FX slot host blends by `fxmix`). A model is a
//! [`Reverb`]: raw WING parameter values by schema slot (clamped to the WING-Edit ranges of its [`ParamSpec`]
//! table), a per-model mapping to [`core::CoreParams`] and a per-model [`core::Character`] (geometry,
//! crossover, colour). The per-model files document the mapping and the emulated unit:
//!
//! | model | file | lines | emulates (models.md) |
//! |---|---|---|---|
//! | `HALL` | [`hall`] | 16 | WING hall (Lexicon/TC-style algorithmic hall) |
//! | `ROOM` | [`room`] | 8 | Lexicon 480L/224-style room with recirculating pre-echoes |
//! | `CHAMBER` | [`chamber`] | 8 | Lexicon-style chamber with echo taps + levels |
//! | `CONCERT` | [`concert`] | 16 | Lexicon-style concert hall with reflections, depth, chorus |
//! | `AMBI` | [`ambi`] | 8 | TC/Lexicon-style ambience (ER-dominant, short) |
//! | `V-ROOM` | [`vintage`] | 16 | Quantec QRS-style vintage room, low-SR band-limit, freeze (T05V) |
//! | `V-REV` | [`vintage`] | 8 | EMT 250-style chorused loops, front/rear outputs, transformer (T05V) |
//! | `VSS3` | [`vss3`] | 16 | TC VSS3-style: ER pattern generator + 4-band-decay tail (T05V) |
//!
//! Common mappings ([I] = inferred, no console measurement yet): `dcy` = mid-band T60 (s); `mult` = LF T60
//! multiplier below the model's crossover; `damp` = frequency above which the T60 falls to the model's
//! `hf_ratio` x `dcy`; `lc`/`hc` = 12 dB/oct output filters on the wet signal; `pdel` = pre-delay of the ER and
//! the tail; `shp`/`sprd` = Lexicon SHAPE/SPREAD (envelope and duration of the build-up, see core);
//! `diff` = input diffusion; `spin`/`mspd`/`mod` = LFO rate of the modulated delay lines; fader-type levels in
//! dB with -144 dB (= -oo) as off.

pub mod ambi;
pub mod chamber;
pub mod concert;
pub mod core;
pub mod hall;
pub mod room;
pub mod vintage;
pub mod vss3;

use self::core::{Character, CoreParams, ReverbCore};
use crate::model::{Model, ParamValue};

/// Largest number of WING parameters of a reverb model (CONCERT).
pub const MAX_PARAMS: usize = 17;

/// One WING parameter: snapshot key, range and WING-Edit default (engineering units).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ParamSpec {
    pub key: &'static str,
    pub min: f32,
    pub max: f32,
    pub default: f32,
}

const fn ps(key: &'static str, min: f32, max: f32, default: f32) -> ParamSpec {
    ParamSpec {
        key,
        min,
        max,
        default,
    }
}

/// Fader-type level (dB, -144 = -oo) to linear.
pub fn fader_gain(db: f32) -> f32 {
    if db <= -143.0 {
        0.0
    } else {
        10f32.powf(db.min(10.0) / 20.0)
    }
}

/// Static description of one reverb model.
pub struct ReverbSpec {
    pub id: &'static str,
    pub params: &'static [ParamSpec],
    pub character: Character,
    /// Raw values by slot (index = slot - 1) to core parameters.
    pub map: fn(&[f32; MAX_PARAMS]) -> CoreParams,
}

/// A reverb model on an `N`-line core.
pub struct Reverb<const N: usize> {
    spec: &'static ReverbSpec,
    raw: [f32; MAX_PARAMS],
    core: ReverbCore<N>,
    dirty: bool,
}

impl<const N: usize> Reverb<N> {
    pub fn new(spec: &'static ReverbSpec) -> Self {
        let mut r = Self {
            spec,
            raw: [0.0; MAX_PARAMS],
            core: ReverbCore::new(spec.character),
            dirty: true,
        };
        r.load_defaults();
        r.apply();
        r
    }

    pub fn spec(&self) -> &'static ReverbSpec {
        self.spec
    }

    /// Core parameters the current WING values map to.
    pub fn core_params(&self) -> CoreParams {
        (self.spec.map)(&self.raw)
    }

    pub fn core(&self) -> &ReverbCore<N> {
        &self.core
    }

    /// Set a parameter by snapshot key (convenience for tests/tools; not RT-critical but alloc-free).
    pub fn set_key(&mut self, key: &str, v: f32) {
        if let Some(i) = self.spec.params.iter().position(|p| p.key == key) {
            self.set_param(i as u16 + 1, ParamValue::Float(v));
        }
    }

    fn apply(&mut self) {
        self.dirty = false;
        let p = (self.spec.map)(&self.raw);
        self.core.set_params(&p);
    }
}

/// `HALL` (16 lines).
pub fn hall() -> Reverb<16> {
    Reverb::new(&hall::SPEC)
}
/// `ROOM` (8 lines).
pub fn room() -> Reverb<8> {
    Reverb::new(&room::SPEC)
}
/// `CHAMBER` (8 lines).
pub fn chamber() -> Reverb<8> {
    Reverb::new(&chamber::SPEC)
}
/// `CONCERT` (16 lines).
pub fn concert() -> Reverb<16> {
    Reverb::new(&concert::SPEC)
}
/// `AMBI` (8 lines).
pub fn ambi() -> Reverb<8> {
    Reverb::new(&ambi::SPEC)
}

/// `V-ROOM` (16 lines).
pub fn v_room() -> vintage::VintageReverb<16> {
    vintage::VintageReverb::new(&vintage::V_ROOM)
}
/// `V-REV` (8 lines).
pub fn v_rev() -> vintage::VintageReverb<8> {
    vintage::VintageReverb::new(&vintage::V_REV)
}
/// `VSS3`.
pub fn vss3() -> vss3::Vss3 {
    vss3::Vss3::new()
}

/// Factory for the FX catalog (`None` for ids that are not reverbs of this family).
pub fn create(id: &str) -> Option<Box<dyn Model>> {
    Some(match id {
        "HALL" => Box::new(hall()),
        "ROOM" => Box::new(room()),
        "CHAMBER" => Box::new(chamber()),
        "CONCERT" => Box::new(concert()),
        "AMBI" => Box::new(ambi()),
        "V-ROOM" => Box::new(v_room()),
        "V-REV" => Box::new(v_rev()),
        "VSS3" => Box::new(vss3()),
        _ => return None,
    })
}

impl<const N: usize> Model for Reverb<N> {
    fn id(&self) -> &'static str {
        self.spec.id
    }

    fn prepare(&mut self, sample_rate: f64, _max_block: usize) {
        self.apply();
        self.core.prepare(sample_rate);
    }

    fn process(&mut self, left: &mut [f32], right: &mut [f32], _sc: Option<(&[f32], &[f32])>) {
        if self.dirty {
            self.apply();
        }
        self.core.process(left, right);
    }

    fn reset(&mut self) {
        if self.dirty {
            self.apply();
        }
        self.core.reset();
    }

    fn set_param(&mut self, slot: u16, v: ParamValue) {
        let i = slot as usize;
        if i == 0 || i > self.spec.params.len() {
            return;
        }
        let s = &self.spec.params[i - 1];
        let x = v.as_f32();
        if !x.is_finite() {
            return;
        }
        self.raw[i - 1] = x.clamp(s.min, s.max);
        self.dirty = true;
    }

    fn param(&self, slot: u16) -> Option<f32> {
        let i = slot as usize;
        (i >= 1 && i <= self.spec.params.len()).then(|| self.raw[i - 1])
    }

    fn load_defaults(&mut self) {
        for (r, p) in self.raw.iter_mut().zip(self.spec.params) {
            *r = p.default;
        }
        self.dirty = true;
    }

    fn commit(&mut self) {
        if self.dirty {
            self.apply();
        }
    }
}

#[cfg(test)]
mod tests;
#[cfg(test)]
mod tests_vintage;
