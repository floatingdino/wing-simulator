//! Vintage reverbs (T05V): `V-ROOM` VINTAGE ROOM and `V-REV` VINTAGE REVERB (models.md §6).
//!
//! models.md names the emulated units as the Quantec Room Simulator QRS (`V-ROOM`, X32 "Vintage Room") and the
//! EMT 250 (`V-REV`, X32 "Vintage Reverb"); both are early-1980s digital reverbs in the Lexicon 224 lineage:
//! low internal sample rates, a handful of long modulated ("chorused") delay loops with in-loop allpasses, and
//! separate bass / mid / treble decay times. Both models run the shared FDN core ([`super::core`]) and add the
//! vintage stages around it:
//!
//! ```text
//!  in --> 4th-order Butterworth LP @ bandwidth --> x input gain (freeze mute) --> FDN core (allpass loops,
//!  ("anti-alias filter" of the low-SR unit)        (V-ROOM FREEZE)              chorused lines, 3-band T60)
//!     --> core HI CUT clamped to the bandwidth ("reconstruction filter") --> transformer (V-REV XFORMER)
//!     --> x output level (V-ROOM LEVEL) --> wet
//! ```
//!
//! The reduced bandwidth of a low-sample-rate unit is emulated without aliasing: the input is band-limited by a
//! steep low-pass at the unit's audio bandwidth and the output low-pass never opens above it, so the whole
//! wet signal lives in the band the original converters could carry (a decimate/hold would alias; this keeps
//! the dull, "smaller" top end without the grit). Bandwidths are [I] (no measurement): QRS 15 kHz, EMT 250
//! 12 kHz (its published response).
//!
//! `V-ROOM` (16 lines; slots per the WING schema, `fx_pardef_vintage_room`):
//!
//! | slot | key | range | mapping [I] |
//! |---|---|---|---|
//! | 1 | `pdel` | 0..200 ms | pre-delay |
//! | 2 | `size` | 1..50 | size scale `0.2 + 0.8 (size-1)/49` (lines 22..80 ms at 50) |
//! | 3 | `dcy` | 0.1..20 s | mid T60 |
//! | 4 | `dens` | 1..30 | diffusion `(dens-1)/29` (input diffusers + in-loop allpasses) |
//! | 5 | `erlvl` | 0..100 | ER level `erlvl/100` (ER bank) and first reflections at 0.6x that |
//! | 6 | `lmult` | 0.1..10 | LF T60 multiplier below 350 Hz |
//! | 7 | `hmult` | 0.1..10 | HF T60 multiplier above 2 kHz (*) |
//! | 8/9 | `lc`/`hc` | Hz | wet output filters (`hc` clamped to the 15 kHz bandwidth) |
//! | 10 | `frz` | off/on | FREEZE: T60 -> 3000 s in every band (loop gain capped just below 1), input muted |
//! | 11/12 | `erl`/`err` | 0..200 ms | first-reflection delay from the input, left / right |
//! | 13 | `add` | off/on | while frozen, keep feeding the input into the frozen room (layering) |
//! | 14 | `lvl` | -6..6 dB | output level |
//!
//! (*) An HF T60 longer than the mid T60 (`hmult` > 1) is capped in the top octaves by the interpolation loss
//! of the modulated reads (core): within 10 % up to ~6 kHz in a large room, shorter above.
//!
//! `V-REV` (8 lines, long chorused loops; `fx_pardef_vintage_reverb`):
//!
//! | slot | key | range | mapping [I] |
//! |---|---|---|---|
//! | 1 | `pdel` | 0..120 ms | pre-delay |
//! | 2 | `dcy` | 0.4..4.5 s | mid T60 |
//! | 3 | `lmult` | 0.5..2 | LF T60 multiplier below 500 Hz |
//! | 4 | `hmult` | 0.25..0.67 | HF T60 multiplier above 3 kHz |
//! | 5 | `mod` | 0..100 | MOD SPD: chorus LFO 0.1..1.0 Hz of the loop delays (0.3 ms depth) |
//! | 6/7 | `lc`/`hc` | Hz | wet output filters (`hc` clamped to the 12 kHz bandwidth) |
//! | 8 | `out` | FRONT/REAR | output tap set: front = core taps + light ER, rear = orthogonal taps, no ER |
//! | 9 | `trans` | off/on | output transformer: `tanh(1.8 x)/1.8` soft saturation (unity small-signal gain) |
//!
//! Bool / enum slots switch at 0.5 and every switch is smoothed (gains 20 ms, core values by its glides).

use wing_dsp::smooth::OnePoleSmoother;
use wing_dsp::svf::{Svf, SvfMode};

use super::core::{Character, CoreParams, Echo, ReverbCore};
use super::{ps, ParamSpec};
use crate::model::{Model, ParamValue};

/// Largest number of WING parameters of a vintage model (V-ROOM).
pub const V_PARAMS: usize = 14;

/// Smoothing time of the wrapper gains (s).
const GAIN_TAU_S: f64 = 0.02;
/// Transformer drive.
const XFORMER_DRIVE: f32 = 1.8;
/// Freeze T60 (s); the core caps the loop gain just below 1.
pub const FREEZE_T60: f32 = 3000.0;

/// Wrapper-level parameters a vintage model maps its WING values to.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct VintageParams {
    pub core: CoreParams,
    /// Input gain into the reverb (0 = frozen without ADD).
    pub input: f32,
    /// Output gain (linear).
    pub output: f32,
    /// Transformer saturation amount 0..1.
    pub xformer: f32,
}

/// Static description of one vintage model.
pub struct VintageSpec {
    pub id: &'static str,
    pub params: &'static [ParamSpec],
    pub character: Character,
    /// Audio bandwidth of the emulated low-sample-rate unit (Hz).
    pub bandwidth_hz: f32,
    pub map: fn(&[f32; V_PARAMS]) -> VintageParams,
}

/// A vintage reverb on an `N`-line core.
pub struct VintageReverb<const N: usize> {
    spec: &'static VintageSpec,
    raw: [f32; V_PARAMS],
    core: ReverbCore<N>,
    vp: VintageParams,
    dirty: bool,
    fs: f64,
    /// 4th-order Butterworth band-limit (two SVF sections) per channel.
    bw: [[Svf; 2]; 2],
    in_g: OnePoleSmoother,
    out_g: OnePoleSmoother,
    xf: OnePoleSmoother,
}

impl<const N: usize> VintageReverb<N> {
    pub fn new(spec: &'static VintageSpec) -> Self {
        let mut r = Self {
            spec,
            raw: [0.0; V_PARAMS],
            core: ReverbCore::new(spec.character),
            vp: (spec.map)(&[0.0; V_PARAMS]),
            dirty: true,
            fs: 48_000.0,
            bw: [[Svf::default(); 2]; 2],
            in_g: OnePoleSmoother::new(1.0, GAIN_TAU_S, 48_000.0),
            out_g: OnePoleSmoother::new(1.0, GAIN_TAU_S, 48_000.0),
            xf: OnePoleSmoother::new(0.0, GAIN_TAU_S, 48_000.0),
        };
        r.load_defaults();
        r.apply();
        r
    }

    pub fn spec(&self) -> &'static VintageSpec {
        self.spec
    }

    /// Mapped parameters of the current WING values.
    pub fn vintage_params(&self) -> VintageParams {
        (self.spec.map)(&self.raw)
    }

    pub fn core(&self) -> &ReverbCore<N> {
        &self.core
    }

    /// Set a parameter by snapshot key (tests/tools; alloc-free).
    pub fn set_key(&mut self, key: &str, v: f32) {
        if let Some(i) = self.spec.params.iter().position(|p| p.key == key) {
            self.set_param(i as u16 + 1, ParamValue::Float(v));
        }
    }

    fn apply(&mut self) {
        self.dirty = false;
        let mut vp = (self.spec.map)(&self.raw);
        vp.core.hc_hz = vp.core.hc_hz.min(self.spec.bandwidth_hz);
        self.vp = vp;
        self.core.set_params(&vp.core);
        self.in_g.set_target(vp.input);
        self.out_g.set_target(vp.output);
        self.xf.set_target(vp.xformer);
    }

    fn snap(&mut self) {
        self.in_g.reset(self.vp.input);
        self.out_g.reset(self.vp.output);
        self.xf.reset(self.vp.xformer);
    }
}

#[inline(always)]
fn xformer(x: f32) -> f32 {
    (XFORMER_DRIVE * x).tanh() * (1.0 / XFORMER_DRIVE)
}

impl<const N: usize> Model for VintageReverb<N> {
    fn id(&self) -> &'static str {
        self.spec.id
    }

    fn prepare(&mut self, sample_rate: f64, _max_block: usize) {
        self.fs = sample_rate;
        let f = (self.spec.bandwidth_hz as f64).min(0.45 * sample_rate);
        // Butterworth 4th order: Q = 1/(2 cos(pi/8)), 1/(2 cos(3 pi/8))
        for c in 0..2 {
            self.bw[c][0] = Svf::new(SvfMode::Lowpass, sample_rate, f, 0.541_196, 0.0);
            self.bw[c][1] = Svf::new(SvfMode::Lowpass, sample_rate, f, 1.306_563, 0.0);
        }
        for s in [&mut self.in_g, &mut self.out_g, &mut self.xf] {
            s.set_time(GAIN_TAU_S, sample_rate);
        }
        self.apply();
        self.core.prepare(sample_rate);
        self.snap();
    }

    fn process(&mut self, left: &mut [f32], right: &mut [f32], _sc: Option<(&[f32], &[f32])>) {
        if self.dirty {
            self.apply();
        }
        let len = left.len().min(right.len());
        let (left, right) = (&mut left[..len], &mut right[..len]);
        // ---- band-limit + input gain (in place)
        for n in 0..len {
            let g = self.in_g.tick();
            let [[l0, l1], [r0, r1]] = &mut self.bw;
            left[n] = g * l1.process(l0.process(left[n]));
            right[n] = g * r1.process(r0.process(right[n]));
        }
        // ---- FDN core: dry -> wet
        self.core.process(left, right);
        // ---- transformer + output level
        let xf_on = self.xf.value() > 1e-6 || self.xf.target() > 0.0;
        for n in 0..len {
            let o = self.out_g.tick();
            let (mut l, mut r) = (left[n], right[n]);
            if xf_on {
                let x = self.xf.tick();
                l += x * (xformer(l) - l);
                r += x * (xformer(r) - r);
            }
            left[n] = o * l;
            right[n] = o * r;
        }
        for side in &mut self.bw {
            for s in side.iter_mut() {
                s.flush_denormals();
            }
        }
    }

    fn reset(&mut self) {
        if self.dirty {
            self.apply();
        }
        for side in &mut self.bw {
            for s in side.iter_mut() {
                s.reset();
            }
        }
        self.core.reset();
        self.snap();
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

// ------------------------------------------------------------------------------------------------ V-ROOM

/// `V-ROOM` parameters (schema slot order).
pub const V_ROOM_PARAMS: [ParamSpec; 14] = [
    ps("pdel", 0.0, 200.0, 20.0),
    ps("size", 1.0, 50.0, 15.0),
    ps("dcy", 0.1, 20.0, 1.14),
    ps("dens", 1.0, 30.0, 30.0),
    ps("erlvl", 0.0, 100.0, 30.0),
    ps("lmult", 0.1, 10.0, 1.1),
    ps("hmult", 0.1, 10.0, 0.69),
    ps("lc", 20.0, 400.0, 79.0),
    ps("hc", 200.0, 20_000.0, 9600.0),
    ps("frz", 0.0, 1.0, 0.0),
    ps("erl", 0.0, 200.0, 28.0),
    ps("err", 0.0, 200.0, 34.0),
    ps("add", 0.0, 1.0, 0.0),
    ps("lvl", -6.0, 6.0, 0.0),
];

/// `V-ROOM` HF multiplier corner (Hz).
pub const V_ROOM_DAMP_HZ: f32 = 2000.0;

pub const V_ROOM_CHARACTER: Character = Character {
    line_ms: (22.0, 80.0),
    max_scale: 1.0,
    loop_ap_ms: (1.1, 4.3),
    loop_ap_gain: 0.62,
    diffuser_ms: [0.83, 1.57, 2.71, 4.13],
    diffuser_gain: 0.75,
    xover_hz: 350.0,
    hf_ratio: 0.69,
    mod_depth_ms: 0.08,
    er_window_ms: (4.0, 60.0),
    max_echo_s: 0.2,
    echo_feedback: false,
    tail_gain: 1.0,
};

fn map_v_room(r: &[f32; V_PARAMS]) -> VintageParams {
    let frz = r[9] > 0.5;
    let add = r[12] > 0.5;
    let er = r[4] / 100.0;
    let refl = |ms: f32| Echo {
        time_ms: ms,
        feedback: 0.0,
        level: 0.6 * er,
    };
    VintageParams {
        core: CoreParams {
            pre_ms: r[0],
            scale: 0.2 + 0.8 * (r[1] - 1.0) / 49.0,
            decay_s: if frz { FREEZE_T60 } else { r[2] },
            bass_mult: if frz { 1.0 } else { r[5] },
            hf_ratio: Some(if frz { 1.0 } else { r[6] }),
            damp_hz: V_ROOM_DAMP_HZ,
            lc_hz: r[7],
            hc_hz: r[8],
            shape: 0.3,
            spread: 0.5,
            diffusion: (r[3] - 1.0) / 29.0,
            // the QRS is essentially unmodulated: a slow, shallow drift only breaks up metallic modes
            mod_rate_hz: 0.25,
            mod_depth: 1.0,
            er_level: er,
            tail_level: 1.0,
            echo: [refl(r[10]), refl(r[11])],
            echo_to_tail: 0.5,
            ..CoreParams::default()
        },
        input: if frz && !add { 0.0 } else { 1.0 },
        output: 10f32.powf(r[13] / 20.0),
        xformer: 0.0,
    }
}

pub static V_ROOM: VintageSpec = VintageSpec {
    id: "V-ROOM",
    params: &V_ROOM_PARAMS,
    character: V_ROOM_CHARACTER,
    bandwidth_hz: 15_000.0,
    map: map_v_room,
};

// ------------------------------------------------------------------------------------------------- V-REV

/// `V-REV` parameters (schema slot order).
pub const V_REV_PARAMS: [ParamSpec; 9] = [
    ps("pdel", 0.0, 120.0, 20.0),
    ps("dcy", 0.4, 4.5, 1.5),
    ps("lmult", 0.5, 2.0, 1.0),
    ps("hmult", 0.25, 0.67, 0.67),
    ps("mod", 0.0, 100.0, 100.0),
    ps("lc", 20.0, 400.0, 75.0),
    ps("hc", 5000.0, 20_000.0, 12_000.0),
    ps("out", 0.0, 1.0, 0.0),
    ps("trans", 0.0, 1.0, 0.0),
];

/// `V-REV` HF multiplier corner (Hz).
pub const V_REV_DAMP_HZ: f32 = 3000.0;

pub const V_REV_CHARACTER: Character = Character {
    line_ms: (26.0, 88.0),
    max_scale: 1.0,
    loop_ap_ms: (2.3, 8.9),
    loop_ap_gain: 0.65,
    diffuser_ms: [1.21, 2.37, 3.89, 5.71],
    diffuser_gain: 0.7,
    xover_hz: 500.0,
    hf_ratio: 0.67,
    mod_depth_ms: 0.3,
    er_window_ms: (6.0, 90.0),
    max_echo_s: 0.0,
    echo_feedback: false,
    tail_gain: 1.1,
};

fn map_v_rev(r: &[f32; V_PARAMS]) -> VintageParams {
    let rear = if r[7] > 0.5 { 1.0 } else { 0.0 };
    VintageParams {
        core: CoreParams {
            pre_ms: r[0],
            scale: 1.0,
            decay_s: r[1],
            bass_mult: r[2],
            hf_ratio: Some(r[3]),
            damp_hz: V_REV_DAMP_HZ,
            lc_hz: r[5],
            hc_hz: r[6],
            shape: 0.4,
            spread: 0.4,
            diffusion: 0.75,
            mod_rate_hz: 0.1 + 0.9 * r[4] / 100.0,
            mod_depth: 1.0,
            er_level: 0.25 * (1.0 - rear),
            tail_level: 1.0,
            rear,
            ..CoreParams::default()
        },
        input: 1.0,
        output: 1.0,
        xformer: if r[8] > 0.5 { 1.0 } else { 0.0 },
    }
}

pub static V_REV: VintageSpec = VintageSpec {
    id: "V-REV",
    params: &V_REV_PARAMS,
    character: V_REV_CHARACTER,
    bandwidth_hz: 12_000.0,
    map: map_v_rev,
};
