//! `VSS3` (VSS3 REVERB, `fx_pardef_vss3_reverb`): TC Electronic System 6000 VSS3-style reverb (models.md §6,
//! T05V). Two independent generators fed from the dry input, as on the TC unit:
//!
//! ```text
//!         +--> ER delay ring --> 2 x 20-tap early-reflection pattern (per ER TYPE x SIZE x POS, crossfaded on
//!   in ---+    change) --> LO CUT --> COLOUR tilt --> distance LP --> BAL --> ER LVL ---------------+
//!         |                                                                                           +--> wet
//!         +--> 16-line FDN core (REV PDLY, 4-band decay: LO / LOMID / HIMID / HI DCY multipliers at  --+
//!              LO / MID / HI X/O, chorused lines per MOD TYPE / RATE / WID, HI CUT) --> LO SHV damp
//!              --> HI SOFT --> WIDTH (M/S) --> BAL --> REV LVL
//! ```
//!
//! Early reflections: each of the 24 ER types is a small parameter set (pattern length, tap count, envelope,
//! clustering, first-reflection offset, seed) from which a deterministic tap table is generated per side (pseudo
//! random positions/signs, exponential envelope, energy-normalised); SML / MED / LRG scale the pattern length by
//! 0.6 / 1 / 1.6 and DIST moves the pattern back (12 % gap), flattens the envelope and darkens it (7 kHz LP).
//! The tables are built without allocation and a type / size / position change crossfades the old and the new
//! table over [`super::core::XFADE_S`]. The table data are our own [I] (no TC data available).
//!
//! | slot | key | range | mapping [I] |
//! |---|---|---|---|
//! | 1 | `preset` | 109 presets | UI only: the preset library is applied by the console (`load`), no DSP effect |
//! | 2 | `load` | off/on | UI action, no DSP effect |
//! | 3 | `erpdly` | 0..100 ms | ER pre-delay (glided) |
//! | 4 | `ertype` | 24 types | ER pattern (table [`ER_TYPES`]) |
//! | 5 | `ersize` | SML/MED/LRG | pattern length x0.6 / x1 / x1.6 |
//! | 6 | `erpos` | NEAR/DIST | DIST: 12 % gap, half envelope slope, 7 kHz LP |
//! | 7 | `erbal` | -100..100 | ER balance (> 0 attenuates L, < 0 attenuates R) |
//! | 8 | `erlc` | 20..400 Hz | ER 12 dB/oct low cut |
//! | 9 | `ercol` | -40..40 | ER colour tilt: -+6 dB below 600 Hz / +-6 dB above 2.5 kHz at +-40 |
//! | 10 | `erlvl` | fader dB | ER level |
//! | 11 | `rvtype` | SMOOTH..ALIVE2 | tail flavour: size, build-up shape/spread, diffusion, mod depth ([`RV_TYPES`]) |
//! | 12 | `rvwid` | NARROW..X-WIDE | tail M/S width 0.3 / 0.7 / 1.0 / 1.4 |
//! | 13 | `rvpdly` | 0..200 ms | tail pre-delay |
//! | 14 | `dcy` | 0.1..20 s | T60 reference (multiplied by the band multipliers) |
//! | 15 | `diff` | -50..50 | diffusion offset `diff/100` on the type's diffusion |
//! | 16 | `rvbal` | -100..100 | tail balance |
//! | 17 | `rvlvl` | fader dB | tail level |
//! | 18-21 | `ldcy`/`lmdcy`/`hmdcy`/`hdcy` | 0.1..2.5 | T60 = `dcy` x multiplier in the 4 bands |
//! | 22 | `hsoft` | -50..50 | HI SOFT: tail cut above 3 kHz, 0 dB at -50 .. -9 dB at +50 |
//! | 23-25 | `lxo`/`mxo`/`hxo` | Hz | decay-band crossovers (core keeps them ordered) |
//! | 26 | `lshv` | 20..200 Hz | LO DAMP shelf frequency |
//! | 27 | `lsdmp` | -18..0 dB | LO DAMP gain below `lshv` on the tail (schema lists it as 0 .. -18) |
//! | 28 | `hcut` | 20..20k Hz | tail 12 dB/oct high cut (core floor 100 Hz) |
//! | 29 | `mtype` | A..F | LFO depth x 0.3 / 0.6 / 1 / 1.4 / 2 / 2.6 and rate x 1.3 / 1.15 / 1 / 0.9 / 0.8 / 0.7 |
//! | 30 | `mrate` | -100..100 | LFO rate `0.6 x 4^(mrate/100)` Hz (0.15..2.4) |
//! | 31 | `mwid` | 0..200 | LFO depth `mwid/100` (x 0.2 ms) |
//! | 32 | `view` | off/on | UI action, no DSP effect |
//!
//! RT: all buffers are allocated in `prepare` (ER ring sized for the longest pattern + one block); blocks are
//! processed in chunks of at most `max_block`. Parameter changes glide: levels and shelf gains per sample (the
//! shelves are `x + (g - 1) band(x)`, so gains never step filter coefficients), filter frequencies per block.

use wing_dsp::smooth::OnePoleSmoother;
use wing_dsp::svf::{Svf, SvfMode};

use super::core::{Character, CoreParams, ReverbCore, XFADE_S};
use super::{fader_gain, ps, ParamSpec};
use crate::model::{Model, ParamValue};

/// Number of WING parameters.
pub const PARAMS_LEN: usize = 32;
/// Early-reflection taps per side.
pub const ER_MAX: usize = 20;
/// Longest ER pre-delay (ms).
const ER_PDLY_MAX_MS: f32 = 100.0;
/// Size multipliers SML / MED / LRG.
pub const ER_SIZE: [f32; 3] = [0.6, 1.0, 1.6];
/// DIST: gap before the pattern (fraction of its length).
const DIST_GAP: f32 = 0.12;
/// Smoothing of gains / filter values (s).
const TAU_S: f64 = 0.03;

pub const PARAMS: [ParamSpec; PARAMS_LEN] = [
    ps("preset", 0.0, 108.0, 29.0),
    ps("load", 0.0, 1.0, 0.0),
    ps("erpdly", 0.0, 100.0, 11.0),
    ps("ertype", 0.0, 23.0, 2.0),
    ps("ersize", 0.0, 2.0, 2.0),
    ps("erpos", 0.0, 1.0, 0.0),
    ps("erbal", -100.0, 100.0, 0.0),
    ps("erlc", 20.0, 400.0, 20.0),
    ps("ercol", -40.0, 40.0, 40.0),
    ps("erlvl", -144.0, 10.0, -6.0),
    ps("rvtype", 0.0, 5.0, 0.0),
    ps("rvwid", 0.0, 3.0, 3.0),
    ps("rvpdly", 0.0, 200.0, 0.0),
    ps("dcy", 0.1, 20.0, 2.88),
    ps("diff", -50.0, 50.0, 6.0),
    ps("rvbal", -100.0, 100.0, 0.0),
    ps("rvlvl", -144.0, 10.0, -6.0),
    ps("ldcy", 0.1, 2.5, 1.0),
    ps("lmdcy", 0.1, 2.5, 1.0),
    ps("hmdcy", 0.1, 2.5, 1.0),
    ps("hdcy", 0.1, 2.5, 1.0),
    ps("hsoft", -50.0, 50.0, -50.0),
    ps("lxo", 20.0, 500.0, 92.0),
    ps("mxo", 200.0, 2000.0, 774.0),
    ps("hxo", 500.0, 20_000.0, 3800.0),
    ps("lshv", 20.0, 200.0, 178.0),
    ps("lsdmp", -18.0, 0.0, 0.0),
    ps("hcut", 20.0, 20_000.0, 6500.0),
    ps("mtype", 0.0, 5.0, 2.0),
    ps("mrate", -100.0, 100.0, -30.0),
    ps("mwid", 0.0, 200.0, 156.0),
    ps("view", 0.0, 1.0, 0.0),
];

/// Parameter slots (1-based) by name, for readability.
pub mod slot {
    pub const ERPDLY: usize = 3;
    pub const ERTYPE: usize = 4;
    pub const ERSIZE: usize = 5;
    pub const ERPOS: usize = 6;
    pub const ERBAL: usize = 7;
    pub const ERLC: usize = 8;
    pub const ERCOL: usize = 9;
    pub const ERLVL: usize = 10;
    pub const RVTYPE: usize = 11;
    pub const RVWID: usize = 12;
    pub const RVPDLY: usize = 13;
    pub const DCY: usize = 14;
    pub const DIFF: usize = 15;
    pub const RVBAL: usize = 16;
    pub const RVLVL: usize = 17;
    pub const LDCY: usize = 18;
    pub const LMDCY: usize = 19;
    pub const HMDCY: usize = 20;
    pub const HDCY: usize = 21;
    pub const HSOFT: usize = 22;
    pub const LXO: usize = 23;
    pub const MXO: usize = 24;
    pub const HXO: usize = 25;
    pub const LSHV: usize = 26;
    pub const LSDMP: usize = 27;
    pub const HCUT: usize = 28;
    pub const MTYPE: usize = 29;
    pub const MRATE: usize = 30;
    pub const MWID: usize = 31;
}

/// One early-reflection pattern type.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ErType {
    pub name: &'static str,
    /// Pattern length at MED size (ms).
    pub len_ms: f32,
    /// Number of taps per side (<= [`ER_MAX`]).
    pub taps: usize,
    /// Envelope `exp(-alpha u)` over the pattern (negative = building up).
    pub alpha: f32,
    /// Position clustering exponent (< 1 late-heavy spacing ... > 1 front-clustered).
    pub cluster: f32,
    /// First reflection offset (ms, before the pattern).
    pub first_ms: f32,
}

const fn er(
    name: &'static str,
    len_ms: f32,
    taps: usize,
    alpha: f32,
    cluster: f32,
    first_ms: f32,
) -> ErType {
    ErType {
        name,
        len_ms,
        taps,
        alpha,
        cluster,
        first_ms,
    }
}

/// The 24 ER types (WING `ertype` order) [I].
pub const ER_TYPES: [ErType; 24] = [
    er("ROYAL", 90.0, 18, 2.0, 1.0, 6.0),
    er("THEATRE", 110.0, 18, 2.2, 1.1, 8.0),
    er("CHURCH", 150.0, 20, 1.5, 1.2, 12.0),
    er("GAS", 160.0, 16, 1.2, 1.3, 15.0),
    er("CONCERT", 120.0, 20, 1.8, 1.0, 10.0),
    er("ROYAL2", 95.0, 18, 2.4, 0.9, 5.0),
    er("V1-NEAR", 40.0, 12, 3.0, 0.7, 1.0),
    er("V2-HARD", 60.0, 14, 1.0, 1.0, 3.0),
    er("V3-SPREAD", 120.0, 20, 1.5, 1.4, 4.0),
    er("V4-BUILD", 80.0, 16, -1.5, 1.0, 2.0),
    er("V5-RANDOM", 100.0, 20, 0.5, 1.0, 2.0),
    er("SLAP", 70.0, 4, 0.5, 1.0, 20.0),
    er("CAR", 8.0, 10, 3.0, 0.8, 0.5),
    er("PHONEBTH", 6.0, 10, 3.0, 0.8, 0.3),
    er("BATHROOM", 18.0, 16, 2.0, 0.9, 1.0),
    er("CONFRM9", 25.0, 14, 2.4, 0.9, 1.5),
    er("CONFRM30", 45.0, 16, 2.2, 1.0, 2.5),
    er("GARAGE", 55.0, 16, 1.5, 1.0, 3.0),
    er("SWIMSTDM", 85.0, 18, 1.4, 1.1, 5.0),
    er("AIRPORT", 180.0, 14, 1.0, 1.4, 18.0),
    er("STREET", 110.0, 8, 1.6, 1.2, 9.0),
    er("ALLEY", 75.0, 12, 1.2, 1.0, 4.0),
    er("PIAZA", 140.0, 12, 1.4, 1.3, 14.0),
    er("FOREST", 130.0, 20, 2.5, 1.0, 7.0),
];

/// Reverb type: (size scale, shape, spread, diffusion, mod depth multiplier).
pub const RV_TYPES: [(f32, f32, f32, f32, f32); 6] = [
    (1.0, 0.35, 0.6, 0.8, 1.0),   // SMOOTH
    (0.9, 0.3, 0.5, 0.7, 0.6),    // NATURAL
    (1.1, 0.25, 0.4, 0.6, 1.4),   // ALIVE
    (0.6, 0.1, 0.2, 0.7, 0.8),    // FAST
    (1.2, 0.4, 0.8, 0.75, 1.2),   // X-WIDE
    (1.05, 0.45, 0.7, 0.65, 1.8), // ALIVE2
];
/// Tail width per `rvwid`.
pub const RV_WIDTH: [f32; 4] = [0.3, 0.7, 1.0, 1.4];
/// MOD TYPE (depth, rate) multipliers.
pub const MOD_TYPES: [(f32, f32); 6] = [
    (0.3, 1.3),
    (0.6, 1.15),
    (1.0, 1.0),
    (1.4, 0.9),
    (2.0, 0.8),
    (2.6, 0.7),
];

pub const CHARACTER: Character = Character {
    line_ms: (28.0, 96.0),
    max_scale: 1.2,
    loop_ap_ms: (1.7, 6.7),
    loop_ap_gain: 0.6,
    diffuser_ms: [1.13, 2.29, 3.67, 5.53],
    diffuser_gain: 0.72,
    xover_hz: 92.0,
    hf_ratio: 1.0,
    mod_depth_ms: 0.2,
    er_window_ms: (6.0, 120.0),
    max_echo_s: 0.0,
    echo_feedback: false,
    tail_gain: 1.0,
};

/// Uniform hash in [0, 1).
fn rnd(seed: u32, i: u32) -> f32 {
    let mut x = seed.wrapping_mul(0x9E37_79B9) ^ i.wrapping_mul(0x85EB_CA6B);
    x ^= x >> 15;
    x = x.wrapping_mul(0x2C1B_3C6D);
    x ^= x >> 12;
    x = x.wrapping_mul(0x297A_2D39);
    x ^= x >> 15;
    (x >> 8) as f32 / (1u32 << 24) as f32
}

/// One side's tap table: delays (ms from the ER pre-delay) and gains.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ErTable {
    pub ms: [f32; ER_MAX],
    pub gain: [f32; ER_MAX],
}

/// Build the tap table of `ty` x `size` x `dist` for `side` (0 = L, 1 = R). Alloc-free, deterministic.
pub fn er_table(ty: usize, size: usize, dist: bool, side: usize) -> ErTable {
    let t = &ER_TYPES[ty.min(ER_TYPES.len() - 1)];
    let len = t.len_ms * ER_SIZE[size.min(2)];
    let first = t.first_ms * ER_SIZE[size.min(2)];
    let gap = if dist { DIST_GAP } else { 0.0 };
    let alpha = if dist { 0.5 * t.alpha } else { t.alpha };
    let seed = 0x5EED_0000 ^ ((ty as u32) << 4) ^ side as u32;
    let mut tab = ErTable {
        ms: [0.0; ER_MAX],
        gain: [0.0; ER_MAX],
    };
    let n = t.taps.min(ER_MAX);
    let mut e = 0.0;
    for k in 0..n {
        let jit = 0.8 * (rnd(seed, 2 * k as u32) - 0.5);
        let u = ((k as f32 + 0.5 + jit) / n as f32)
            .clamp(0.0, 1.0)
            .powf(t.cluster);
        let sgn = if rnd(seed, 2 * k as u32 + 1) < 0.5 {
            -1.0
        } else {
            1.0
        };
        let mag = (-alpha * u).exp() * (0.6 + 0.4 * rnd(seed ^ 0xA5A5, k as u32));
        tab.ms[k] = first + len * (gap + (1.0 - gap) * u);
        tab.gain[k] = sgn * mag;
        e += mag * mag;
    }
    let norm = if e > 0.0 { 1.0 / e.sqrt() } else { 0.0 };
    for g in tab.gain.iter_mut() {
        *g *= norm;
    }
    tab
}

/// Longest ER tap (ms) over every type / size / position.
fn er_max_ms() -> f32 {
    ER_TYPES
        .iter()
        .map(|t| (t.first_ms + t.len_ms) * ER_SIZE[2])
        .fold(0.0, f32::max)
}

/// Core parameters of the tail for raw values `r` (index = slot - 1).
pub fn core_params(r: &[f32; PARAMS_LEN]) -> CoreParams {
    let g = |s: usize| r[s - 1];
    let (scale, shape, spread, diff, depth) = RV_TYPES[(g(slot::RVTYPE).round() as usize).min(5)];
    let (md, mr) = MOD_TYPES[(g(slot::MTYPE).round() as usize).min(5)];
    let lm = g(slot::LMDCY);
    CoreParams {
        pre_ms: g(slot::RVPDLY),
        scale,
        decay_s: g(slot::DCY) * lm,
        bass_mult: g(slot::LDCY) / lm,
        xover_hz: g(slot::LXO),
        mid_split: Some((g(slot::MXO), g(slot::HMDCY) / lm)),
        damp_hz: g(slot::HXO),
        hf_ratio: Some(g(slot::HDCY) / lm),
        lc_hz: 20.0,
        hc_hz: g(slot::HCUT),
        shape,
        spread,
        diffusion: (diff + g(slot::DIFF) / 100.0).clamp(0.0, 1.0),
        mod_rate_hz: 0.6 * 4f32.powf(g(slot::MRATE) / 100.0) * mr,
        mod_depth: md * depth * g(slot::MWID) / 100.0,
        er_level: 0.0,
        tail_level: fader_gain(g(slot::RVLVL)),
        ..CoreParams::default()
    }
}

/// Balance gains (L, R) for `bal` in -100..100.
fn balance(bal: f32) -> (f32, f32) {
    let b = (bal / 100.0).clamp(-1.0, 1.0);
    ((1.0 - b).min(1.0), (1.0 + b).min(1.0))
}

/// The VSS3 model.
pub struct Vss3 {
    raw: [f32; PARAMS_LEN],
    core: ReverbCore<16>,
    dirty: bool,
    fs: f32,
    max_block: usize,
    // ---- ER generator
    ring: [Vec<f32>; 2],
    mask: usize,
    w: usize,
    /// Two tap-table sets (per side); `act` is playing, the other is the crossfade target.
    tab: [[ErTable; 2]; 2],
    act: usize,
    xf: f32,
    er_key: [usize; 3],
    er_pdly: OnePoleSmoother,
    er_lvl: [OnePoleSmoother; 2],
    er_lc: [Svf; 2],
    /// Colour tilt bands (2nd-order LP @ 600 Hz / HP @ 2.5 kHz), added back with gain `g - 1`.
    er_lo: [Svf; 2],
    er_hi: [Svf; 2],
    er_lp: [Svf; 2],
    /// Block-rate filter frequencies: erlc, distance LP, lshv.
    fv: [OnePoleSmoother; 3],
    fkey: [f32; 3],
    /// Per-sample linear gains: colour low, colour high, LO DAMP, HI SOFT (shelves as `x + (g - 1) band(x)`,
    /// so a gain change never steps a filter coefficient).
    gs: [OnePoleSmoother; 4],
    // ---- tail post
    /// LO DAMP band (LP @ lshv) and HI SOFT band (HP @ 3 kHz).
    lo_damp: [Svf; 2],
    hsoft: [Svf; 2],
    width: OnePoleSmoother,
    rv_bal: [OnePoleSmoother; 2],
}

impl Default for Vss3 {
    fn default() -> Self {
        Self::new()
    }
}

impl Vss3 {
    pub fn new() -> Self {
        let tab0 = [er_table(2, 2, false, 0), er_table(2, 2, false, 1)];
        let sm = |v: f32| OnePoleSmoother::new(v, TAU_S, 48_000.0);
        let mut m = Self {
            raw: [0.0; PARAMS_LEN],
            core: ReverbCore::new(CHARACTER),
            dirty: true,
            fs: 48_000.0,
            max_block: 1,
            ring: [vec![0.0; 1], vec![0.0; 1]],
            mask: 0,
            w: 0,
            tab: [tab0, tab0],
            act: 0,
            xf: 0.0,
            er_key: [2, 2, 0],
            er_pdly: sm(0.0),
            er_lvl: [sm(0.0), sm(0.0)],
            er_lc: [Svf::default(); 2],
            er_lo: [Svf::default(); 2],
            er_hi: [Svf::default(); 2],
            er_lp: [Svf::default(); 2],
            fv: [sm(20.0), sm(20_000.0), sm(100.0)],
            fkey: [f32::NAN; 3],
            gs: [sm(1.0); 4],
            lo_damp: [Svf::default(); 2],
            hsoft: [Svf::default(); 2],
            width: sm(1.0),
            rv_bal: [sm(1.0), sm(1.0)],
        };
        m.load_defaults();
        m.apply();
        m
    }

    pub fn core(&self) -> &ReverbCore<16> {
        &self.core
    }

    /// Set a parameter by snapshot key (tests/tools; alloc-free).
    pub fn set_key(&mut self, key: &str, v: f32) {
        if let Some(i) = PARAMS.iter().position(|p| p.key == key) {
            self.set_param(i as u16 + 1, ParamValue::Float(v));
        }
    }

    fn g(&self, s: usize) -> f32 {
        self.raw[s - 1]
    }

    fn er_key(&self) -> [usize; 3] {
        [
            self.g(slot::ERTYPE).round() as usize,
            self.g(slot::ERSIZE).round() as usize,
            (self.g(slot::ERPOS) > 0.5) as usize,
        ]
    }

    /// Stage targets from the raw values.
    fn apply(&mut self) {
        self.dirty = false;
        self.core.set_params(&core_params(&self.raw));
        let ms = self.fs * 0.001;
        self.er_pdly.set_target(self.g(slot::ERPDLY) * ms);
        let lvl = fader_gain(self.g(slot::ERLVL));
        let (bl, br) = balance(self.g(slot::ERBAL));
        self.er_lvl[0].set_target(lvl * bl);
        self.er_lvl[1].set_target(lvl * br);
        let dist = self.g(slot::ERPOS) > 0.5;
        let fv = [
            self.g(slot::ERLC),
            if dist { 7000.0 } else { 20_000.0 },
            self.g(slot::LSHV),
        ];
        for (s, v) in self.fv.iter_mut().zip(fv) {
            s.set_target(v);
        }
        let db = |d: f32| 10f32.powf(d / 20.0);
        let col = self.g(slot::ERCOL) / 40.0 * 6.0;
        let gs = [
            db(-col),
            db(col),
            db(self.g(slot::LSDMP)),
            db(-9.0 * (self.g(slot::HSOFT) + 50.0) / 100.0),
        ];
        for (s, v) in self.gs.iter_mut().zip(gs) {
            s.set_target(v);
        }
        self.width
            .set_target(RV_WIDTH[(self.g(slot::RVWID).round() as usize).min(3)]);
        let (bl, br) = balance(self.g(slot::RVBAL));
        self.rv_bal[0].set_target(bl);
        self.rv_bal[1].set_target(br);
    }

    fn snap(&mut self) {
        for s in self
            .fv
            .iter_mut()
            .chain(self.gs.iter_mut())
            .chain(self.er_lvl.iter_mut())
            .chain(self.rv_bal.iter_mut())
            .chain([&mut self.er_pdly, &mut self.width])
        {
            let t = s.target();
            s.reset(t);
        }
        let k = self.er_key();
        self.er_key = k;
        for c in 0..2 {
            self.tab[self.act][c] = er_table(k[0], k[1], k[2] == 1, c);
        }
        self.xf = 0.0;
        self.fkey = [f32::NAN; 3];
        self.update_filters();
    }

    fn update_filters(&mut self) {
        let fv: [f32; 3] = core::array::from_fn(|i| self.fv[i].value());
        if fv == self.fkey {
            return;
        }
        self.fkey = fv;
        let fs = self.fs as f64;
        let q = core::f64::consts::FRAC_1_SQRT_2;
        let nyq = 0.45 * fs;
        for c in 0..2 {
            self.er_lc[c].set(SvfMode::Highpass, fs, fv[0] as f64, q, 0.0);
            self.er_lo[c].set(SvfMode::Lowpass, fs, 600.0, q, 0.0);
            self.er_hi[c].set(SvfMode::Highpass, fs, 2500f64.min(nyq), q, 0.0);
            self.er_lp[c].set(SvfMode::Lowpass, fs, (fv[1] as f64).min(nyq), q, 0.0);
            self.lo_damp[c].set(SvfMode::Lowpass, fs, fv[2] as f64, q, 0.0);
            self.hsoft[c].set(SvfMode::Highpass, fs, 3000f64.min(nyq), q, 0.0);
        }
    }

    fn process_chunk(&mut self, left: &mut [f32], right: &mut [f32]) {
        let len = left.len();
        if self.dirty {
            self.apply();
        }
        // ---- ER table crossfade start (one at a time; a newer change waits for the running fade)
        let k = self.er_key();
        if k != self.er_key && self.xf == 0.0 {
            self.er_key = k;
            let nxt = 1 - self.act;
            for c in 0..2 {
                self.tab[nxt][c] = er_table(k[0], k[1], k[2] == 1, c);
            }
            self.xf = f32::MIN_POSITIVE;
        }
        // ---- block-rate filter glides
        for s in &mut self.fv {
            s.advance(len);
        }
        self.update_filters();
        // ---- dry input into the ER ring
        let m = self.mask;
        let w0 = self.w;
        for n in 0..len {
            let i = (w0 + 1 + n) & m;
            self.ring[0][i] = left[n];
            self.ring[1][i] = right[n];
        }
        self.w = (w0 + len) & m;
        // ---- tail
        self.core.process(left, right);
        // ---- per sample: tail post + ER
        let ms = self.fs * 0.001;
        let xf_inc = 1.0 / (XFADE_S * self.fs);
        for n in 0..len {
            // tail: LO DAMP, HI SOFT, width, balance
            let gs: [f32; 4] = core::array::from_fn(|i| self.gs[i].tick());
            let mut t = [left[n], right[n]];
            for (c, v) in t.iter_mut().enumerate() {
                let x = *v + (gs[2] - 1.0) * self.lo_damp[c].process(*v);
                *v = x + (gs[3] - 1.0) * self.hsoft[c].process(x);
            }
            let wd = self.width.tick();
            let (mid, side) = (0.5 * (t[0] + t[1]), 0.5 * wd * (t[0] - t[1]));
            let tl = (mid + side) * self.rv_bal[0].tick();
            let tr = (mid - side) * self.rv_bal[1].tick();
            // ER: read at (newest - (len-1-n)) - pre-delay - tap
            let pd = self.er_pdly.tick();
            let (a, b, x) = (self.act, 1 - self.act, self.xf);
            if x > 0.0 {
                self.xf += xf_inc;
                if self.xf >= 1.0 {
                    self.xf = 0.0;
                    self.act = b;
                }
            }
            let back = (len - 1 - n) as f32 + pd;
            let mut e = [0.0f32; 2];
            for (c, ev) in e.iter_mut().enumerate() {
                let ring = &self.ring[c];
                let read = |tab: &ErTable| {
                    let mut acc = 0.0;
                    for k in 0..ER_MAX {
                        let g = tab.gain[k];
                        if g != 0.0 {
                            acc += g * tap(ring, m, self.w, back + tab.ms[k] * ms);
                        }
                    }
                    acc
                };
                let mut y = read(&self.tab[a][c]);
                if x > 0.0 {
                    y += x * (read(&self.tab[b][c]) - y);
                }
                let y = self.er_lc[c].process(y);
                let y = y
                    + (gs[0] - 1.0) * self.er_lo[c].process(y)
                    + (gs[1] - 1.0) * self.er_hi[c].process(y);
                *ev = self.er_lp[c].process(y) * self.er_lvl[c].tick();
            }
            left[n] = tl + e[0];
            right[n] = tr + e[1];
        }
        for f in self
            .er_lc
            .iter_mut()
            .chain(self.er_lo.iter_mut())
            .chain(self.er_hi.iter_mut())
            .chain(self.er_lp.iter_mut())
            .chain(self.lo_damp.iter_mut())
            .chain(self.hsoft.iter_mut())
        {
            f.flush_denormals();
        }
    }
}

/// Linear-interpolated read `d` samples back from write index `w` of a power-of-two ring.
#[inline(always)]
fn tap(buf: &[f32], mask: usize, w: usize, d: f32) -> f32 {
    let i = d as usize;
    let f = d - i as f32;
    let a = buf[w.wrapping_sub(i) & mask];
    let b = buf[w.wrapping_sub(i + 1) & mask];
    a + (b - a) * f
}

impl Model for Vss3 {
    fn id(&self) -> &'static str {
        "VSS3"
    }

    fn prepare(&mut self, sample_rate: f64, max_block: usize) {
        self.fs = sample_rate as f32;
        self.max_block = max_block.max(1);
        let ms = self.fs * 0.001;
        let n = ((ER_PDLY_MAX_MS + er_max_ms() + 2.0) * ms) as usize + self.max_block + 4;
        let n = n.next_power_of_two();
        self.ring = [vec![0.0; n], vec![0.0; n]];
        self.mask = n - 1;
        self.w = 0;
        for s in self
            .fv
            .iter_mut()
            .chain(self.gs.iter_mut())
            .chain(self.er_lvl.iter_mut())
            .chain(self.rv_bal.iter_mut())
            .chain([&mut self.er_pdly, &mut self.width])
        {
            s.set_time(TAU_S, sample_rate);
        }
        self.apply();
        self.core.prepare(sample_rate);
        self.reset();
    }

    fn process(&mut self, left: &mut [f32], right: &mut [f32], _sc: Option<(&[f32], &[f32])>) {
        let len = left.len().min(right.len());
        let mb = self.max_block;
        let mut i = 0;
        while i < len {
            let e = (i + mb).min(len);
            self.process_chunk(&mut left[i..e], &mut right[i..e]);
            i = e;
        }
    }

    fn reset(&mut self) {
        if self.dirty {
            self.apply();
        }
        for r in &mut self.ring {
            r.iter_mut().for_each(|x| *x = 0.0);
        }
        for f in self
            .er_lc
            .iter_mut()
            .chain(self.er_lo.iter_mut())
            .chain(self.er_hi.iter_mut())
            .chain(self.er_lp.iter_mut())
            .chain(self.lo_damp.iter_mut())
            .chain(self.hsoft.iter_mut())
        {
            f.reset();
        }
        self.core.reset();
        self.snap();
    }

    fn set_param(&mut self, slot: u16, v: ParamValue) {
        let i = slot as usize;
        if i == 0 || i > PARAMS_LEN {
            return;
        }
        let s = &PARAMS[i - 1];
        let x = v.as_f32();
        if !x.is_finite() {
            return;
        }
        self.raw[i - 1] = x.clamp(s.min, s.max);
        self.dirty = true;
    }

    fn param(&self, slot: u16) -> Option<f32> {
        let i = slot as usize;
        (1..=PARAMS_LEN).contains(&i).then(|| self.raw[i - 1])
    }

    fn load_defaults(&mut self) {
        for (r, p) in self.raw.iter_mut().zip(PARAMS.iter()) {
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
