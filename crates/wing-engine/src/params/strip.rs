//! Parameter-driven block hosts: a reference channel processor ([`ParamStrip`]: input -> filter ->
//! gate slot -> dynamics slot -> fader) built from wing-blocks, the model-slot host with
//! preallocated model instances and click-free swaps ([`ModelSlot`]) and a smoothed fader
//! ([`FaderGain`]).
//!
//! `ParamStrip` is the T03A test/bench host for the pipeline; T035 assembles the real channel strip
//! (process-order `proc`, EQ, inserts, side-chain routing) and is expected to reuse `ModelSlot`,
//! `FaderGain` and the layouts below.

use wing_blocks::filter::{FilterBlock, FilterParams, HcSlope, LcSlope};
use wing_blocks::gate::ScFilter;
use wing_blocks::input::{DelayMode, InputParams, InputStage, SourceMode};
use wing_dsp::gain::db_to_lin;
use wing_dsp::smooth::{LinearRamp, OnePoleSmoother};
use wing_dsp::svf::{Svf, SvfMode};
use wing_model::{Schema, FADER_MIN_DB};

use super::map::{BindError, BlockLayout, ParamMapBuilder, TARGET_BASE};
use super::std_models::std_factory;
use super::telemetry::kind;
use super::{ParamMsg, TelemetryWriter};
use crate::graph::{NodeId, NodeIo, ProcessCtx, Processor};

/// Block indices of [`ParamStrip`].
pub mod block {
    pub const INPUT: u8 = 0;
    pub const FILTER: u8 = 1;
    pub const GATE: u8 = 2;
    pub const DYN: u8 = 3;
    pub const FADER: u8 = 4;
}

/// `ch/N/in/set`: trim, balance, polarity, delay on, source mode, source gain mirror; model group
/// `dlymode` with target `dly`.
pub const INPUT_LAYOUT: BlockLayout = BlockLayout {
    params: &[
        ("trim", 0),
        ("bal", 1),
        ("inv", 2),
        ("dlyon", 3),
        ("$mode", 4),
        ("$g", 5),
    ],
    targets: &["dly"],
};

/// `ch/N/flt`: low/high cut (the filter-tool model group is T046 and stays unbound).
pub const FILTER_LAYOUT: BlockLayout = BlockLayout {
    params: &[
        ("lc", 0),
        ("lcf", 1),
        ("lcs", 2),
        ("hc", 3),
        ("hcf", 4),
        ("hcs", 5),
    ],
    targets: &[],
};

/// `ch/N/gate`, `ch/N/dyn`: slot on/mix/gain plus the 19 model slots.
pub const SLOT_LAYOUT: BlockLayout = BlockLayout {
    params: &[("on", 0), ("mix", 1), ("gain", 2)],
    targets: &[
        "1", "2", "3", "4", "5", "6", "7", "8", "9", "10", "11", "12", "13", "14", "15", "16",
        "17", "18", "19",
    ],
};

/// `ch/N`: fader and mute.
pub const FADER_LAYOUT: BlockLayout = BlockLayout {
    params: &[("fdr", 0), ("mute", 1)],
    targets: &[],
};

/// Bind channel `ch` (1-based) of the wing-model tree to a [`ParamStrip`] at graph node `node`
/// and register it as a telemetry source.
pub fn bind_channel(b: &mut ParamMapBuilder, ch: u32, node: NodeId) -> Result<(), BindError> {
    b.bind_block(
        &format!("ch/{ch}/in/set"),
        node,
        block::INPUT,
        &INPUT_LAYOUT,
    )?;
    b.bind_block(&format!("ch/{ch}/flt"), node, block::FILTER, &FILTER_LAYOUT)?;
    b.bind_block(&format!("ch/{ch}/gate"), node, block::GATE, &SLOT_LAYOUT)?;
    b.bind_block(&format!("ch/{ch}/dyn"), node, block::DYN, &SLOT_LAYOUT)?;
    b.bind_block(&format!("ch/{ch}"), node, block::FADER, &FADER_LAYOUT)?;
    b.telemetry_node(node);
    Ok(())
}

// ---- fader ---------------------------------------------------------------------------------

/// Default fader/mute smoothing time constant.
pub const FADER_TAU_S: f64 = 0.005;

/// Fader + mute as one smoothed linear gain (one-pole on the linear gain, so a fade to -oo reaches
/// silence smoothly). The fader value is in dB (`-144` = -oo).
pub struct FaderGain {
    fdr_db: f32,
    mute: bool,
    g: OnePoleSmoother,
    fs: f64,
}

impl FaderGain {
    pub fn new(fs: f64) -> Self {
        Self {
            fdr_db: FADER_MIN_DB,
            mute: false,
            g: OnePoleSmoother::new(0.0, FADER_TAU_S, fs),
            fs,
        }
    }

    /// Smoothing time constant (0 = none; only for tests that need the unsmoothed baseline).
    pub fn set_smoothing(&mut self, tau_s: f64) {
        self.g.set_time(tau_s, self.fs);
    }

    pub fn set_fader_db(&mut self, db: f32) {
        self.fdr_db = db;
        self.update();
    }

    pub fn set_mute(&mut self, mute: bool) {
        self.mute = mute;
        self.update();
    }

    fn update(&mut self) {
        let t = if self.mute || self.fdr_db <= FADER_MIN_DB {
            0.0
        } else {
            db_to_lin(self.fdr_db)
        };
        self.g.set_target(t);
    }

    /// Target linear gain.
    pub fn target(&self) -> f32 {
        self.g.target()
    }

    /// Jump to the target.
    pub fn reset(&mut self) {
        self.g.reset(self.g.target());
    }

    pub fn process(&mut self, l: &mut [f32], r: &mut [f32]) {
        if self.g.is_settled(1e-6) {
            let g = self.g.target();
            self.g.reset(g);
            if g == 1.0 {
                return;
            }
            for (a, b) in l.iter_mut().zip(r.iter_mut()) {
                *a *= g;
                *b *= g;
            }
            return;
        }
        for (a, b) in l.iter_mut().zip(r.iter_mut()) {
            let g = self.g.tick();
            *a *= g;
            *b *= g;
        }
    }
}

// ---- model slot ----------------------------------------------------------------------------

/// Model swap crossfade length.
pub const MODEL_XFADE_S: f64 = 0.005;
/// Slot on/off, mix and gain ramp length.
pub const SLOT_RAMP_S: f64 = 0.005;

/// The interface a model implementation exposes to a [`ModelSlot`]. This is the *only* thing the
/// engine's model-swap mechanism depends on; wing-blocks' model registry (T045) implements it.
/// Every method except construction runs on RT threads and must not allocate, lock or block.
///
/// Parameter positions are the 0-based wing-model slot positions of the model (`ch/N/dyn/1` =
/// position 0), values are engineering units as produced by the pipeline: floats in dB/Hz/ms/%,
/// ints/bools as numbers, enums as their index, float-enums (compressor ratio) as their value.
pub trait RtModel: Send {
    /// Load the model's default parameters (called when the slot switches *to* this model; the
    /// pipeline follows with the store's values for every position).
    fn load_defaults(&mut self);
    /// Set parameter `pos` (accumulate; apply in [`RtModel::commit`]).
    fn set_param(&mut self, pos: u16, value: f32);
    /// Current value of parameter `pos` as last set (introspection/tests); `None` if unused.
    fn param(&self, _pos: u16) -> Option<f32> {
        None
    }
    /// Apply accumulated parameter changes (block rate, at most once per cycle).
    fn commit(&mut self);
    /// Clear signal state (detectors, filters, delay lines) before the model is faded in.
    fn reset(&mut self);
    /// Process a stereo block in place (`l.len() == r.len() <= max_block` given at construction).
    fn process(&mut self, l: &mut [f32], r: &mut [f32]);
    /// Process with a side-chain key (already filtered by the slot; `None` = key from the input).
    /// Default: ignore the key (models without a detector).
    fn process_keyed(&mut self, l: &mut [f32], r: &mut [f32], key: Option<(&[f32], &[f32])>) {
        let _ = key;
        self.process(l, r);
    }
    /// Gain reduction in dB (>= 0) for telemetry; 0 if not applicable.
    fn gain_reduction_db(&self) -> f32 {
        0.0
    }
    /// Processing latency in samples.
    fn latency(&self) -> usize {
        0
    }
}

/// Instance index meaning "no implementation: pass-through".
const BYPASS: u8 = u8::MAX;
const MAX_MODELS: usize = 64;

/// A model-dependent slot (gate, dynamics, EQ, filter tool, ...): one preallocated [`RtModel`]
/// instance per model the slot can host, crossfaded swaps and smoothed wrapper parameters
/// (`out = dry * (1 - w) + w * gain * model(dry)`, `w = on * mix`). With `w = gain = 1` settled and no
/// swap running, the model output passes untouched (bit-exact with the bare block).
///
/// Side-chain (T035): an optional key filter (`gatesc`/`dynsc` `type f q`, TPT SVF) shapes the key the
/// model detects on: the external key passed to [`ModelSlot::process_keyed`] or, for SELF, a copy of
/// the input. The filtered key is kept for key-listen ([`ModelSlot::key_signal`]).
///
/// Swap protocol (all RT-safe, no allocation): [`ModelSlot::select`] loads the incoming model's
/// defaults; if nothing is fading, the incoming instance is `reset` and crossfaded in linearly over
/// [`MODEL_XFADE_S`] (both paths process the same input, so outputs are correlated and an
/// equal-gain fade keeps the level). Selecting the fading-out model during a fade reverses it;
/// any other selection is applied when the running fade completes. Models without an instance
/// pass audio through.
pub struct ModelSlot {
    instances: Box<[Box<dyn RtModel>]>,
    /// WING enum index -> instance index (or BYPASS).
    index: [u8; MAX_MODELS],
    dirty: u64,
    wrapper_dirty: bool,
    logical: u8,
    logical_index: u16,
    cur: u8,
    prev: u8,
    fading: bool,
    xf_pos: u32,
    xf_len: u32,
    on: bool,
    mix_pct: f32,
    gain_db: f32,
    wet: LinearRamp,
    makeup: LinearRamp,
    idle: bool,
    max_block: usize,
    dry_l: Vec<f32>,
    dry_r: Vec<f32>,
    prev_l: Vec<f32>,
    prev_r: Vec<f32>,
    mix_max: f32,
    fs: f64,
    key_type: ScFilter,
    key_freq: f32,
    key_q: f32,
    key_filt: [Svf; 2],
    key_l: Vec<f32>,
    key_r: Vec<f32>,
    key_len: usize,
}

impl ModelSlot {
    /// Build a slot for the wing-model group `group_key` (`ch/*/dyn`, ...): `factory(enum index,
    /// model name)` is called once per model of the group and returns its preallocated instance, or
    /// `None` for pass-through. Allocates (control thread).
    pub fn from_group(
        group_key: &str,
        fs: f64,
        max_block: usize,
        mut factory: impl FnMut(u16, &'static str) -> Option<Box<dyn RtModel>>,
    ) -> Self {
        let mut index = [BYPASS; MAX_MODELS];
        let mut instances: Vec<Box<dyn RtModel>> = Vec::new();
        let s = Schema::get();
        if let Some(g) = s.groups().iter().find(|g| g.key == group_key) {
            for m in s.models(g) {
                let e = m.enum_index as usize;
                if e >= MAX_MODELS || instances.len() >= BYPASS as usize {
                    continue;
                }
                if let Some(inst) = factory(m.enum_index as u16, m.name) {
                    index[e] = instances.len() as u8;
                    instances.push(inst);
                }
            }
        }
        assert!(instances.len() <= 64, "at most 64 model instances per slot");
        let n = max_block.max(1);
        let ramp = (SLOT_RAMP_S * fs).round() as u32;
        Self {
            instances: instances.into(),
            index,
            dirty: 0,
            wrapper_dirty: true,
            logical: BYPASS,
            logical_index: u16::MAX,
            cur: BYPASS,
            prev: BYPASS,
            fading: false,
            xf_pos: 0,
            xf_len: (MODEL_XFADE_S * fs).round() as u32,
            on: false,
            mix_pct: 100.0,
            gain_db: 0.0,
            wet: LinearRamp::new(0.0, ramp),
            makeup: LinearRamp::new(1.0, ramp),
            idle: true,
            max_block: n,
            dry_l: vec![0.0; n],
            dry_r: vec![0.0; n],
            prev_l: vec![0.0; n],
            prev_r: vec![0.0; n],
            mix_max: 100.0,
            fs,
            key_type: ScFilter::Off,
            key_freq: 1000.0,
            key_q: 0.707,
            key_filt: [Svf::default(); 2],
            key_l: vec![0.0; n],
            key_r: vec![0.0; n],
            key_len: 0,
        }
    }

    /// Upper limit of `mix` in % (100 for gate/dyn, 125 for EQ).
    pub fn set_mix_max(&mut self, pct: f32) {
        self.mix_max = pct.max(0.0);
    }

    /// Side-chain key filter (`type`, `f` 20..20k Hz, `q` 0.44..10). RT-safe.
    pub fn set_key_filter(&mut self, kind: ScFilter, freq: f32, q: f32) {
        self.key_type = kind;
        self.key_freq = freq.clamp(20.0, 20_000.0);
        self.key_q = q.clamp(0.44, 10.0);
        let mode = match kind {
            ScFilter::Off => return,
            ScFilter::Lp12 => SvfMode::Lowpass,
            ScFilter::Hp12 => SvfMode::Highpass,
            ScFilter::Bp => SvfMode::Bandpass,
            ScFilter::Notch => SvfMode::Notch,
        };
        for f in &mut self.key_filt {
            f.set(mode, self.fs, self.key_freq as f64, self.key_q as f64, 0.0);
        }
    }

    /// `(type, f, q)` of the key filter.
    pub fn key_filter(&self) -> (ScFilter, f32, f32) {
        (self.key_type, self.key_freq, self.key_q)
    }

    /// Filtered key of the last processed chunk (key listen); empty when no key was formed.
    pub fn key_signal(&self) -> (&[f32], &[f32]) {
        (&self.key_l[..self.key_len], &self.key_r[..self.key_len])
    }

    /// Commit pending changes and jump every ramp/fade to its target, clearing signal state
    /// (tests, offline rendering, scene start).
    pub fn settle(&mut self) {
        self.commit();
        self.wet
            .reset(if self.on { self.mix_pct * 0.01 } else { 0.0 });
        self.makeup.reset(db_to_lin(self.gain_db));
        self.fading = false;
        if self.logical != self.cur {
            self.prev = self.cur;
            self.cur = self.logical;
        }
        self.reset_instance(self.cur);
        for f in &mut self.key_filt {
            f.reset();
        }
        self.idle = self.wet.value() == 0.0;
    }

    /// Whether a model is currently being run (on, or fading).
    pub fn is_running(&self) -> bool {
        !self.idle
    }

    /// Latency of the audible model (samples).
    pub fn latency(&self) -> usize {
        self.instances
            .get(self.cur as usize)
            .map_or(0, |m| m.latency())
    }

    /// Crossfade length in samples (0 = hard switch; tests use it for the click baseline).
    pub fn set_xfade_samples(&mut self, n: u32) {
        self.xf_len = n;
    }

    /// Number of preallocated model instances.
    pub fn num_instances(&self) -> usize {
        self.instances.len()
    }

    /// Model receiving parameters (WING enum index), if any was selected.
    pub fn model_index(&self) -> Option<u16> {
        (self.logical_index != u16::MAX).then_some(self.logical_index)
    }

    /// Whether the selected model has an implementation (else the slot passes audio through).
    pub fn has_instance(&self) -> bool {
        self.logical != BYPASS
    }

    /// Instance receiving parameters.
    pub fn model(&self) -> Option<&dyn RtModel> {
        self.instances.get(self.logical as usize).map(|b| &**b)
    }

    /// Enum index -> instance index of the audible model (the fade-in side while crossfading).
    pub fn audible_instance(&self) -> Option<usize> {
        (self.cur != BYPASS).then_some(self.cur as usize)
    }

    /// Instance index hosting WING model `index`.
    pub fn instance_of(&self, index: u16) -> Option<usize> {
        self.index
            .get(index as usize)
            .copied()
            .filter(|&i| i != BYPASS)
            .map(|i| i as usize)
    }

    pub fn is_crossfading(&self) -> bool {
        self.fading
    }

    /// `(on, mix %, gain dB)`.
    pub fn wrapper(&self) -> (bool, f32, f32) {
        (self.on, self.mix_pct, self.gain_db)
    }

    /// Select a model by WING enum index (RT-safe).
    pub fn select(&mut self, index: u16) {
        let k = self.index.get(index as usize).copied().unwrap_or(BYPASS);
        self.logical_index = index;
        self.logical = k;
        if k != BYPASS {
            self.instances[k as usize].load_defaults();
            self.dirty |= 1 << k;
        }
        if self.fading {
            if k == self.prev {
                // reverse the running fade back to the previous model (no reset: it is audible)
                std::mem::swap(&mut self.cur, &mut self.prev);
                self.xf_pos = self.xf_len - self.xf_pos.min(self.xf_len);
            }
            // otherwise: picked up when the running fade completes
        } else if k != self.cur {
            self.start_fade(k);
        }
    }

    fn start_fade(&mut self, k: u8) {
        self.reset_instance(k);
        self.prev = self.cur;
        self.cur = k;
        if self.xf_len == 0 || self.idle {
            self.fading = false;
        } else {
            self.fading = true;
            self.xf_pos = 0;
        }
    }

    fn reset_instance(&mut self, k: u8) {
        if let Some(m) = self.instances.get_mut(k as usize) {
            m.reset();
        }
    }

    /// Set model target `pos` (0-based) of the model receiving parameters (RT-safe).
    pub fn set_target(&mut self, pos: u16, v: f32) {
        let k = self.logical;
        if k != BYPASS {
            self.instances[k as usize].set_param(pos, v);
            self.dirty |= 1 << k;
        }
    }

    /// Wrapper parameters: 0 = on, 1 = mix %, 2 = gain dB.
    pub fn set_wrapper(&mut self, param: u16, v: f32) {
        match param {
            0 => self.on = v != 0.0,
            1 => self.mix_pct = v.clamp(0.0, self.mix_max),
            2 => self.gain_db = v.clamp(-6.0, 12.0),
            _ => return,
        }
        self.wrapper_dirty = true;
    }

    /// Apply accumulated parameter changes (block rate).
    pub fn commit(&mut self) {
        if self.wrapper_dirty {
            self.wrapper_dirty = false;
            let w = if self.on { self.mix_pct * 0.01 } else { 0.0 };
            if self.idle && w > 0.0 {
                // waking up: start from a clean state and fade in from dry
                self.reset_instance(self.cur);
            }
            self.wet.set_target(w);
            self.makeup.set_target(db_to_lin(self.gain_db));
        }
        let mut d = self.dirty;
        while d != 0 {
            let k = d.trailing_zeros() as usize;
            d &= d - 1;
            self.instances[k].commit();
        }
        self.dirty = 0;
    }

    /// Gain reduction (dB, >= 0) of the audible model (0 while idle).
    pub fn gain_reduction_db(&self) -> f32 {
        if self.idle {
            return 0.0;
        }
        self.instances
            .get(self.cur as usize)
            .map_or(0.0, |m| m.gain_reduction_db())
    }

    /// Process a stereo block in place (RT-safe), keying from the input.
    pub fn process(&mut self, l: &mut [f32], r: &mut [f32]) {
        self.process_keyed(l, r, None);
    }

    /// Process with an optional external side-chain key (`src` != SELF, resolved by the engine);
    /// key samples beyond a shorter key slice read as 0. RT-safe.
    pub fn process_keyed(&mut self, l: &mut [f32], r: &mut [f32], key: Option<(&[f32], &[f32])>) {
        let mut start = 0;
        while start < l.len() {
            let end = (start + self.max_block).min(l.len());
            let k = key.map(|(a, b)| (sub(a, start, end), sub(b, start, end)));
            self.process_chunk(&mut l[start..end], &mut r[start..end], k);
            start = end;
        }
    }

    fn process_chunk(&mut self, l: &mut [f32], r: &mut [f32], key: Option<(&[f32], &[f32])>) {
        let n = l.len();
        if !self.wet.is_ramping() && self.wet.value() == 0.0 {
            self.idle = true;
            self.key_len = 0;
            self.fading = false; // inaudible: finish instantly
            if self.logical != self.cur {
                self.start_fade(self.logical);
            }
            return;
        }
        if self.idle {
            for f in &mut self.key_filt {
                f.reset();
            }
        }
        self.idle = false;
        let full = !self.fading
            && !self.wet.is_ramping()
            && self.wet.value() == 1.0
            && !self.makeup.is_ramping()
            && self.makeup.value() == 1.0;
        if !full {
            self.dry_l[..n].copy_from_slice(l);
            self.dry_r[..n].copy_from_slice(r);
        }
        // ---- side-chain key ----
        let filt = self.key_type != ScFilter::Off;
        let use_key = filt || key.is_some();
        if use_key {
            let (kl, kr) = (&mut self.key_l[..n], &mut self.key_r[..n]);
            match key {
                Some((a, b)) => {
                    copy_padded(kl, a);
                    copy_padded(kr, b);
                }
                None => {
                    kl.copy_from_slice(l);
                    kr.copy_from_slice(r);
                }
            }
            if filt {
                self.key_filt[0].process_block(kl);
                self.key_filt[1].process_block(kr);
            }
        }
        self.key_len = if use_key { n } else { 0 };
        let sc = use_key.then(|| (&self.key_l[..n], &self.key_r[..n]));
        if self.fading {
            self.prev_l[..n].copy_from_slice(l);
            self.prev_r[..n].copy_from_slice(r);
            if let Some(m) = self.instances.get_mut(self.prev as usize) {
                m.process_keyed(&mut self.prev_l[..n], &mut self.prev_r[..n], sc);
            }
        }
        if let Some(m) = self.instances.get_mut(self.cur as usize) {
            m.process_keyed(l, r, sc);
        }
        if self.fading {
            let len = self.xf_len as f32;
            for i in 0..n {
                let g = ((self.xf_pos + i as u32) as f32 / len).min(1.0);
                l[i] = self.prev_l[i] + g * (l[i] - self.prev_l[i]);
                r[i] = self.prev_r[i] + g * (r[i] - self.prev_r[i]);
            }
            self.xf_pos += n as u32;
            if self.xf_pos >= self.xf_len {
                self.fading = false;
                if self.logical != self.cur {
                    self.start_fade(self.logical);
                }
            }
        }
        if !full {
            for i in 0..n {
                let w = self.wet.tick();
                let m = self.makeup.tick();
                l[i] = self.dry_l[i] + w * (m * l[i] - self.dry_l[i]);
                r[i] = self.dry_r[i] + w * (m * r[i] - self.dry_r[i]);
            }
        }
    }
}

#[inline]
fn sub(k: &[f32], s: usize, e: usize) -> &[f32] {
    &k[s.min(k.len())..e.min(k.len())]
}

#[inline]
fn copy_padded(dst: &mut [f32], src: &[f32]) {
    let m = src.len().min(dst.len());
    dst[..m].copy_from_slice(&src[..m]);
    dst[m..].fill(0.0);
}

// ---- strip ---------------------------------------------------------------------------------

/// Reference parameter-driven channel processor: 2 inputs (L, R; a single input feeds both) ->
/// input stage -> filter -> gate slot -> dynamics slot -> fader -> 2 outputs.
pub struct ParamStrip {
    input: InputStage,
    input_p: InputParams,
    input_dirty: bool,
    filter: FilterBlock,
    filter_p: FilterParams,
    filter_dirty: bool,
    gate: ModelSlot,
    dyn_: ModelSlot,
    fader: FaderGain,
}

impl ParamStrip {
    /// Allocates everything (control thread).
    pub fn new(fs: f64, max_block: usize) -> Self {
        let mut input = InputStage::new();
        input.prepare(fs, max_block);
        let mut filter = FilterBlock::new();
        filter.prepare(fs, max_block);
        Self {
            input,
            input_p: InputParams::default(),
            input_dirty: false,
            filter,
            filter_p: FilterParams::default(),
            filter_dirty: false,
            gate: ModelSlot::from_group("ch/*/gate", fs, max_block, std_factory(fs, max_block)),
            dyn_: ModelSlot::from_group("ch/*/dyn", fs, max_block, std_factory(fs, max_block)),
            fader: FaderGain::new(fs),
        }
    }

    pub fn input_params(&self) -> &InputParams {
        self.input.params()
    }
    pub fn filter_params(&self) -> &FilterParams {
        self.filter.params()
    }
    pub fn gate_slot(&self) -> &ModelSlot {
        &self.gate
    }
    pub fn gate_slot_mut(&mut self) -> &mut ModelSlot {
        &mut self.gate
    }
    pub fn dyn_slot(&self) -> &ModelSlot {
        &self.dyn_
    }
    pub fn dyn_slot_mut(&mut self) -> &mut ModelSlot {
        &mut self.dyn_
    }
    pub fn fader(&self) -> &FaderGain {
        &self.fader
    }
    pub fn fader_mut(&mut self) -> &mut FaderGain {
        &mut self.fader
    }

    fn set(&mut self, block: u8, param: u16, v: f32) {
        match block {
            block::INPUT => {
                let p = &mut self.input_p;
                match param {
                    0 => p.trim_db = v,
                    1 => p.balance_db = v,
                    2 => p.invert = v != 0.0,
                    3 => p.delay_on = v != 0.0,
                    4 => {
                        p.source_mode = match v as u32 {
                            0 => SourceMode::Mono,
                            1 => SourceMode::Stereo,
                            _ => SourceMode::MidSide,
                        }
                    }
                    5 => p.gain_db = v,
                    TARGET_BASE => p.delay = v,
                    _ => return,
                }
                self.input_dirty = true;
            }
            block::FILTER => {
                let p = &mut self.filter_p;
                match param {
                    0 => p.lc_on = v != 0.0,
                    1 => p.lc_freq = v,
                    2 => {
                        p.lc_slope = match v as u32 {
                            0 => LcSlope::Db6,
                            1 => LcSlope::Db12,
                            2 => LcSlope::Db18,
                            _ => LcSlope::Db24,
                        }
                    }
                    3 => p.hc_on = v != 0.0,
                    4 => p.hc_freq = v,
                    5 => {
                        p.hc_slope = if v as u32 == 0 {
                            HcSlope::Db6
                        } else {
                            HcSlope::Db12
                        }
                    }
                    _ => return,
                }
                self.filter_dirty = true;
            }
            block::GATE | block::DYN => {
                let s = if block == block::GATE {
                    &mut self.gate
                } else {
                    &mut self.dyn_
                };
                if param >= TARGET_BASE {
                    s.set_target(param - TARGET_BASE, v);
                } else {
                    s.set_wrapper(param, v);
                }
            }
            block::FADER => match param {
                0 => self.fader.set_fader_db(v),
                1 => self.fader.set_mute(v != 0.0),
                _ => {}
            },
            _ => {}
        }
    }

    fn select(&mut self, block: u8, model: u16) {
        match block {
            block::INPUT => {
                self.input_p.delay_mode = match model {
                    0 => DelayMode::Meters,
                    1 => DelayMode::Feet,
                    2 => DelayMode::Ms,
                    _ => DelayMode::Samples,
                };
                self.input_dirty = true;
            }
            block::GATE => self.gate.select(model),
            block::DYN => self.dyn_.select(model),
            _ => {}
        }
    }

    /// Apply a message directly (same as [`Processor::set_param`]).
    pub fn apply(&mut self, msg: &ParamMsg) {
        match *msg {
            ParamMsg::Set {
                block,
                param,
                value,
            } => self.set(block, param, value),
            ParamMsg::Model { block, model } => self.select(block, model),
            ParamMsg::Commit => {
                if self.input_dirty {
                    self.input.set_params(&self.input_p);
                    self.input_dirty = false;
                }
                if self.filter_dirty {
                    self.filter.set_params(&self.filter_p);
                    self.filter_dirty = false;
                }
                self.gate.commit();
                self.dyn_.commit();
            }
        }
    }

    /// Process a stereo block in place.
    pub fn process_stereo(&mut self, l: &mut [f32], r: &mut [f32]) {
        self.input.process(l, r);
        self.filter.process(l, r);
        self.gate.process(l, r);
        self.dyn_.process(l, r);
        self.fader.process(l, r);
    }
}

impl Processor for ParamStrip {
    fn process(&mut self, _ctx: &ProcessCtx, io: &mut NodeIo<'_>) {
        let ins = io.inputs();
        let b = io.block_size();
        let out = io.outputs();
        let (l, rest) = out.split_at_mut(b);
        let r = &mut rest[..b];
        match ins.len() {
            0 => {
                l.fill(0.0);
                r.fill(0.0);
            }
            1 => {
                l.copy_from_slice(ins.get(0));
                r.copy_from_slice(ins.get(0));
            }
            _ => {
                l.copy_from_slice(ins.get(0));
                r.copy_from_slice(ins.get(1));
            }
        }
        self.process_stereo(l, r);
    }

    fn set_param(&mut self, msg: &ParamMsg) {
        self.apply(msg);
    }

    fn telemetry(&mut self, out: &mut TelemetryWriter<'_>) {
        out.push(
            block::GATE,
            kind::GAIN_REDUCTION_DB,
            self.gate.gain_reduction_db(),
        );
        out.push(
            block::DYN,
            kind::GAIN_REDUCTION_DB,
            self.dyn_.gain_reduction_db(),
        );
    }
}
