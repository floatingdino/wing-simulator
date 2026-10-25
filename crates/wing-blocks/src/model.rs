//! Model framework (T045): the [`Model`] trait every selectable processing model implements, the model
//! catalogs of `docs/research/models.md`, a factory, and [`ModelRegistry`] (one preallocated instance per
//! selectable model so a model switch on the RT thread is an index change plus `reset`, never an allocation).
//!
//! The engine's model-swap host (`wing_engine::params::strip::ModelSlot`, T03A: preallocated instances,
//! crossfaded swaps, on/mix/gain wrapper, key filter) drives these models through its `RtModel` trait, which
//! wing-engine implements for `Box<dyn Model>`; [`ModelRegistry`] is the stand-alone equivalent for offline
//! use (fidelity harness, renderer, tests).
//!
//! Parameter addressing: [`Model::set_param`] takes the **schema slot position** of the parameter, i.e. the
//! numeric child name in the WING tree (`ch/1/dyn/1` = slot 1 = `thr` for COMP, `ingain` for LA; see
//! `wing_model::Schema::model_params`). Blocks that are not model-dependent (input stage, LC/HC filter) use
//! the 1-based position of the key among the block's schema children (constants in [`crate::std_models`]).
//! Values arrive as [`ParamValue`] in engineering units: floats in dB/Hz/ms/%, ints/bools as `Int`, enums as
//! their index in the schema token list, float enums (`FEnum`, e.g. compressor ratios) as `Float(value)`.
//!
//! Wrapper parameters that WING applies to *every* model of a block (`on`, `mix`, `gain`, side-chain key filter)
//! are handled by the slot host (wing-engine `ModelSlot`), not by the models.
//!
//! Models still to be emulated are [`StubModel`]s (passthrough, `is_stub() == true`, the owning task in
//! [`Model::todo`]) so the whole catalog is selectable today; S4/S5 tasks replace them in [`create_model`].

use crate::eq::EqVariant;
use crate::models::comp_console::{ConsoleComp, ConsoleKind};
use crate::models::comp_fet::{FetComp, FetKind};
use crate::models::comp_multi::{OneKnobComp, ThreeBandComp};
use crate::models::comp_opto::{OptoComp, OptoKind};
use crate::models::comp_vca::{VcaComp, VcaKind};
use crate::models::dyn_eq::{DeEsser902, DynEq};
use crate::models::dyn_special::{AutoRider, TransientDesigner, WarmthPre};
use crate::models::eq_alt::{AltEq, AltKind};
use crate::models::filter_tool::FilterToolModel;
use crate::models::gate_vintage::{VintageGate, VintageGateKind};
use crate::models::pse::{PseKind, SourceExtractor};
use crate::std_models::{DynModel, GateModel, StdEqModel};

/// A parameter value in engineering units (see module doc for the encoding).
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ParamValue {
    Float(f32),
    Int(i32),
    /// Index into the schema token list of an `Enum` parameter.
    Enum(u32),
}

impl ParamValue {
    /// Numeric view: floats as-is, ints and enum indices converted.
    pub fn as_f32(self) -> f32 {
        match self {
            ParamValue::Float(v) => v,
            ParamValue::Int(v) => v as f32,
            ParamValue::Enum(v) => v as f32,
        }
    }

    /// Integer / bool / enum-index view (floats are rounded).
    pub fn as_i32(self) -> i32 {
        match self {
            ParamValue::Float(v) => v.round() as i32,
            ParamValue::Int(v) => v,
            ParamValue::Enum(v) => v as i32,
        }
    }

    pub fn as_bool(self) -> bool {
        self.as_i32() != 0
    }

    pub fn as_index(self) -> usize {
        self.as_i32().max(0) as usize
    }
}

/// Per-model readout for meters and the UI.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ModelTelemetry {
    /// Current gain reduction in dB (>= 0; 0 for non-dynamics models).
    pub gain_reduction_db: f32,
    /// Gate open / ducker ducking / compressor over threshold (model-defined; false if not applicable).
    pub engaged: bool,
}

/// A selectable processing model. All methods except [`Model::prepare`] must be allocation-, lock- and
/// syscall-free (RT-safe).
pub trait Model: Send {
    /// Snapshot model id (`mdl` token, e.g. `COMP`).
    fn id(&self) -> &'static str;

    /// Set sample rate and maximum block length, allocate scratch, reset state. Off the RT thread.
    fn prepare(&mut self, sample_rate: f64, max_block: usize);

    /// Process one stereo block in place (stereo-linked). `sidechain` is an external key (already filtered by
    /// the slot's key filter); `None` = key from the model's own input. Blocks longer than the `max_block`
    /// given to `prepare` must not be passed (the slot hosts chunk).
    fn process(&mut self, left: &mut [f32], right: &mut [f32], sidechain: Option<(&[f32], &[f32])>);

    /// Processing latency in samples.
    fn latency(&self) -> usize {
        0
    }

    /// Clear internal state (filters, envelopes) and settle parameter smoothers.
    fn reset(&mut self);

    /// Set the parameter at schema slot `slot` (1-based). Unknown slots are ignored. Implementations may
    /// stage the value and apply it in [`Model::commit`] (or lazily at the next `process`/`reset`).
    fn set_param(&mut self, slot: u16, value: ParamValue);

    /// Current (staged) value of slot `slot` as a number, `None` if the slot is unused.
    fn param(&self, _slot: u16) -> Option<f32> {
        None
    }

    /// Load the model's WING-Edit default parameters (staged like `set_param`).
    fn load_defaults(&mut self) {}

    /// EQ `mix` (%): WING scales every EQ model's band gains (dB) by `mix/100` instead of blending dry/wet
    /// (`ae::Cequalizer::update`, T03C). Models without gain-scaled bands ignore it. Survives
    /// [`Model::load_defaults`]; staged like `set_param`.
    fn set_gain_scale(&mut self, _mix_pct: f32) {}

    /// Apply staged parameter changes now (block rate). Models that apply lazily may do nothing.
    fn commit(&mut self) {}

    /// Gain reduction / state readout.
    fn telemetry(&self) -> ModelTelemetry {
        ModelTelemetry::default()
    }

    /// True for placeholder models that pass audio through unchanged.
    fn is_stub(&self) -> bool {
        false
    }

    /// For stubs: the backlog task that will implement the model.
    fn todo(&self) -> Option<&'static str> {
        None
    }
}

/// Block families that own a model table.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BlockKind {
    /// Channel filter tool (`flt.mdl`, enabled by `flt.tf`).
    FilterTool,
    /// Gate *and* dynamics slots (one shared table since fw 3.1).
    Dynamics,
    /// EQ (`eq.mdl`) of the given strip variant.
    Eq(EqVariant),
    /// FX rack slot (`fx.N.mdl`).
    Fx,
}

/// Filter tool models (models.md §2).
pub const FILTER_TOOL_IDS: [&str; 4] = ["TILT", "MAX", "AP1", "AP2"];

/// Gate/dynamics model table in WING-Edit order (models.md §3; `3CMP` is hidden in WING-Edit 3.3.4).
pub const DYN_IDS: [&str; 33] = [
    "GATE", "DUCK", "E88", "9000G", "D241G", "DS902", "DEQ", "DEQ2", "WAVE", "PSE", "CMB", "RIDE",
    "WARM", "COMP", "EXP", "B160", "B560", "D241C", "ECL33", "9000C", "SBUS", "RED3", "76LA", "LA",
    "F670", "BLISS", "NSTR", "2250", "L100", "E88C", "LMT", "ONEC", "3CMP",
];

/// EQ models (models.md §4); channel: STD..MACH4, aux: STD..PULSAR, bus/main/mtx: STD..PULSAR + PIA.
pub const EQ_IDS: [&str; 8] = [
    "STD", "SOUL", "E88", "E84", "F110", "PULSAR", "MACH4", "PIA",
];

/// FX rack models (models.md §6, FX1-8 selector order incl. `NONE`).
pub const FX_IDS: [&str; 65] = [
    "NONE",
    "EXT",
    "HALL",
    "ROOM",
    "CHAMBER",
    "PLATE",
    "CONCERT",
    "AMBI",
    "V-ROOM",
    "V-REV",
    "V-PLATE",
    "GATED",
    "REVERSE",
    "DEL/REV",
    "SHIMMER",
    "SPRING",
    "DIMCRS",
    "CHORUS",
    "FLANGER",
    "ST-DL",
    "TAP-DL",
    "TAPE-DL",
    "OILCAN",
    "BBD-DL",
    "PITCH",
    "D-PITCH",
    "VSS3",
    "BPLATE",
    "GEQ",
    "PIA",
    "DOUBLE",
    "PCORR",
    "LIMITER",
    "DE-S2",
    "ENHANCE",
    "EXCITER",
    "P-BASS",
    "ROTARY",
    "PHASER",
    "PANNER",
    "TAPE",
    "MOOD",
    "SUB",
    "RACKAMP",
    "UKROCK",
    "ANGEL",
    "JAZZC",
    "DELUXE",
    "BODY",
    "SOUL",
    "E88",
    "E84",
    "F110",
    "PULSAR",
    "MACH4",
    "C5-CMB",
    "SUB-M",
    "V-IMG",
    "SPKMAN",
    "DEQ3",
    "*EVEN*",
    "*SOUL*",
    "*VINTAGE*",
    "*BUS*",
    "*MASTER*",
];

/// The catalog of a block kind (all ids known to models.md).
pub fn catalog(kind: BlockKind) -> &'static [&'static str] {
    match kind {
        BlockKind::FilterTool => &FILTER_TOOL_IDS,
        BlockKind::Dynamics => &DYN_IDS,
        BlockKind::Eq(_) => &EQ_IDS,
        BlockKind::Fx => &FX_IDS,
    }
}

/// Backlog task that replaces the stub for `id` (for [`Model::todo`]).
fn owning_task(kind: BlockKind, id: &str) -> &'static str {
    match kind {
        BlockKind::FilterTool => "T046",
        BlockKind::Dynamics => match id {
            "E88" | "9000G" | "D241G" => "T047",
            "DS902" | "DEQ" | "DEQ2" => "T048",
            "WAVE" | "RIDE" | "WARM" => "T049",
            "PSE" | "CMB" => "T04A",
            "B160" | "B560" | "D241C" | "9000C" | "SBUS" => "T04B",
            "ECL33" | "E88C" | "RED3" | "2250" | "LMT" => "T04C",
            "76LA" | "NSTR" => "T04D",
            "LA" | "L100" | "F670" | "BLISS" => "T04E",
            _ => "T04F",
        },
        BlockKind::Eq(_) => match id {
            "SOUL" | "E88" | "E84" => "T04G",
            _ => "T04H",
        },
        BlockKind::Fx => match id {
            "RACKAMP" | "UKROCK" | "ANGEL" | "JAZZC" | "DELUXE" | "BODY" => "T05A",
            "*EVEN*" | "*SOUL*" | "*VINTAGE*" | "*BUS*" | "*MASTER*" | "EXT" => "T05C",
            "ST-DL" | "TAP-DL" | "TAPE-DL" | "OILCAN" | "BBD-DL" => "T05D",
            "ENHANCE" | "EXCITER" | "P-BASS" | "SUB-M" | "V-IMG" | "TAPE" => "T05E",
            "DIMCRS" | "CHORUS" | "FLANGER" | "PHASER" | "ROTARY" | "PANNER" | "MOOD" => "T05M",
            "SPRING" | "PLATE" | "V-PLATE" | "BPLATE" => "T05P",
            "HALL" | "ROOM" | "CHAMBER" | "CONCERT" | "AMBI" => "T05R",
            "GATED" | "REVERSE" | "DEL/REV" | "SHIMMER" => "T05S",
            "PITCH" | "D-PITCH" | "DOUBLE" | "PCORR" | "SUB" => "T05T",
            "V-ROOM" | "V-REV" | "VSS3" => "T05V",
            _ => "T05Q",
        },
    }
}

/// Placeholder model: passes audio through unchanged and ignores parameters.
#[derive(Clone, Debug)]
pub struct StubModel {
    id: &'static str,
    task: &'static str,
}

impl StubModel {
    pub fn new(id: &'static str, task: &'static str) -> Self {
        Self { id, task }
    }
}

impl Model for StubModel {
    fn id(&self) -> &'static str {
        self.id
    }
    fn prepare(&mut self, _sample_rate: f64, _max_block: usize) {}
    fn process(&mut self, _l: &mut [f32], _r: &mut [f32], _sc: Option<(&[f32], &[f32])>) {
        // TODO(stub): replaced by the owning S4/S5 task, see `todo()`.
    }
    fn reset(&mut self) {}
    fn set_param(&mut self, _slot: u16, _value: ParamValue) {}
    fn is_stub(&self) -> bool {
        true
    }
    fn todo(&self) -> Option<&'static str> {
        Some(self.task)
    }
}

/// Create a model instance (allocates; off the RT thread). Every id of [`catalog`]`(kind)` is accepted
/// (implemented model or stub); ids unknown to models.md also get a stub (`todo() = "unknown"`) so a newer
/// schema never breaks registry construction.
pub fn create_model(kind: BlockKind, id: &'static str) -> Box<dyn Model> {
    match (kind, id) {
        (BlockKind::Dynamics, "GATE") => Box::new(GateModel::gate()),
        (BlockKind::Dynamics, "DUCK") => Box::new(GateModel::duck()),
        (BlockKind::Dynamics, "E88") => Box::new(VintageGate::new(VintageGateKind::E88)),
        (BlockKind::Dynamics, "9000G") => Box::new(VintageGate::new(VintageGateKind::Ssl9000)),
        (BlockKind::Dynamics, "D241G") => Box::new(VintageGate::new(VintageGateKind::D241)),
        (BlockKind::Dynamics, "DS902") => Box::new(DeEsser902::new()),
        (BlockKind::Dynamics, "DEQ") => Box::new(DynEq::single()),
        (BlockKind::Dynamics, "DEQ2") => Box::new(DynEq::dual()),
        (BlockKind::Dynamics, "COMP") => Box::new(DynModel::comp()),
        (BlockKind::Dynamics, "EXP") => Box::new(DynModel::exp()),
        (BlockKind::Dynamics, "B160") => Box::new(VcaComp::new(VcaKind::B160)),
        (BlockKind::Dynamics, "B560") => Box::new(VcaComp::new(VcaKind::B560)),
        (BlockKind::Dynamics, "D241C") => Box::new(VcaComp::new(VcaKind::D241C)),
        (BlockKind::Dynamics, "9000C") => Box::new(VcaComp::new(VcaKind::Ssl9000)),
        (BlockKind::Dynamics, "SBUS") => Box::new(VcaComp::new(VcaKind::SBus)),
        (BlockKind::Dynamics, "LA") => Box::new(OptoComp::new(OptoKind::La)),
        (BlockKind::Dynamics, "L100") => Box::new(OptoComp::new(OptoKind::L100)),
        (BlockKind::Dynamics, "F670") => Box::new(OptoComp::new(OptoKind::F670)),
        (BlockKind::Dynamics, "BLISS") => Box::new(OptoComp::new(OptoKind::Bliss)),
        (BlockKind::Dynamics, "ECL33") => Box::new(ConsoleComp::new(ConsoleKind::Ecl33)),
        (BlockKind::Dynamics, "E88C") => Box::new(ConsoleComp::new(ConsoleKind::E88c)),
        (BlockKind::Dynamics, "RED3") => Box::new(ConsoleComp::new(ConsoleKind::Red3)),
        (BlockKind::Dynamics, "2250") => Box::new(ConsoleComp::new(ConsoleKind::Api2250)),
        (BlockKind::Dynamics, "LMT") => Box::new(ConsoleComp::new(ConsoleKind::Lmt)),
        (BlockKind::Dynamics, "76LA") => Box::new(FetComp::new(FetKind::La76)),
        (BlockKind::Dynamics, "NSTR") => Box::new(FetComp::new(FetKind::Nstr)),
        (BlockKind::Dynamics, "ONEC") => Box::new(OneKnobComp::new()),
        (BlockKind::Dynamics, "3CMP") => Box::new(ThreeBandComp::new()),
        (BlockKind::Dynamics, "WAVE") => Box::new(TransientDesigner::new()),
        (BlockKind::Dynamics, "RIDE") => Box::new(AutoRider::new()),
        (BlockKind::Dynamics, "WARM") => Box::new(WarmthPre::new()),
        (BlockKind::Dynamics, "PSE") => Box::new(SourceExtractor::new(PseKind::Pse)),
        (BlockKind::Dynamics, "CMB") => Box::new(SourceExtractor::new(PseKind::Combo)),
        (BlockKind::Eq(v), "STD") => Box::new(StdEqModel::new(v)),
        (BlockKind::Eq(_), "SOUL") => Box::new(AltEq::new(AltKind::Soul)),
        (BlockKind::Eq(_), "E88") => Box::new(AltEq::new(AltKind::E88)),
        (BlockKind::Eq(_), "E84") => Box::new(AltEq::new(AltKind::E84)),
        (BlockKind::Eq(_), "F110") => Box::new(AltEq::new(AltKind::F110)),
        (BlockKind::Eq(_), "PULSAR") => Box::new(AltEq::new(AltKind::Pulsar)),
        (BlockKind::Eq(_), "MACH4") => Box::new(AltEq::new(AltKind::Mach4)),
        (BlockKind::Eq(_), "PIA") => Box::new(AltEq::new(AltKind::Pia)),
        (BlockKind::Fx, "ST-DL") => Box::new(crate::models::fx_delay::StereoDelay::new()),
        (BlockKind::FilterTool, "TILT") => Box::new(FilterToolModel::tilt()),
        (BlockKind::FilterTool, "MAX") => Box::new(FilterToolModel::maxer()),
        (BlockKind::FilterTool, "AP1") => Box::new(FilterToolModel::ap1()),
        (BlockKind::FilterTool, "AP2") => Box::new(FilterToolModel::ap2()),
        _ => {
            let task = if catalog(kind).contains(&id) {
                owning_task(kind, id)
            } else {
                "unknown"
            };
            Box::new(StubModel::new(id, task))
        }
    }
}

/// One preallocated instance per selectable model of a block; exactly one is active.
///
/// Construction and [`ModelRegistry::prepare`] allocate; [`ModelRegistry::select`] and everything else are
/// RT-safe. Inactive instances keep their parameters, so switching back restores the previous settings of
/// that model (the store re-sends slot values on a model switch anyway).
pub struct ModelRegistry {
    kind: BlockKind,
    ids: Vec<&'static str>,
    models: Vec<Box<dyn Model>>,
    active: usize,
}

impl ModelRegistry {
    /// Registry over `ids` (e.g. the model list of a schema group, in selector order).
    pub fn new(kind: BlockKind, ids: &[&'static str]) -> Self {
        assert!(!ids.is_empty(), "a model registry needs at least one model");
        Self {
            kind,
            ids: ids.to_vec(),
            models: ids.iter().map(|&id| create_model(kind, id)).collect(),
            active: 0,
        }
    }

    /// Registry over the full models.md catalog of `kind`.
    pub fn with_catalog(kind: BlockKind) -> Self {
        Self::new(kind, catalog(kind))
    }

    pub fn kind(&self) -> BlockKind {
        self.kind
    }

    /// Prepare every instance (allocates scratch; off the RT thread).
    pub fn prepare(&mut self, sample_rate: f64, max_block: usize) {
        for m in &mut self.models {
            m.prepare(sample_rate, max_block);
        }
    }

    pub fn len(&self) -> usize {
        self.models.len()
    }

    pub fn is_empty(&self) -> bool {
        self.models.is_empty()
    }

    pub fn ids(&self) -> &[&'static str] {
        &self.ids
    }

    pub fn position(&self, id: &str) -> Option<usize> {
        self.ids.iter().position(|&m| m == id)
    }

    pub fn active(&self) -> usize {
        self.active
    }

    pub fn active_id(&self) -> &'static str {
        self.ids[self.active]
    }

    /// Activate model `idx` (RT-safe: no allocation). The newly active instance is reset so it starts from a
    /// clean state. Returns false (and changes nothing) for an out-of-range index.
    pub fn select(&mut self, idx: usize) -> bool {
        if idx >= self.models.len() {
            return false;
        }
        if idx != self.active {
            self.active = idx;
            self.models[idx].reset();
        }
        true
    }

    /// [`ModelRegistry::select`] by id (linear search, RT-safe).
    pub fn select_id(&mut self, id: &str) -> bool {
        match self.position(id) {
            Some(i) => self.select(i),
            None => false,
        }
    }

    pub fn active_model(&self) -> &dyn Model {
        &*self.models[self.active]
    }

    pub fn active_mut(&mut self) -> &mut dyn Model {
        &mut *self.models[self.active]
    }

    /// Any instance (e.g. for a hot-swap crossfade that runs the old and the new model side by side).
    pub fn model_mut(&mut self, idx: usize) -> Option<&mut (dyn Model + 'static)> {
        self.models.get_mut(idx).map(|m| &mut **m)
    }

    /// Maximum latency over all instances (for fixed delay compensation).
    pub fn max_latency(&self) -> usize {
        self.models.iter().map(|m| m.latency()).max().unwrap_or(0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    #[rustfmt::skip]
    fn catalogs_create_and_stub_flags() {
        let implemented = [
            // One line per model family; append yours on its own line to keep merges clean.
            "GATE", "DUCK", "COMP", "EXP",
            "B160", "B560", "D241C", "9000C", "SBUS",
            "E88", "9000G", "D241G",
            "ECL33", "E88C", "RED3", "2250", "LMT",
            "LA", "L100", "F670", "BLISS",
            "DS902", "DEQ", "DEQ2",
            "ONEC", "3CMP",
            "PSE", "CMB",
            "GATE", "DUCK", "COMP", "EXP", "B160", "B560", "D241C", "9000C", "SBUS", "76LA", "NSTR",
            "WAVE", "RIDE", "WARM",
        ];
        for id in DYN_IDS {
            let m = create_model(BlockKind::Dynamics, id);
            assert_eq!(m.id(), id);
            assert_eq!(m.is_stub(), !implemented.contains(&id), "{id}");
            if m.is_stub() {
                assert!(m.todo().unwrap().starts_with('T'), "{id}");
            }
        }
        for id in EQ_IDS {
            let m = create_model(BlockKind::Eq(EqVariant::Channel), id);
            assert!(!m.is_stub(), "{id}");
        }
        for id in FILTER_TOOL_IDS {
            assert!(!create_model(BlockKind::FilterTool, id).is_stub());
        }
        let fx_implemented = [
            // One line per FX model family.
            "ST-DL",
        ];
        for id in FX_IDS {
            let m = create_model(BlockKind::Fx, id);
            assert_eq!(m.id(), id);
            assert_eq!(m.is_stub(), !fx_implemented.contains(&id), "{id}");
        }
        assert_eq!(create_model(BlockKind::Fx, "NEWFX").todo(), Some("unknown"));
    }

    #[test]
    fn stub_is_passthrough() {
        // Every dynamics model is implemented now; FX models are still stubs (T05x).
        let mut m = create_model(BlockKind::Fx, "HALL");
        m.prepare(48_000.0, 64);
        m.set_param(1, ParamValue::Float(12.0));
        let mut l: Vec<f32> = (0..64).map(|i| (i as f32 * 0.1).sin()).collect();
        let mut r = l.clone();
        let (l0, r0) = (l.clone(), r.clone());
        m.process(&mut l, &mut r, None);
        assert_eq!(l, l0);
        assert_eq!(r, r0);
        assert_eq!(m.latency(), 0);
    }

    #[test]
    fn registry_select() {
        let mut reg = ModelRegistry::with_catalog(BlockKind::Dynamics);
        reg.prepare(48_000.0, 64);
        assert_eq!(reg.len(), 33);
        assert_eq!(reg.active_id(), "GATE");
        assert!(reg.select_id("COMP"));
        assert_eq!(reg.active_id(), "COMP");
        assert!(!reg.active_model().is_stub());
        assert!(reg.select_id("WAVE"));
        assert!(!reg.active_model().is_stub());
        assert!(!reg.select(99));
        assert!(!reg.select_id("NOPE"));
        assert_eq!(reg.active_id(), "WAVE");
        // 76LA / NSTR: 2x oversampling (39) + control-path alignment (18).
        assert_eq!(reg.max_latency(), 57);
    }
}
