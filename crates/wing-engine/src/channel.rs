//! Channel and aux strips (T035): the WING input-strip processing up to the pre-fader point, built from
//! wing-blocks models and driven by the T03A parameter pipeline.
//!
//! Lives in wing-engine (not wing-blocks) because the strip is where DSP meets the engine: it binds to
//! wing-model schema paths ([`bind_channel_strip`] / [`bind_aux_strip`] via [`ParamMapBuilder`]), hosts
//! model slots through the engine's crossfading [`ModelSlot`], takes side-chain keys and insert FX from the
//! graph, and publishes telemetry. wing-blocks stays a schema-free DSP library.
//!
//! ```text
//! channel: src -> INPUT (trim/bal/inv/delay, M/ST/M-S) -TAP IN-> FILTER (LC -> HC -> tool slot) -TAP FILT->
//!          INSERT 1 -> [G E D I in `proc` order] (-TAP 3- -TAP 4- -TAP 5- after blocks 1..3) -TAP PFL-> out
//!          G = gate slot (any dyn model, key `gatesc`), E = PRE EQ (`peq`) -> EQ slot,
//!          D = dynamics slot (any dyn model, key `dynsc`), I = INSERT 2 (`postins`)
//! aux:     src -> INPUT -TAP IN-> INSERT -TAP 3-> EQ -TAP 4-> DYN -TAP 5/PFL-> out   (fixed order, no
//!          filter/gate/PRE EQ; FILT reads as IN)
//! ```
//! Mute/fader/pan/sends follow in T036 (taps AFL/POST are captured as PFL until then).
//!
//! * **Process order** (`proc`, 24 permutations of G/E/D/I, [`PROC_ORDERS`] = schema enum order) is switched
//!   at block boundaries (an order change is a discontinuity, as on the console).
//! * **Bypass**: every block has its own switch (`lc`/`hc`/`tf`, `gate.on`, `peq.on`, `eq.on`, `dyn.on`,
//!   `preins.on`/`postins.on`). Model slots crossfade on/off over 5 ms and cost nothing when off; LC/HC and
//!   PRE EQ fade internally; an insert that is off, unassigned (`NONE`) or in automix mode passes through.
//! * **Stereo-linked**: strips are always processed as stereo pairs (a mono source feeds both sides in the
//!   input stage); gate and dynamics models detect on the linked key and apply one gain to both sides.
//! * **Side-chain**: `gatesc`/`dynsc` `type f q` filter the key inside the slot; `src` != SELF makes the slot
//!   key from the external key buffers the graph provides (inputs 2/3 = gate key, 4/5 = dyn key; resolving
//!   `src`/`tap` to a buffer is routing's job). Without external buffers the slot keys from itself.
//! * **Telemetry** ([`StripTelemetry`]): peak level per tap point (peak-hold until polled), gate/dyn gain
//!   reduction; published through [`Processor::telemetry`] as `TAP_PEAK` (block = tap) and
//!   `GAIN_REDUCTION_DB` (block = [`block::GATE`] / [`block::DYN`]).
//! * **Meters** (T039, optional [`StripMeter`]): per block, the L/R peaks at taps IN and PFL, the gate and
//!   dynamics key peaks (the filtered side-chain when the slot forms one, else the slot input) and their
//!   gain reduction are folded into the engine's [`crate::meters::LevelBank`] (lock-free, no allocation).
//!   The post-fader words belong to the node that applies the fader.
//! * **Latency** (T03E, [`crate::latency`]): constant, independent of models and switches. Channel and aux
//!   strips share one latency (aux pads by the filter-tool and gate slot latency it does not run), applied
//!   by an alignment pad at the strip output ([`ChannelStrip::latency`], [`ChannelStrip::set_align_latency`],
//!   [`ChannelStrip::set_insert_latency`]).
//! * **Tap capture**: the signal at `ptap` is copied into [`ChannelStrip::tap_signal`] (and graph outputs
//!   2/3 when the node has 4 outputs) for PRE sends.

use wing_blocks::eq::{BandParams, PreEq, PreEqParams};
use wing_blocks::gate::ScFilter;
use wing_blocks::model::{BlockKind, Model, ParamValue};
use wing_blocks::std_models::{flt_slot, in_slot, FilterModel, InputModel};

use crate::graph::{NodeId, NodeIo, ProcessCtx, Processor};
use crate::latency::StripAlign;
use crate::meters::{abs_max, word, StripMeter};
use crate::params::std_models::model_factory;
use crate::params::strip::{ModelSlot, INPUT_LAYOUT, SLOT_LAYOUT};
use crate::params::telemetry_kind as kind;
use crate::params::{
    BindError, BlockLayout, ParamMapBuilder, ParamMsg, TelemetryWriter, TARGET_BASE,
};

/// Block indices used in [`ParamMsg`]s and telemetry.
pub mod block {
    pub const INPUT: u8 = 0;
    pub const FILTER: u8 = 1;
    pub const TOOL: u8 = 2;
    pub const GATE: u8 = 3;
    pub const GATE_SC: u8 = 4;
    pub const PEQ: u8 = 5;
    pub const EQ: u8 = 6;
    pub const DYN: u8 = 7;
    pub const DYN_SC: u8 = 8;
    pub const PREINS: u8 = 9;
    pub const POSTINS: u8 = 10;
    pub const STRIP: u8 = 11;
    pub const DYN_XO: u8 = 12;
}

/// `ch/N/flt` low/high cut.
pub const FILTER_LAYOUT: BlockLayout = BlockLayout {
    params: &[
        ("lc", flt_slot::LC),
        ("lcf", flt_slot::LCF),
        ("lcs", flt_slot::LCS),
        ("hc", flt_slot::HC),
        ("hcf", flt_slot::HCF),
        ("hcs", flt_slot::HCS),
    ],
    targets: &[],
};

/// `ch/N/flt` filter tool: `tf` (= slot on) + model group `mdl` with targets `1 2`.
pub const TOOL_LAYOUT: BlockLayout = BlockLayout {
    params: &[("tf", 0)],
    targets: &["1", "2"],
};

/// `gatesc` / `dynsc`.
pub const SC_LAYOUT: BlockLayout = BlockLayout {
    params: &[("type", 0), ("f", 1), ("q", 2), ("src", 3)],
    targets: &[],
};

/// `ch/N/dynxo`: dynamics crossover (T03C) `type` (OFF LO HI S6 S12 PEAK), `f`, `q`, `depth`.
pub const XO_LAYOUT: BlockLayout = BlockLayout {
    params: &[("type", 0), ("f", 1), ("q", 2), ("depth", 3)],
    targets: &[],
};

/// `ch/N/peq`: on + 3 bands g/f/q (param = 1 + 3*band + {0 g, 1 f, 2 q}).
pub const PEQ_LAYOUT: BlockLayout = BlockLayout {
    params: &[
        ("on", 0),
        ("1g", 1),
        ("1f", 2),
        ("1q", 3),
        ("2g", 4),
        ("2f", 5),
        ("2q", 6),
        ("3g", 7),
        ("3f", 8),
        ("3q", 9),
    ],
    targets: &[],
};

/// `ch/N/eq`, `aux/N/eq`: on, mix + the 20 STD slots (other models use a prefix of them).
pub const EQ_LAYOUT: BlockLayout = BlockLayout {
    params: &[("on", 0), ("mix", 1)],
    targets: &[
        "1", "2", "3", "4", "5", "6", "7", "8", "9", "10", "11", "12", "13", "14", "15", "16",
        "17", "18", "19", "20",
    ],
};

/// `preins`: on + FX slot (`ins` 0 = NONE, n = FXn).
pub const PREINS_LAYOUT: BlockLayout = BlockLayout {
    params: &[("on", 0), ("ins", 1)],
    targets: &[],
};

/// `ch/N/postins`: on, automix weight `w`; model group `mode` (FX / AUTO_X / AUTO_Y) with target `ins`.
pub const POSTINS_LAYOUT: BlockLayout = BlockLayout {
    params: &[("on", 0), ("w", 2)],
    targets: &["ins"],
};

/// `ch/N`: process order and tap point.
pub const STRIP_LAYOUT: BlockLayout = BlockLayout {
    params: &[("proc", 0), ("ptap", 1)],
    targets: &[],
};

/// Bind `ch/<ch>` (1-based) to a [`ChannelStrip`] at graph `node` and register it for telemetry.
pub fn bind_channel_strip(b: &mut ParamMapBuilder, ch: u32, node: NodeId) -> Result<(), BindError> {
    let p = format!("ch/{ch}");
    b.bind_block(&format!("{p}/in/set"), node, block::INPUT, &INPUT_LAYOUT)?;
    b.bind_block(&format!("{p}/flt"), node, block::FILTER, &FILTER_LAYOUT)?;
    b.bind_block(&format!("{p}/flt"), node, block::TOOL, &TOOL_LAYOUT)?;
    b.bind_block(&format!("{p}/gate"), node, block::GATE, &SLOT_LAYOUT)?;
    b.bind_block(&format!("{p}/gatesc"), node, block::GATE_SC, &SC_LAYOUT)?;
    b.bind_block(&format!("{p}/peq"), node, block::PEQ, &PEQ_LAYOUT)?;
    b.bind_block(&format!("{p}/eq"), node, block::EQ, &EQ_LAYOUT)?;
    b.bind_block(&format!("{p}/dyn"), node, block::DYN, &SLOT_LAYOUT)?;
    b.bind_block(&format!("{p}/dynsc"), node, block::DYN_SC, &SC_LAYOUT)?;
    b.bind_block(&format!("{p}/dynxo"), node, block::DYN_XO, &XO_LAYOUT)?;
    b.bind_block(&format!("{p}/preins"), node, block::PREINS, &PREINS_LAYOUT)?;
    b.bind_block(
        &format!("{p}/postins"),
        node,
        block::POSTINS,
        &POSTINS_LAYOUT,
    )?;
    b.bind_block(&p, node, block::STRIP, &STRIP_LAYOUT)?;
    b.telemetry_node(node);
    Ok(())
}

/// Bind `aux/<aux>` (1-based) to an aux [`ChannelStrip`] at graph `node` and register it for telemetry.
pub fn bind_aux_strip(b: &mut ParamMapBuilder, aux: u32, node: NodeId) -> Result<(), BindError> {
    let p = format!("aux/{aux}");
    b.bind_block(&format!("{p}/in/set"), node, block::INPUT, &INPUT_LAYOUT)?;
    b.bind_block(&format!("{p}/eq"), node, block::EQ, &EQ_LAYOUT)?;
    b.bind_block(&format!("{p}/dyn"), node, block::DYN, &SLOT_LAYOUT)?;
    b.bind_block(&format!("{p}/dynsc"), node, block::DYN_SC, &SC_LAYOUT)?;
    b.bind_block(&format!("{p}/preins"), node, block::PREINS, &PREINS_LAYOUT)?;
    b.telemetry_node(node);
    Ok(())
}

// ---- process order and taps -------------------------------------------------------------------------------

/// One position of the `proc` chain.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ProcBlock {
    Gate,
    Eq,
    Dyn,
    Insert,
}

/// `proc` tokens in schema enum order (signal-flow.md §2).
pub const PROC_ORDERS: [&str; 24] = [
    "GEDI", "GEID", "GIED", "IGED", "GDEI", "GDIE", "GIDE", "IGDE", "EGDI", "EGID", "EIGD", "IEGD",
    "EDGI", "EDIG", "EIDG", "IEDG", "DEGI", "DEIG", "DIEG", "IDEG", "DGEI", "DGIE", "DIGE", "IDGE",
];

/// A process order (permutation of the four chain blocks).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ProcOrder(pub [ProcBlock; 4]);

impl ProcOrder {
    /// Factory default `GEDI`.
    pub const DEFAULT: ProcOrder = ProcOrder([
        ProcBlock::Gate,
        ProcBlock::Eq,
        ProcBlock::Dyn,
        ProcBlock::Insert,
    ]);

    /// Parse a 4-letter token (`GEDI`); `None` unless it is a permutation of G, E, D, I.
    pub fn parse(s: &str) -> Option<Self> {
        let b = s.as_bytes();
        if b.len() != 4 {
            return None;
        }
        let mut out = [ProcBlock::Gate; 4];
        let mut seen = 0u8;
        for (o, c) in out.iter_mut().zip(b) {
            let (blk, bit) = match c {
                b'G' => (ProcBlock::Gate, 1),
                b'E' => (ProcBlock::Eq, 2),
                b'D' => (ProcBlock::Dyn, 4),
                b'I' => (ProcBlock::Insert, 8),
                _ => return None,
            };
            if seen & bit != 0 {
                return None;
            }
            seen |= bit;
            *o = blk;
        }
        Some(ProcOrder(out))
    }

    /// Order for the schema enum index of `proc` (out of range = default).
    pub fn from_index(i: usize) -> Self {
        PROC_ORDERS
            .get(i)
            .and_then(|s| Self::parse(s))
            .unwrap_or(Self::DEFAULT)
    }
}

/// Tap points (`ptap` / side-chain `tap` enum order: IN FILT 3 4 5 PFL AFL POST).
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
#[repr(u8)]
pub enum Tap {
    In = 0,
    Filt = 1,
    T3 = 2,
    T4 = 3,
    T5 = 4,
    Pfl = 5,
    /// Post fader (T036): captured as PFL until the fader exists.
    Afl = 6,
    /// Post processing (T036): captured as PFL until pan exists.
    Post = 7,
}

/// Number of tap points measured by the strip (IN .. PFL).
pub const STRIP_TAPS: usize = 6;

impl Tap {
    pub fn from_index(i: usize) -> Self {
        [
            Tap::In,
            Tap::Filt,
            Tap::T3,
            Tap::T4,
            Tap::T5,
            Tap::Pfl,
            Tap::Afl,
            Tap::Post,
        ]
        .get(i)
        .copied()
        .unwrap_or(Tap::T5)
    }

    /// The tap the strip actually captures for this setting (AFL/POST -> PFL for now).
    fn captured(self) -> usize {
        (self as usize).min(Tap::Pfl as usize)
    }
}

// ---- inserts ----------------------------------------------------------------------------------------------

/// Hosts insert effects (the FX rack, T050). The strip calls it for an engaged insert.
pub trait InsertHost {
    /// Process FX slot `fx` (1..=16) in place on the strip signal.
    fn process_insert(&mut self, fx: u8, l: &mut [f32], r: &mut [f32]);
}

/// No FX rack: every insert passes through.
pub struct NoInserts;

impl InsertHost for NoInserts {
    fn process_insert(&mut self, _fx: u8, _l: &mut [f32], _r: &mut [f32]) {}
}

/// Insert point state (placeholder until the FX rack exists: engaged inserts call the [`InsertHost`]).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct InsertPoint {
    pub on: bool,
    /// 0 = NONE, n = FXn.
    pub fx: u8,
    /// `postins.mode`: 0 FX, 1 AUTO_X, 2 AUTO_Y (automix: TODO, passes through).
    pub mode: u8,
    /// Automix weight `w` (dB).
    pub weight_db: f32,
}

impl InsertPoint {
    /// The FX slot to run, if the insert is engaged.
    pub fn engaged(&self) -> Option<u8> {
        (self.on && self.fx != 0 && self.mode == 0).then_some(self.fx)
    }
}

// ---- telemetry --------------------------------------------------------------------------------------------

/// Per-strip readout for meters.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct StripTelemetry {
    /// Linear peak per tap (IN, FILT, 3, 4, 5, PFL), max since the last [`ChannelStrip::take_telemetry`].
    pub tap_peak: [f32; STRIP_TAPS],
    pub gate_gr_db: f32,
    pub dyn_gr_db: f32,
}

/// Side-chain keys resolved by the engine (`src` != SELF).
#[derive(Clone, Copy, Debug, Default)]
pub struct StripKeys<'a> {
    pub gate: Option<(&'a [f32], &'a [f32])>,
    pub dyn_: Option<(&'a [f32], &'a [f32])>,
}

// ---- strip ------------------------------------------------------------------------------------------------

/// Channel (`ch.N`) or aux (`aux.N`) strip.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StripKind {
    Channel,
    Aux,
}

/// The assembled strip; see the module doc. Construct off the RT thread; everything else is RT-safe.
pub struct ChannelStrip {
    kind: StripKind,
    input: InputModel,
    filter: FilterModel,
    tool: ModelSlot,
    gate: ModelSlot,
    peq: PreEq,
    peq_p: PreEqParams,
    peq_dirty: bool,
    eq: ModelSlot,
    dyn_: ModelSlot,
    preins: InsertPoint,
    postins: InsertPoint,
    order: ProcOrder,
    ptap: Tap,
    gate_sc: (ScFilter, f32, f32, bool),
    dyn_sc: (ScFilter, f32, f32, bool),
    tap_l: Vec<f32>,
    tap_r: Vec<f32>,
    tap_len: usize,
    telem: StripTelemetry,
    meter: Option<StripMeter>,
    align: StripAlign,
}

fn sc_filter(i: u32) -> ScFilter {
    match i {
        1 => ScFilter::Lp12,
        2 => ScFilter::Hp12,
        3 => ScFilter::Bp,
        4 => ScFilter::Notch,
        _ => ScFilter::Off,
    }
}

/// Key level of a dynamics slot after processing: the filtered key when the slot formed one, else the
/// external key when one is selected, else the level of the slot input (`input_peak`).
#[inline]
pub(crate) fn slot_key_peak(
    slot: &ModelSlot,
    ext: Option<(&[f32], &[f32])>,
    input_peak: f32,
) -> f32 {
    let (kl, kr) = slot.key_signal();
    match ext {
        _ if !kl.is_empty() => abs_max(kl).max(abs_max(kr)),
        Some((a, b)) => abs_max(a).max(abs_max(b)),
        None => input_peak,
    }
}

impl ChannelStrip {
    /// A channel strip (all blocks off, factory model choices). Allocates.
    pub fn channel(fs: f64, max_block: usize) -> Self {
        Self::new(StripKind::Channel, fs, max_block)
    }

    /// An aux strip. Allocates.
    pub fn aux(fs: f64, max_block: usize) -> Self {
        Self::new(StripKind::Aux, fs, max_block)
    }

    pub fn new(kind: StripKind, fs: f64, max_block: usize) -> Self {
        let pre = match kind {
            StripKind::Channel => "ch",
            StripKind::Aux => "aux",
        };
        let slot = |blk: &str| {
            let key = format!("{pre}/*/{blk}");
            let k = crate::params::std_models::block_kind(&key).expect("model group");
            ModelSlot::from_group(&key, fs, max_block, model_factory(k, fs, max_block))
        };
        let mut input = InputModel::new();
        input.prepare(fs, max_block);
        let mut filter = FilterModel::new();
        filter.prepare(fs, max_block);
        let mut peq = PreEq::new();
        peq.prepare(fs, max_block);
        let mut eq = slot("eq");
        eq.set_mix_max(125.0);
        // WING EQ mix scales the band gains (T03C), it is not a dry/wet blend.
        eq.set_mix_in_model(true);
        let mut s = Self {
            kind,
            input,
            filter,
            // aux has no filter tool / gate: keep empty-capable slots so the struct is uniform
            tool: ModelSlot::from_group(
                "ch/*/flt",
                fs,
                max_block,
                model_factory(BlockKind::FilterTool, fs, max_block),
            ),
            gate: ModelSlot::from_group(
                "ch/*/gate",
                fs,
                max_block,
                model_factory(BlockKind::Dynamics, fs, max_block),
            ),
            peq,
            peq_p: PreEqParams::default(),
            peq_dirty: false,
            eq,
            dyn_: slot("dyn"),
            preins: InsertPoint::default(),
            postins: InsertPoint::default(),
            order: ProcOrder::DEFAULT,
            ptap: Tap::T5,
            gate_sc: (ScFilter::Off, 1000.0, 2.0, false),
            dyn_sc: (ScFilter::Off, 1000.0, 2.0, false),
            tap_l: vec![0.0; max_block.max(1)],
            tap_r: vec![0.0; max_block.max(1)],
            tap_len: 0,
            telem: StripTelemetry::default(),
            meter: None,
            align: StripAlign::new(0, 0),
        };
        let natural = s.tool.latency() + s.gate.latency() + s.eq.latency() + s.dyn_.latency();
        let own = match kind {
            StripKind::Channel => natural,
            StripKind::Aux => s.eq.latency() + s.dyn_.latency(),
        };
        s.align = StripAlign::new(own, natural);
        // Factory model choices (the pipeline's sync_all overrides them from the store).
        s.tool.select(0);
        s.gate.select(0);
        s.eq.select(0);
        let comp = if kind == StripKind::Channel { 13 } else { 0 };
        s.dyn_.select(comp);
        s.settle();
        s
    }

    pub fn kind(&self) -> StripKind {
        self.kind
    }

    /// Attach (or detach) the strip's meter cells. Call off the RT thread.
    pub fn set_meter(&mut self, m: Option<StripMeter>) {
        self.meter = m;
    }

    pub fn meter(&self) -> Option<&StripMeter> {
        self.meter.as_ref()
    }

    pub fn order(&self) -> ProcOrder {
        self.order
    }

    /// Set the process order directly (channel only; aux order is fixed).
    pub fn set_order(&mut self, o: ProcOrder) {
        self.order = o;
    }

    pub fn ptap(&self) -> Tap {
        self.ptap
    }

    pub fn input(&self) -> &InputModel {
        &self.input
    }
    pub fn filter(&self) -> &FilterModel {
        &self.filter
    }
    pub fn tool_slot(&self) -> &ModelSlot {
        &self.tool
    }
    pub fn gate_slot(&self) -> &ModelSlot {
        &self.gate
    }
    pub fn eq_slot(&self) -> &ModelSlot {
        &self.eq
    }
    pub fn dyn_slot(&self) -> &ModelSlot {
        &self.dyn_
    }
    pub fn gate_slot_mut(&mut self) -> &mut ModelSlot {
        &mut self.gate
    }
    pub fn eq_slot_mut(&mut self) -> &mut ModelSlot {
        &mut self.eq
    }
    pub fn dyn_slot_mut(&mut self) -> &mut ModelSlot {
        &mut self.dyn_
    }
    pub fn pre_eq_params(&self) -> &PreEqParams {
        &self.peq_p
    }
    pub fn inserts(&self) -> (InsertPoint, InsertPoint) {
        (self.preins, self.postins)
    }

    /// Total strip latency in samples (model slots + inserts + alignment pad). Constant: it does not
    /// depend on the selected models or block switches.
    pub fn latency(&self) -> usize {
        self.align.total()
    }

    /// Latency alignment state (own slot latency, insert latency, pad).
    pub fn align(&self) -> &StripAlign {
        &self.align
    }

    /// Align the strip output to `target` samples (`None` = the input-strip default: sum of the channel
    /// slot latencies). Returns the resulting total. RT-safe.
    pub fn set_align_latency(&mut self, target: Option<usize>) -> usize {
        self.align.set_target(target)
    }

    /// Latency added by the engaged inserts (FX rack); the alignment pad absorbs it. RT-safe.
    pub fn set_insert_latency(&mut self, n: usize) -> usize {
        self.align.set_insert_latency(n)
    }

    /// Commit everything and jump all smoothers/fades to their targets (tests, offline renders).
    pub fn settle(&mut self) {
        self.apply(&ParamMsg::Commit);
        Model::reset(&mut self.input);
        Model::reset(&mut self.filter);
        self.peq.reset();
        for s in [&mut self.tool, &mut self.gate, &mut self.eq, &mut self.dyn_] {
            s.settle();
        }
        self.align.reset();
    }

    fn set(&mut self, blk: u8, param: u16, v: f32) {
        let pv = ParamValue::Float(v);
        match blk {
            block::INPUT => {
                let slot = match param {
                    0 => in_slot::TRIM,
                    1 => in_slot::BAL,
                    2 => in_slot::INV,
                    3 => in_slot::DLYON,
                    4 => in_slot::MODE,
                    5 => in_slot::GAIN,
                    TARGET_BASE => in_slot::DLY,
                    _ => return,
                };
                Model::set_param(&mut self.input, slot, pv);
            }
            block::FILTER => Model::set_param(&mut self.filter, param, pv),
            block::TOOL => {
                if param >= TARGET_BASE {
                    self.tool.set_target(param - TARGET_BASE, v);
                } else {
                    self.tool.set_wrapper(0, v);
                }
            }
            block::GATE | block::EQ | block::DYN => {
                let s = match blk {
                    block::GATE => &mut self.gate,
                    block::EQ => &mut self.eq,
                    _ => &mut self.dyn_,
                };
                if param >= TARGET_BASE {
                    s.set_target(param - TARGET_BASE, v);
                } else {
                    s.set_wrapper(param, v);
                }
            }
            block::GATE_SC | block::DYN_SC => {
                let (sc, slot) = if blk == block::GATE_SC {
                    (&mut self.gate_sc, &mut self.gate)
                } else {
                    (&mut self.dyn_sc, &mut self.dyn_)
                };
                match param {
                    0 => sc.0 = sc_filter(v as u32),
                    1 => sc.1 = v,
                    2 => sc.2 = v,
                    3 => sc.3 = v as u32 != 0,
                    _ => return,
                }
                slot.set_key_filter(sc.0, sc.1, sc.2);
            }
            block::DYN_XO => {
                let mut x = *self.dyn_.xover();
                match param {
                    0 => {
                        use wing_blocks::sidechain::XoverKind as K;
                        x.kind = [K::Off, K::Lo, K::Hi, K::S6, K::S12, K::Peak]
                            .get(v as usize)
                            .copied()
                            .unwrap_or(K::Off)
                    }
                    1 => x.freq = v,
                    2 => x.q = v,
                    3 => x.depth_db = v,
                    _ => return,
                }
                self.dyn_.set_xover(&x);
            }
            block::PEQ => {
                let p = &mut self.peq_p;
                match param {
                    0 => p.on = v != 0.0,
                    1..=9 => {
                        let b: &mut BandParams = &mut p.bands[(param as usize - 1) / 3];
                        match (param - 1) % 3 {
                            0 => b.gain_db = v,
                            1 => b.freq = v,
                            _ => b.q = v,
                        }
                    }
                    _ => return,
                }
                self.peq_dirty = true;
            }
            block::PREINS | block::POSTINS => {
                let ins = if blk == block::PREINS {
                    &mut self.preins
                } else {
                    &mut self.postins
                };
                match param {
                    0 => ins.on = v != 0.0,
                    1 | TARGET_BASE => ins.fx = v as u8,
                    2 => ins.weight_db = v,
                    _ => {}
                }
            }
            block::STRIP => match param {
                0 if self.kind == StripKind::Channel => {
                    self.order = ProcOrder::from_index(v as usize)
                }
                1 => self.ptap = Tap::from_index(v as usize),
                _ => {}
            },
            _ => {}
        }
    }

    fn select(&mut self, blk: u8, model: u16) {
        match blk {
            block::INPUT => Model::set_param(
                &mut self.input,
                in_slot::DLYMODE,
                ParamValue::Int(model as i32),
            ),
            block::TOOL => self.tool.select(model),
            block::GATE => self.gate.select(model),
            block::EQ => self.eq.select(model),
            block::DYN => self.dyn_.select(model),
            block::POSTINS => self.postins.mode = model as u8,
            _ => {}
        }
    }

    /// Apply a pipeline message (same as [`Processor::set_param`]). RT-safe.
    pub fn apply(&mut self, msg: &ParamMsg) {
        match *msg {
            ParamMsg::Set {
                block,
                param,
                value,
            } => self.set(block, param, value),
            ParamMsg::Model { block, model } => self.select(block, model),
            ParamMsg::Commit => {
                Model::commit(&mut self.input);
                Model::commit(&mut self.filter);
                if self.peq_dirty {
                    self.peq_dirty = false;
                    self.peq.set_params(&self.peq_p);
                }
                self.tool.commit();
                self.gate.commit();
                self.eq.commit();
                self.dyn_.commit();
            }
        }
    }

    /// Signal captured at `ptap` during the last `process` call.
    pub fn tap_signal(&self) -> (&[f32], &[f32]) {
        (&self.tap_l[..self.tap_len], &self.tap_r[..self.tap_len])
    }

    /// Current telemetry (peaks accumulate until [`ChannelStrip::take_telemetry`]).
    pub fn telemetry(&self) -> StripTelemetry {
        self.telem
    }

    /// Read and clear the peak-hold telemetry.
    pub fn take_telemetry(&mut self) -> StripTelemetry {
        let t = self.telem;
        self.telem.tap_peak = [0.0; STRIP_TAPS];
        t
    }

    #[inline]
    fn tap(&mut self, t: usize, l: &[f32], r: &[f32]) {
        let (pl, pr) = (abs_max(l), abs_max(r));
        let p = &mut self.telem.tap_peak[t];
        *p = p.max(pl).max(pr);
        if let Some(m) = &self.meter {
            if t == Tap::In as usize {
                m.level(word::IN_L, pl);
                m.level(word::IN_R, pr);
            } else if t == Tap::Pfl as usize {
                m.level(word::PRE_L, pl);
                m.level(word::PRE_R, pr);
            }
        }
        if self.ptap.captured() == t {
            let n = l.len().min(self.tap_l.len());
            self.tap_l[..n].copy_from_slice(&l[..n]);
            self.tap_r[..n].copy_from_slice(&r[..n]);
            self.tap_len = n;
        }
    }

    /// Process one stereo block in place without external keys or FX.
    pub fn process_stereo(&mut self, l: &mut [f32], r: &mut [f32]) {
        self.process_with(l, r, StripKeys::default(), &mut NoInserts);
    }

    /// Process one stereo block (`len <= max_block`) in place. RT-safe.
    pub fn process_with(
        &mut self,
        l: &mut [f32],
        r: &mut [f32],
        keys: StripKeys<'_>,
        fx: &mut dyn InsertHost,
    ) {
        debug_assert_eq!(l.len(), r.len());
        Model::process(&mut self.input, l, r, None);
        self.tap(Tap::In as usize, l, r);
        let gate_key = keys.gate.filter(|_| self.gate_sc.3);
        let dyn_key = keys.dyn_.filter(|_| self.dyn_sc.3);
        let metered = self.meter.is_some();
        let (mut gate_in, mut dyn_in) = (0.0f32, 0.0f32);
        match self.kind {
            StripKind::Channel => {
                if !self.filter.is_off() {
                    Model::process(&mut self.filter, l, r, None);
                }
                self.tool.process(l, r);
                self.tap(Tap::Filt as usize, l, r);
                if let Some(n) = self.preins.engaged() {
                    fx.process_insert(n, l, r);
                }
                let order = self.order.0;
                for (k, b) in order.into_iter().enumerate() {
                    match b {
                        ProcBlock::Gate => {
                            if metered {
                                gate_in = abs_max(l).max(abs_max(r));
                            }
                            self.gate.process_keyed(l, r, gate_key)
                        }
                        ProcBlock::Eq => {
                            self.peq.process(l, r);
                            self.eq.process(l, r);
                        }
                        ProcBlock::Dyn => {
                            if metered {
                                dyn_in = abs_max(l).max(abs_max(r));
                            }
                            self.dyn_.process_keyed(l, r, dyn_key)
                        }
                        ProcBlock::Insert => {
                            if let Some(n) = self.postins.engaged() {
                                fx.process_insert(n, l, r);
                            }
                        }
                    }
                    self.tap(Tap::T3 as usize + k, l, r);
                }
            }
            StripKind::Aux => {
                self.tap(Tap::Filt as usize, l, r);
                if let Some(n) = self.preins.engaged() {
                    fx.process_insert(n, l, r);
                }
                self.tap(Tap::T3 as usize, l, r);
                self.eq.process(l, r);
                self.tap(Tap::T4 as usize, l, r);
                if metered {
                    dyn_in = abs_max(l).max(abs_max(r));
                }
                self.dyn_.process_keyed(l, r, dyn_key);
                self.tap(Tap::T5 as usize, l, r);
                self.tap(Tap::Pfl as usize, l, r);
            }
        }
        self.align.process(l, r);
        self.telem.gate_gr_db = self.gate.gain_reduction_db();
        self.telem.dyn_gr_db = self.dyn_.gain_reduction_db();
        if let Some(m) = &self.meter {
            if self.kind == StripKind::Channel {
                m.level(word::GATE_KEY, slot_key_peak(&self.gate, gate_key, gate_in));
                m.level(word::GATE_GR, self.telem.gate_gr_db);
            }
            m.level(word::DYN_KEY, slot_key_peak(&self.dyn_, dyn_key, dyn_in));
            m.level(word::DYN_GR, self.telem.dyn_gr_db);
        }
    }
}

impl Processor for ChannelStrip {
    /// Inputs: 0 = L (or mono), 1 = R, 2/3 = gate key L/R, 4/5 = dyn key L/R (used when the slot's
    /// `src` != SELF). Outputs: 0/1 = strip out (PFL), optional 2/3 = `ptap` signal.
    fn process(&mut self, _ctx: &ProcessCtx, io: &mut NodeIo<'_>) {
        let ins = io.inputs();
        let b = io.block_size();
        let n_out = io.num_outputs();
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
        let key = |i: usize| (ins.len() > i + 1).then(|| (ins.get(i), ins.get(i + 1)));
        let keys = StripKeys {
            gate: key(2),
            dyn_: key(4),
        };
        self.process_with(l, r, keys, &mut NoInserts);
        if n_out >= 4 {
            let (tl, tr) = (
                &self.tap_l[..b.min(self.tap_len)],
                &self.tap_r[..b.min(self.tap_len)],
            );
            let out = io.outputs();
            out[2 * b..2 * b + tl.len()].copy_from_slice(tl);
            out[3 * b..3 * b + tr.len()].copy_from_slice(tr);
        }
    }

    fn set_param(&mut self, msg: &ParamMsg) {
        self.apply(msg);
    }

    fn telemetry(&mut self, out: &mut TelemetryWriter<'_>) {
        let t = self.take_telemetry();
        out.push(block::GATE, kind::GAIN_REDUCTION_DB, t.gate_gr_db);
        out.push(block::DYN, kind::GAIN_REDUCTION_DB, t.dyn_gr_db);
        for (i, p) in t.tap_peak.iter().enumerate() {
            out.push(i as u8, kind::TAP_PEAK, *p);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use wing_model::Schema;

    #[test]
    fn proc_orders_match_schema() {
        let s = Schema::get();
        let id = s.by_path("ch/1/proc").unwrap();
        let d = s.base_desc(id).unwrap();
        let toks: Vec<&str> = s.enum_items(d).iter().map(|e| e.token).collect();
        assert_eq!(toks, PROC_ORDERS);
        for t in PROC_ORDERS {
            assert!(ProcOrder::parse(t).is_some(), "{t}");
        }
        assert_eq!(ProcOrder::parse("GGDI"), None);
        assert_eq!(ProcOrder::from_index(0), ProcOrder::DEFAULT);
        let tap = s.base_desc(s.by_path("ch/1/ptap").unwrap()).unwrap();
        let toks: Vec<&str> = s.enum_items(tap).iter().map(|e| e.token).collect();
        assert_eq!(toks, ["IN", "FILT", "3", "4", "5", "PFL", "AFL", "POST"]);
    }

    #[test]
    fn default_strip_is_transparent() {
        for mut s in [
            ChannelStrip::channel(48_000.0, 64),
            ChannelStrip::aux(48_000.0, 64),
        ] {
            // transparent up to the strip's constant latency (T03E)
            let lat = s.latency();
            let x: Vec<f32> = (0..512).map(|i| (i as f32 * 0.3).sin() * 0.5).collect();
            let (mut l, mut r) = (x.clone(), x.clone());
            for (a, b) in l.chunks_mut(64).zip(r.chunks_mut(64)) {
                s.process_stereo(a, b);
            }
            assert!(l[..lat].iter().all(|&v| v == 0.0));
            assert_eq!(l[lat..], x[..512 - lat]);
            assert_eq!(r[lat..], x[..512 - lat]);
            assert_eq!(
                s.dyn_slot().model_index(),
                Some(if s.kind() == StripKind::Channel {
                    13
                } else {
                    0
                })
            );
        }
    }
}
