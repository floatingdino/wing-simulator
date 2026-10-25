//! Bus / main / matrix strip processing (T037): the [`StripHook`] that fills the slot between a
//! summing node's send sum and its fader, plus the output delay after pan.
//!
//! ```text
//! sum -> INPUT (trim, bal, inv) -TAP IN-> INSERT 1 -TAP INS1-> EQ slot -TAP EQ-> DYN slot -TAP DYN->
//!        INSERT 2 -TAP PFL-> [MixStripNode: mute, fader, width, pan, busmono] -> OUT DELAY -TAP POST->
//! ```
//!
//! * **Order** is fixed (signal-flow.md §4/§5 [I]): the schema exposes no `proc` key for
//!   `bus`/`main`/`mtx`, so there are no process-order options to bind.
//! * **EQ slot** (`<bus|main|mtx>/N/eq`, model group `<kind>/*/eq`): STD bus variant (L + 6 bands + H +
//!   tilt, exact WING coefficients) and the alternate EQ models of the registry; busses offer PIA where
//!   channels offer MACH4 (the group's model list decides). `mix` 0..125 % scales the band gains.
//! * **Dynamics slot** (`dyn`, any model of the shared gate/dynamics table), key filter `dynsc
//!   type/f/q`, crossover `dynxo`. The key is the strip itself (SELF) or, for `dynsc.src` != SELF, the
//!   `dynsc.tap` of another bus/main/matrix/aux read from the key bank one block late (T080,
//!   [`crate::mix::keys`]); `dynsc.$solo` (key listen) publishes the detected key for the monitor. The
//!   strip publishes its own taps (IN, INS1, EQ, DYN, PFL) for other strips' keys.
//! * **Inserts** (`preins`/`postins` `on ins`) run their FX slot inline through the strip's
//!   [`InsertHost`] (the FX rack handle, [`crate::fx`]; T050/T044).
//! * **Output delay** (`dly on`, model group `dly/mode` M/FT/MS/SMP with target `dly`) runs on the
//!   POST signal ([`StripHook::process_post`]) so PRE taps/sends stay undelayed. It is the
//!   wing-blocks input-stage delay (50 ms glide, linear interpolation, settled integer delays exact).
//! * **Latency** (T03E, [`crate::latency`]): constant (EQ + dynamics slot latency, independent of models
//!   and switches), plus an alignment pad before the fader ([`MixStrip::set_align_latency`],
//!   [`MixStrip::set_insert_latency`]); the output delay comes on top.
//! * **Stereo-linked**: one stereo pair; dynamics models detect on the linked key.
//! * **Bypass**: every block has its own switch; model slots crossfade on/off and cost nothing when
//!   off; trim at unity is skipped.
//! * **Meters** (T039, optional [`StripMeter`]): per block the IN word (after trim, L/R peaks), the
//!   dynamics key peak and GR go to the engine's level bank (PRE/OUT come from the graph's port
//!   meters, [`crate::meters::PortMeters::for_mix`]). Bus/main/matrix have no gate: GATE words stay 0.
//! * **Telemetry**: `TAP_PEAK` per [`MixTap`] (block = tap index) and `GAIN_REDUCTION_DB` at
//!   [`hook::DYN`].

use wing_blocks::input::{DelayMode, InputParams, InputStage, SourceMode};
use wing_blocks::sidechain::XoverKind;

use crate::channel::{
    claimed_latency, listen_post, listen_pre, slot_key_peak, sync_inserts, InsertHost, InsertPoint,
    InsertPos, NoInserts, SideChain, PREINS_LAYOUT, SC_LAYOUT, XO_LAYOUT,
};
use crate::graph::NodeId;
use crate::latency::StripAlign;
use crate::meters::{abs_max, word, StripMeter};
use crate::params::std_models::{block_kind, model_factory};
use crate::params::strip::{ModelSlot, MODEL_XFADE_S, SLOT_LAYOUT};
use crate::params::telemetry_kind as kind;
use crate::params::{
    BindError, BlockLayout, ParamMapBuilder, ParamMsg, TelemetryWriter, TARGET_BASE,
};
use crate::routing::Strip;

use super::keys::{key_route, slot as key_slot, KeyBank, KeyFamily, KeyIn, KeyOut};
use super::law::{self, Ramp};
use super::nodes::StripHook;
use super::{block, MixConfig};

/// Hook block ids (all `>=` [`block::HOOK_BASE`]).
pub mod hook {
    use super::block::HOOK_BASE;
    pub const INPUT: u8 = HOOK_BASE;
    pub const PREINS: u8 = HOOK_BASE + 1;
    pub const EQ: u8 = HOOK_BASE + 2;
    pub const DYN: u8 = HOOK_BASE + 3;
    pub const DYN_SC: u8 = HOOK_BASE + 4;
    pub const DYN_XO: u8 = HOOK_BASE + 5;
    pub const POSTINS: u8 = HOOK_BASE + 6;
    pub const DLY: u8 = HOOK_BASE + 7;
}

/// `<strip>/N/in/set`: trim, balance, polarity.
pub const MIX_INPUT_LAYOUT: BlockLayout = BlockLayout {
    params: &[("trim", 0), ("bal", 1), ("inv", 2)],
    targets: &[],
};

/// `<strip>/N/eq`: on, mix + the 27 bus STD slots (other models use a prefix of them).
pub const MIX_EQ_LAYOUT: BlockLayout = BlockLayout {
    params: &[("on", 0), ("mix", 1)],
    targets: &[
        "1", "2", "3", "4", "5", "6", "7", "8", "9", "10", "11", "12", "13", "14", "15", "16",
        "17", "18", "19", "20", "21", "22", "23", "24", "25", "26", "27",
    ],
};

/// `<strip>/N/dly`: on + model group `mode` (M FT MS SMP) with target `dly`.
pub const DLY_LAYOUT: BlockLayout = BlockLayout {
    params: &[("on", 0)],
    targets: &["dly"],
};

/// Tap points of a bus/main/matrix strip (telemetry block index).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum MixTap {
    /// After trim/balance/polarity.
    In = 0,
    /// After insert 1.
    Ins1 = 1,
    /// After the EQ slot.
    Eq = 2,
    /// After the dynamics slot.
    Dyn = 3,
    /// After insert 2 (= pre fader).
    Pfl = 4,
    /// Strip output after fader, pan and output delay.
    Post = 5,
}

/// Number of [`MixTap`]s.
pub const MIX_TAPS: usize = 6;

/// Per-strip readout for meters.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct MixStripTelemetry {
    /// Linear peak per [`MixTap`], max since the last [`MixStrip::take_telemetry`].
    pub tap_peak: [f32; MIX_TAPS],
    pub dyn_gr_db: f32,
}

#[inline]
fn peak(l: &[f32], r: &[f32]) -> f32 {
    let mut p = 0.0f32;
    for (a, b) in l.iter().zip(r) {
        p = p.max(a.abs()).max(b.abs());
    }
    p
}

/// Bus/main/matrix strip processing; see the module doc. Construct off the RT thread.
pub struct MixStrip {
    strip: Strip,
    trim: InputParams,
    trim_dirty: bool,
    /// Per-side linear gain (trim, balance, polarity), ramped per block.
    gain: Ramp<2>,
    preins: InsertPoint,
    eq: ModelSlot,
    dyn_: ModelSlot,
    dyn_sc: SideChain,
    /// External dynamics key (T080); `None` until [`StripHook::set_keys`].
    key_in: Option<KeyIn>,
    /// Key bank outputs (taps, key listen).
    key_out: Option<KeyOut>,
    postins: InsertPoint,
    dly: InputStage,
    dly_p: InputParams,
    dly_dirty: bool,
    fx: Box<dyn InsertHost + Send>,
    /// Insert state last forwarded to `fx`.
    ins_seen: (InsertPoint, InsertPoint),
    telem: MixStripTelemetry,
    meter: Option<StripMeter>,
    align: StripAlign,
}

impl MixStrip {
    /// Strip processing for `strip` (bus, main or matrix), all blocks off. Allocates.
    pub fn new(strip: Strip, fs: f64, max_block: usize) -> Self {
        let pre = match strip {
            Strip::Bus(_) => "bus",
            Strip::Main(_) => "main",
            Strip::Mtx(_) => "mtx",
            _ => panic!("MixStrip for {strip:?}"),
        };
        let slot = |blk: &str| {
            let key = format!("{pre}/*/{blk}");
            let k = block_kind(&key).expect("model group");
            ModelSlot::from_group(&key, fs, max_block, model_factory(k, fs, max_block))
        };
        let mut eq = slot("eq");
        eq.set_mix_max(125.0);
        eq.set_mix_in_model(true);
        let mut dyn_ = slot("dyn");
        eq.select(0);
        dyn_.select(13); // COMP
        let mut dly = InputStage::new();
        let dly_p = InputParams {
            source_mode: SourceMode::Stereo,
            ..InputParams::default()
        };
        dly.set_params(&dly_p);
        dly.prepare(fs, max_block);
        // own: the model slots (constant per slot, T03E); natural: the family worst case
        let mut align = StripAlign::new(
            eq.latency() + dyn_.latency(),
            eq.max_latency() + dyn_.max_latency(),
        );
        align.set_fade_samples((MODEL_XFADE_S * fs).round() as usize);
        let trim = InputParams {
            source_mode: SourceMode::Stereo,
            ..InputParams::default()
        };
        let a = law::block_alpha(law::MIX_TAU_S, fs, max_block);
        let mut gain = Ramp::new(a);
        gain.set([1.0, 1.0]);
        let mut s = Self {
            strip,
            trim,
            trim_dirty: false,
            gain,
            preins: InsertPoint::default(),
            eq,
            dyn_,
            dyn_sc: SideChain::default(),
            key_in: None,
            key_out: None,
            postins: InsertPoint::default(),
            dly,
            dly_p,
            dly_dirty: false,
            fx: Box::new(NoInserts),
            ins_seen: Default::default(),
            telem: MixStripTelemetry::default(),
            meter: None,
            align,
        };
        s.settle();
        s
    }

    /// Hook factory for [`MixConfig::hooks`]: a [`MixStrip`] per strip.
    pub fn factory(cfg: &MixConfig) -> super::HookFactory {
        let (fs, bs) = (cfg.sample_rate as f64, cfg.block_size);
        Box::new(move |s| Box::new(MixStrip::new(s, fs, bs)) as Box<dyn StripHook>)
    }

    pub fn strip(&self) -> Strip {
        self.strip
    }
    pub fn eq_slot(&self) -> &ModelSlot {
        &self.eq
    }
    pub fn dyn_slot(&self) -> &ModelSlot {
        &self.dyn_
    }
    pub fn eq_slot_mut(&mut self) -> &mut ModelSlot {
        &mut self.eq
    }
    pub fn dyn_slot_mut(&mut self) -> &mut ModelSlot {
        &mut self.dyn_
    }
    pub fn inserts(&self) -> (InsertPoint, InsertPoint) {
        (self.preins, self.postins)
    }
    /// Output delay parameters (`delay_on`, `delay`, `delay_mode`).
    pub fn delay_params(&self) -> &InputParams {
        &self.dly_p
    }

    /// Attach (or detach) the strip's meter cells. Call off the RT thread.
    pub fn set_meter(&mut self, m: Option<StripMeter>) {
        self.meter = m;
    }

    pub fn meter(&self) -> Option<&StripMeter> {
        self.meter.as_ref()
    }

    /// Install the FX rack (off the RT thread); the current insert assignment is forwarded to it.
    pub fn set_insert_host(&mut self, fx: Box<dyn InsertHost + Send>) {
        self.fx = fx;
        self.ins_seen = Default::default();
        sync_inserts(
            &mut *self.fx,
            &mut self.ins_seen,
            (self.preins, self.postins),
        );
        self.fx.set_own_latency(self.own_latency());
    }

    /// Latency of the EQ and dynamics slots (constant per slot, T03E).
    pub fn own_latency(&self) -> usize {
        self.eq.latency() + self.dyn_.latency()
    }

    /// Strip latency in samples (model slots + inserts + alignment pad; output delay excluded) = the
    /// alignment target: the family worst case standalone, the engine family target under an FX rack
    /// (T05U). It does not depend on block switches.
    pub fn latency(&self) -> usize {
        self.align.total()
    }

    /// Latency alignment state.
    pub fn align(&self) -> &StripAlign {
        &self.align
    }

    /// Align the pre-fader output to `target` samples (`None` = the engine family target, else the EQ +
    /// dynamics slots' maximum latency). RT-safe.
    pub fn set_align_latency(&mut self, target: Option<usize>) -> usize {
        self.align.set_target(target)
    }

    /// Latency added by the engaged inserts (FX rack); the alignment pad absorbs it. RT-safe.
    pub fn set_insert_latency(&mut self, n: usize) -> usize {
        self.align.set_insert_latency(n)
    }

    /// Commit and jump every ramp/fade/glide to its target (tests, scene start).
    pub fn settle(&mut self) {
        self.apply(&ParamMsg::Commit);
        self.gain.settle();
        self.eq.settle();
        self.dyn_.settle();
        self.dly.reset();
        let own = self.own_latency();
        self.align.set_own_latency(own);
        self.align.reset();
    }

    pub fn telemetry(&self) -> MixStripTelemetry {
        self.telem
    }

    /// Read and clear the peak-hold telemetry.
    pub fn take_telemetry(&mut self) -> MixStripTelemetry {
        let t = self.telem;
        self.telem.tap_peak = [0.0; MIX_TAPS];
        t
    }

    #[inline]
    fn tap(&mut self, t: MixTap, l: &[f32], r: &[f32]) {
        let p = &mut self.telem.tap_peak[t as usize];
        *p = p.max(peak(l, r));
        // key taps IN..PFL (= MixTap index); the node publishes AFL itself (POST is delayed output)
        if let Some(k) = self.key_out.as_ref().filter(|_| t != MixTap::Post) {
            k.tap(t as usize, l, r);
        }
    }

    /// Dynamics side-chain settings (`dynsc`).
    pub fn dyn_sc(&self) -> &SideChain {
        &self.dyn_sc
    }

    /// Route the dynamics key through `bank` as strip index `idx` and publish the strip's taps (off the
    /// RT thread).
    pub fn set_keys(&mut self, bank: KeyBank, idx: usize) {
        self.key_out = Some(KeyOut::new(bank.clone(), idx));
        self.key_in = Some(KeyIn::new(bank));
        self.sync_keys();
    }

    /// Follow `dynsc.src`/`tap` (between cycles).
    fn sync_keys(&mut self) {
        if let Some(k) = self.key_in.as_mut() {
            k.route(key_route(KeyFamily::Mix, self.dyn_sc.src, self.dyn_sc.tap));
            k.sync();
        }
    }

    fn set(&mut self, blk: u8, param: u16, v: f32) {
        match blk {
            hook::INPUT => {
                match param {
                    0 => self.trim.trim_db = v,
                    1 => self.trim.balance_db = v,
                    2 => self.trim.invert = v != 0.0,
                    _ => return,
                }
                self.trim_dirty = true;
            }
            hook::EQ | hook::DYN => {
                let s = if blk == hook::EQ {
                    &mut self.eq
                } else {
                    &mut self.dyn_
                };
                if param >= TARGET_BASE {
                    s.set_target(param - TARGET_BASE, v);
                } else {
                    s.set_wrapper(param, v);
                }
            }
            hook::DYN_SC => {
                let sc = &mut self.dyn_sc;
                if sc.set(param, v) {
                    self.dyn_.set_key_filter(sc.filter, sc.freq, sc.q);
                }
            }
            hook::DYN_XO => {
                let mut x = *self.dyn_.xover();
                match param {
                    0 => {
                        use XoverKind as K;
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
            hook::PREINS | hook::POSTINS => {
                let ins = if blk == hook::PREINS {
                    &mut self.preins
                } else {
                    &mut self.postins
                };
                match param {
                    0 => ins.on = v != 0.0,
                    1 => ins.fx = v.clamp(0.0, 16.0) as u8,
                    _ => {}
                }
                // mode FX only (bus/main/mtx inserts have no automix)
                ins.mode = 0;
            }
            hook::DLY => {
                match param {
                    0 => self.dly_p.delay_on = v != 0.0,
                    TARGET_BASE => self.dly_p.delay = v,
                    _ => return,
                }
                self.dly_dirty = true;
            }
            _ => {}
        }
    }

    fn select(&mut self, blk: u8, model: u16) {
        match blk {
            hook::EQ => self.eq.select(model),
            hook::DYN => self.dyn_.select(model),
            hook::DLY => {
                self.dly_p.delay_mode = match model {
                    1 => DelayMode::Feet,
                    2 => DelayMode::Ms,
                    3 => DelayMode::Samples,
                    _ => DelayMode::Meters,
                };
                self.dly_dirty = true;
            }
            _ => {}
        }
    }

    /// Apply a pipeline message (RT-safe).
    pub fn apply(&mut self, msg: &ParamMsg) {
        match *msg {
            ParamMsg::Set {
                block,
                param,
                value,
            } => self.set(block, param, value),
            ParamMsg::Model { block, model } => self.select(block, model),
            ParamMsg::Commit => {
                if self.trim_dirty {
                    self.trim_dirty = false;
                    let (l, r) = self.trim.side_gains();
                    self.gain.set([l, r]);
                }
                if self.dly_dirty {
                    self.dly_dirty = false;
                    self.dly.set_params(&self.dly_p);
                }
                self.eq.commit();
                self.dyn_.commit();
                sync_inserts(
                    &mut *self.fx,
                    &mut self.ins_seen,
                    (self.preins, self.postins),
                );
                let own = self.own_latency();
                self.fx.set_own_latency(own);
                self.sync_keys();
            }
        }
    }

    /// Process the summed signal up to the pre-fader point, in place (RT-safe).
    pub fn process_pre(&mut self, l: &mut [f32], r: &mut [f32]) {
        // own and insert FX latency and the family target set the pad (they only change between cycles)
        let il = claimed_latency(&*self.fx, &self.preins, &self.postins);
        let t = self.fx.align_target();
        self.align.sync(self.own_latency(), il, t);
        let (g0, g1) = self.gain.step();
        if g0 != [1.0, 1.0] || g1 != [1.0, 1.0] {
            ramp_in_place(l, g0[0], g1[0]);
            ramp_in_place(r, g0[1], g1[1]);
        }
        self.tap(MixTap::In, l, r);
        if let Some(m) = &self.meter {
            m.stereo(word::IN_L, l, r);
        }
        if let Some(n) = self.preins.claimed() {
            self.fx
                .process_claimed(InsertPos::Pre, n, self.preins.on, l, r);
        }
        self.tap(MixTap::Ins1, l, r);
        self.eq.process(l, r);
        self.tap(MixTap::Eq, l, r);
        let dyn_in = if self.meter.is_some() {
            abs_max(l).max(abs_max(r))
        } else {
            0.0
        };
        let cycle = self.key_out.as_ref().map_or(0, |k| k.cycle());
        let key = match self.key_in.as_mut() {
            Some(k) if self.dyn_sc.external() => k.fetch(cycle, l.len()),
            _ => None,
        };
        let listen = self.dyn_sc.listen;
        if listen {
            listen_pre(&self.key_out, key_slot::LISTEN_DYN, key, l, r);
        }
        self.dyn_.process_keyed(l, r, key);
        if listen {
            listen_post(&self.key_out, key_slot::LISTEN_DYN, &self.dyn_);
        }
        let dyn_key = if self.meter.is_some() {
            slot_key_peak(&self.dyn_, key, dyn_in)
        } else {
            0.0
        };
        self.tap(MixTap::Dyn, l, r);
        if let Some(n) = self.postins.claimed() {
            self.fx
                .process_claimed(InsertPos::Post, n, self.postins.on, l, r);
        }
        self.align.process(l, r);
        self.tap(MixTap::Pfl, l, r);
        // An off slot can still run its model for one wrapper-ramp after a parameter change
        // (inaudible, wet = 0): report GR only while the slot is on.
        self.telem.dyn_gr_db = if self.dyn_.wrapper().0 {
            self.dyn_.gain_reduction_db()
        } else {
            0.0
        };
        if let Some(m) = &self.meter {
            m.level(word::DYN_KEY, dyn_key);
            m.level(word::DYN_GR, self.telem.dyn_gr_db);
        }
    }

    /// Output delay on the post-fader/pan signal, in place (RT-safe).
    pub fn process_out(&mut self, l: &mut [f32], r: &mut [f32]) {
        self.dly.process(l, r);
        self.tap(MixTap::Post, l, r);
    }
}

#[inline]
fn ramp_in_place(x: &mut [f32], g0: f32, g1: f32) {
    if g0 == g1 {
        for v in x.iter_mut() {
            *v *= g0;
        }
        return;
    }
    let inc = (g1 - g0) / x.len().max(1) as f32;
    for (i, v) in x.iter_mut().enumerate() {
        *v *= g0 + inc * (i + 1) as f32;
    }
}

impl StripHook for MixStrip {
    fn process(&mut self, l: &mut [f32], r: &mut [f32]) {
        self.process_pre(l, r);
    }

    fn process_post(&mut self, l: &mut [f32], r: &mut [f32]) {
        self.process_out(l, r);
    }

    fn set_param(&mut self, msg: &ParamMsg) {
        self.apply(msg);
    }

    fn set_insert_host(&mut self, fx: Box<dyn InsertHost + Send>) {
        MixStrip::set_insert_host(self, fx);
    }

    fn latency(&self) -> usize {
        MixStrip::latency(self)
    }

    fn set_keys(&mut self, bank: KeyBank, idx: usize) {
        MixStrip::set_keys(self, bank, idx);
    }

    fn begin_cycle(&mut self, cycle: u64) {
        if let Some(k) = self.key_out.as_mut() {
            k.set_cycle(cycle);
        }
    }

    fn telemetry(&mut self, out: &mut TelemetryWriter<'_>) {
        let t = self.take_telemetry();
        out.push(hook::DYN, kind::GAIN_REDUCTION_DB, t.dyn_gr_db);
        for (i, p) in t.tap_peak.iter().enumerate() {
            out.push(i as u8, kind::TAP_PEAK, *p);
        }
    }
}

/// Bind the processing blocks of `bus/N`, `main/N` or `mtx/N` to the [`MixStrip`] hook of `node`
/// and register the node for telemetry.
pub fn bind_mix_strip(
    b: &mut ParamMapBuilder,
    strip: Strip,
    node: NodeId,
) -> Result<(), BindError> {
    let p = match strip {
        Strip::Bus(n) => format!("bus/{n}"),
        Strip::Main(n) => format!("main/{n}"),
        Strip::Mtx(n) => format!("mtx/{n}"),
        _ => return Err(BindError::UnknownPath(format!("{strip:?}"))),
    };
    b.bind_block(&format!("{p}/in/set"), node, hook::INPUT, &MIX_INPUT_LAYOUT)?;
    b.bind_block(&format!("{p}/preins"), node, hook::PREINS, &PREINS_LAYOUT)?;
    b.bind_block(&format!("{p}/eq"), node, hook::EQ, &MIX_EQ_LAYOUT)?;
    b.bind_block(&format!("{p}/dyn"), node, hook::DYN, &SLOT_LAYOUT)?;
    b.bind_block(&format!("{p}/dynsc"), node, hook::DYN_SC, &SC_LAYOUT)?;
    b.bind_block(&format!("{p}/dynxo"), node, hook::DYN_XO, &XO_LAYOUT)?;
    b.bind_block(&format!("{p}/postins"), node, hook::POSTINS, &PREINS_LAYOUT)?;
    b.bind_block(&format!("{p}/dly"), node, hook::DLY, &DLY_LAYOUT)?;
    b.telemetry_node(node);
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn eq_models_per_strip_type() {
        for s in [Strip::Bus(1), Strip::Main(2), Strip::Mtx(3)] {
            let m = MixStrip::new(s, 48_000.0, 64);
            // STD..PULSAR + PIA (7 models), no MACH4
            assert_eq!(m.eq_slot().num_instances(), 7, "{s:?}");
            assert_eq!(m.dyn_slot().model_index(), Some(13));
        }
    }

    #[test]
    fn default_strip_is_transparent() {
        // transparent up to the strip's constant latency (T03E)
        let mut m = MixStrip::new(Strip::Main(1), 48_000.0, 64);
        let lat = m.latency();
        let x: Vec<f32> = (0..512).map(|i| (i as f32 * 0.3).sin() * 0.5).collect();
        let (mut l, mut r) = (x.clone(), x.clone());
        for (a, b) in l.chunks_mut(64).zip(r.chunks_mut(64)) {
            m.process_pre(a, b);
            m.process_out(a, b);
        }
        assert!(l[..lat].iter().all(|&v| v == 0.0));
        assert_eq!(l[lat..], x[..512 - lat]);
        assert_eq!(r[lat..], x[..512 - lat]);
    }
}
