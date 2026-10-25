//! Mix processors: the input strip node (channel/aux strip + fader/pan/taps) and the summing strip
//! node shared by busses, mains and matrices (send sum -> strip hook -> fader/pan).

use std::sync::Arc;

use crate::channel::{
    sync_inserts, ChannelStrip, InsertHost, InsertPoint, NoInserts, StripKeys, Tap as StripTap,
};
use crate::graph::{NodeIo, ProcessCtx, Processor};
use crate::params::{ParamMsg, TelemetryWriter};
use crate::routing::Strip;

use super::law::{self, Matrix, PanLaw, Ramp, IDENTITY, ZERO};
use super::logic::{flag, MixShared};
use super::{block, param, MixConfig, SRC_COUNT};

/// Signals an input strip node outputs (stereo pairs, output channel = `2 * index + side`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum InputSignal {
    /// Pre-fader (post-processing, `PFL`), not muted: user signals "PRE FADER", direct outs.
    Pfl = 0,
    /// The channel tap `ptap` (aux: pre-fader), muted, `tapwid` applied: PRE sends.
    Pre = 1,
    /// [`InputSignal::Pre`] through the channel pan: PRE sends with pan link, main sends with `pre`.
    PreLinked = 2,
    /// Post mute and fader, before pan/width: POST sends without pan link.
    Afl = 3,
    /// Post fader, width and pan: main sends, GRP sends, POST sends with pan link, user signals
    /// "POST PROCESSING".
    Post = 4,
}

/// `in/set/$mode` and `$g` parameter ids of the strip's INPUT block ([`crate::params::strip::INPUT_LAYOUT`]).
const INPUT_MODE: u16 = 4;
const INPUT_GAIN: u16 = 5;

/// Number of input-strip output channels.
pub const INPUT_OUTPUTS: u32 = 10;

/// Signals a bus/main/matrix node outputs.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum MixSignal {
    /// Pre-fader (post strip processing), not muted: user signals "PRE FADER".
    Pfl = 0,
    /// Pre-fader, muted: sends with `pre`.
    Pre = 1,
    /// Post mute, fader, width, pan and `busmono`: the strip output (BUS/MAIN/MTX legs), sends
    /// without `pre`, user signals "POST PROCESSING".
    Post = 2,
}

/// Number of bus/main/matrix output channels.
pub const MIX_OUTPUTS: u32 = 6;

/// Own fader section of any strip (`mute fdr pan wid` + `tapwid`/`busmono` + external control).
#[derive(Clone, Copy, Debug)]
struct Fader {
    mute: bool,
    fdr_db: f32,
    pan: f32,
    wid: f32,
    tapwid: f32,
    busmono: bool,
    ext_mute: bool,
    ext_gain_db: f32,
    mute_safe: bool,
}

impl Default for Fader {
    fn default() -> Self {
        Self {
            mute: false,
            fdr_db: law::FADER_OFF_DB,
            pan: 0.0,
            wid: 100.0,
            tapwid: 100.0,
            busmono: false,
            ext_mute: false,
            ext_gain_db: 0.0,
            mute_safe: false,
        }
    }
}

impl Fader {
    fn set(&mut self, p: u16, v: f32) -> bool {
        match p {
            param::MUTE => self.mute = v != 0.0,
            param::FDR => self.fdr_db = v,
            param::PAN => self.pan = v,
            param::WID => self.wid = v,
            param::TAPWID => self.tapwid = v,
            param::BUSMONO => self.busmono = v != 0.0,
            param::EXT_MUTE => self.ext_mute = v != 0.0,
            param::EXT_GAIN => self.ext_gain_db = v,
            param::MUTE_SAFE => self.mute_safe = v != 0.0,
            _ => return false,
        }
        true
    }

    /// Effective mute: own mute, or an external (mute group / DCA, T038) mute unless mute-safe.
    fn muted(&self) -> bool {
        self.mute || (self.ext_mute && !self.mute_safe)
    }

    /// 1 or 0.
    fn on_gain(&self) -> f32 {
        if self.muted() {
            0.0
        } else {
            1.0
        }
    }

    /// Fader gain including mute and the external (DCA) offset: `fdr + ext` dB, clamped to
    /// +10 dB, -oo if either is -oo.
    fn gain(&self) -> f32 {
        let off = law::FADER_OFF_DB + 0.01;
        if self.muted() || self.fdr_db <= off || self.ext_gain_db <= off {
            0.0
        } else {
            law::fader_gain((self.fdr_db + self.ext_gain_db).min(10.0))
        }
    }
}

/// RT reader of a strip's DCA / mute-group / solo-in-place state ([`MixShared`]).
struct LogicLink {
    shared: Arc<MixShared>,
    idx: usize,
    seen: u32,
}

impl LogicLink {
    /// Copy the strip's external gain/mute into `f` if a new state was published; returns
    /// whether `f` changed. RT-safe.
    #[inline]
    fn poll(link: &mut Option<Self>, f: &mut Fader) -> bool {
        let Some(l) = link.as_mut() else {
            return false;
        };
        let g = l.shared.generation();
        if g == l.seen {
            return false;
        }
        l.seen = g;
        let gain = l.shared.gain_db(l.idx);
        let mute = l.shared.flags(l.idx) & flag::MUTE != 0;
        let changed = gain != f.ext_gain_db || mute != f.ext_mute;
        f.ext_gain_db = gain;
        f.ext_mute = mute;
        changed
    }
}

/// Split a planar output buffer into `N` channels.
#[inline]
fn channels<const N: usize>(out: &mut [f32], b: usize) -> [&mut [f32]; N] {
    let mut it = out.chunks_exact_mut(b);
    std::array::from_fn(|_| it.next().expect("output channel"))
}

// ---- input strips -------------------------------------------------------------------------------

/// Channel (`ch.N`) / aux (`aux.N`) strip node: T035 strip processing, then mute and fader
/// (`AFL`), width and pan (`POST`) and the send taps.
///
/// Inputs: 0/1 = source L/R (input router). Outputs: [`InputSignal`] pairs. Parameter blocks:
/// the [`crate::channel::block`] ids (forwarded to the [`ChannelStrip`]) and
/// [`block::INPUT_MIX`] (`mute fdr pan wid tapwid`).
///
/// A `ptap` change switches the PRE source at a block boundary (a discontinuity, like `proc`).
///
/// The input router (T024) already applies the source's preamp gain and decodes its mode
/// (mono legs arrive as L = R, M/S as L/R), so the strip's `in/set/$mode` and `$g` mirrors are
/// pinned to ST / 0 dB here; forwarding them would mono-sum stereo sources or apply the gain
/// twice.
pub struct InputStripNode {
    strip: ChannelStrip,
    fader: Fader,
    pan_law: PanLaw,
    dirty: bool,
    /// AFL gain.
    fdr: Ramp<1>,
    /// POST = matrix * AFL (width, pan).
    post: Ramp<4>,
    /// PRE = matrix * tap (mute, tapwid).
    pre: Ramp<4>,
    /// PRE-linked = matrix * PRE (pan).
    prep: Ramp<4>,
    logic: Option<LogicLink>,
    /// Insert FX host (the FX rack handle of this strip, [`crate::fx`]).
    fx: Box<dyn InsertHost + Send>,
    /// Insert state last forwarded to `fx`.
    ins_seen: (InsertPoint, InsertPoint),
}

impl InputStripNode {
    /// Wrap a strip. Allocates nothing beyond the strip.
    pub fn new(mut strip: ChannelStrip, cfg: &MixConfig) -> Self {
        let a = law::block_alpha(law::MIX_TAU_S, cfg.sample_rate as f64, cfg.block_size);
        for (param, value) in [(INPUT_MODE, 1.0), (INPUT_GAIN, 0.0)] {
            strip.apply(&ParamMsg::Set {
                block: crate::channel::block::INPUT,
                param,
                value,
            });
        }
        strip.settle();
        let mut s = Self {
            strip,
            fader: Fader::default(),
            pan_law: cfg.input_pan,
            dirty: true,
            fdr: Ramp::new(a),
            post: Ramp::new(a),
            pre: Ramp::new(a),
            prep: Ramp::new(a),
            logic: None,
            fx: Box::new(NoInserts),
            ins_seen: Default::default(),
        };
        s.update_targets();
        s
    }

    /// Use `fx` for the strip's inserts (the FX rack handle, [`crate::fx::FxRack::host`]). The current
    /// insert assignment is forwarded to it. Off the RT thread.
    pub fn with_inserts(mut self, fx: Box<dyn InsertHost + Send>) -> Self {
        self.fx = fx;
        self.ins_seen = Default::default();
        sync_inserts(&mut *self.fx, &mut self.ins_seen, self.strip.inserts());
        self
    }

    /// Processing latency (samples): active models plus engaged insert FX.
    pub fn latency(&self) -> usize {
        self.strip.latency_with(&*self.fx)
    }

    /// Follow the DCA / mute-group / SIP state of strip index `idx` ([`Strip::index`]).
    pub fn with_logic(mut self, shared: Arc<MixShared>, idx: usize) -> Self {
        self.logic = Some(LogicLink {
            shared,
            idx,
            seen: u32::MAX,
        });
        if LogicLink::poll(&mut self.logic, &mut self.fader) {
            self.update_targets();
        }
        self
    }

    pub fn strip(&self) -> &ChannelStrip {
        &self.strip
    }

    pub fn strip_mut(&mut self) -> &mut ChannelStrip {
        &mut self.strip
    }

    /// Jump every gain to its target on the next block (and settle the strip).
    pub fn settle(&mut self) {
        if LogicLink::poll(&mut self.logic, &mut self.fader) {
            self.dirty = true;
        }
        self.apply(&ParamMsg::Commit);
        self.strip.settle();
        for r in [&mut self.post, &mut self.pre, &mut self.prep] {
            r.settle();
        }
        self.fdr.settle();
    }

    /// Current AFL gain target (fader x mute x external).
    pub fn fader_gain(&self) -> f32 {
        self.fdr.target()[0]
    }

    fn update_targets(&mut self) {
        let f = &self.fader;
        self.fdr.set([f.gain()]);
        self.post.set(law::pan_width(self.pan_law, f.pan, f.wid));
        self.pre.set(law::scale(law::width(f.tapwid), f.on_gain()));
        self.prep.set(law::pan(self.pan_law, f.pan));
        self.dirty = false;
    }

    /// Apply a parameter message (same as [`Processor::set_param`]). RT-safe.
    pub fn apply(&mut self, msg: &ParamMsg) {
        match *msg {
            ParamMsg::Set {
                block: block::INPUT_MIX,
                param,
                value,
            } => self.dirty |= self.fader.set(param, value),
            ParamMsg::Set {
                block: crate::channel::block::INPUT,
                param: INPUT_MODE | INPUT_GAIN,
                ..
            } => {}
            ParamMsg::Set { block: b, .. } | ParamMsg::Model { block: b, .. } => {
                if b < block::INPUT_MIX {
                    self.strip.apply(msg);
                }
            }
            ParamMsg::Commit => {
                self.strip.apply(msg);
                sync_inserts(&mut *self.fx, &mut self.ins_seen, self.strip.inserts());
                if self.dirty {
                    self.update_targets();
                }
            }
        }
    }
}

impl Processor for InputStripNode {
    fn process(&mut self, _ctx: &ProcessCtx, io: &mut NodeIo<'_>) {
        if LogicLink::poll(&mut self.logic, &mut self.fader) {
            self.update_targets();
        }
        let ins = io.inputs();
        let b = io.block_size();
        let [pfl_l, pfl_r, pre_l, pre_r, prep_l, prep_r, afl_l, afl_r, post_l, post_r] =
            channels::<10>(io.outputs(), b);
        match ins.len() {
            0 => {
                pfl_l.fill(0.0);
                pfl_r.fill(0.0);
            }
            1 => {
                pfl_l.copy_from_slice(ins.get(0));
                pfl_r.copy_from_slice(ins.get(0));
            }
            _ => {
                pfl_l.copy_from_slice(ins.get(0));
                pfl_r.copy_from_slice(ins.get(1));
            }
        }
        self.strip
            .process_with(pfl_l, pfl_r, StripKeys::default(), &mut *self.fx);

        let (g0, g1) = self.fdr.step();
        law::gain_write(afl_l, pfl_l, g0[0], g1[0]);
        law::gain_write(afl_r, pfl_r, g0[0], g1[0]);
        let (m0, m1) = self.post.step();
        law::matrix_write(post_l, post_r, afl_l, afl_r, m0, m1);

        let (p0, p1) = self.pre.step();
        let (q0, q1) = self.prep.step();
        match self.strip.ptap() {
            StripTap::Afl => {
                pre_l.copy_from_slice(afl_l);
                pre_r.copy_from_slice(afl_r);
            }
            StripTap::Post => {
                pre_l.copy_from_slice(post_l);
                pre_r.copy_from_slice(post_r);
            }
            _ => {
                let (tl, tr) = self.strip.tap_signal();
                if tl.len() == b {
                    law::matrix_write(pre_l, pre_r, tl, tr, p0, p1);
                } else {
                    pre_l.fill(0.0);
                    pre_r.fill(0.0);
                }
            }
        }
        if self.strip.ptap() == StripTap::Post {
            prep_l.copy_from_slice(pre_l);
            prep_r.copy_from_slice(pre_r);
        } else {
            law::matrix_write(prep_l, prep_r, pre_l, pre_r, q0, q1);
        }
    }

    fn set_param(&mut self, msg: &ParamMsg) {
        self.apply(msg);
    }

    fn telemetry(&mut self, out: &mut TelemetryWriter<'_>) {
        Processor::telemetry(&mut self.strip, out);
    }
}

// ---- bus / main / matrix ------------------------------------------------------------------------

/// Processing of a bus/main/matrix strip between its sum and its fader (T037 fills it: inserts,
/// EQ, dynamics, delay). Runs on RT threads: no allocation, locks or syscalls.
pub trait StripHook: Send {
    /// Process the summed stereo signal in place (sum -> pre-fader).
    fn process(&mut self, l: &mut [f32], r: &mut [f32]);
    /// Process the strip output in place after mute, fader, width, pan and `busmono` (output
    /// delay); PRE/PFL taps are not affected.
    fn process_post(&mut self, _l: &mut [f32], _r: &mut [f32]) {}
    /// Parameter messages for blocks `>=` [`block::HOOK_BASE`] (and every `Commit`).
    fn set_param(&mut self, _msg: &ParamMsg) {}
    /// Telemetry (polled if the node is registered as a telemetry source).
    fn telemetry(&mut self, _out: &mut TelemetryWriter<'_>) {}
    /// Install the insert FX host (the FX rack handle of this strip). Off the RT thread. Default:
    /// the hook has no inserts and drops it.
    fn set_insert_host(&mut self, _fx: Box<dyn InsertHost + Send>) {}
    /// Processing latency in samples (models and engaged insert FX).
    fn latency(&self) -> usize {
        0
    }
}

/// No strip processing (T036 behaviour; the standard hook is [`super::MixStrip`]).
pub struct NoProcessing;

impl StripHook for NoProcessing {
    fn process(&mut self, _l: &mut [f32], _r: &mut [f32]) {}
}

/// Send mode (`ch|aux/*/send/*/mode`; `pre` flags map to `Pre`/`Post`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SendMode {
    /// At the sender's tap (channels: `ptap`), independent of its fader.
    Pre,
    /// Post the sender's fader (and mute).
    Post,
    /// Subgroup: post fader and pan, `on` is the assignment; the level is ignored by default
    /// (PUBLIC.snap assigns drums to their group bus at -oo) [I].
    Grp,
}

impl SendMode {
    fn from_index(v: f32) -> Self {
        match v as i32 {
            0 => SendMode::Pre,
            2 => SendMode::Grp,
            _ => SendMode::Post,
        }
    }
}

/// What feeds a send slot.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SlotKind {
    /// A channel/aux: inputs `Pre, PreLinked, Afl, Post` (8 ports).
    Input,
    /// A bus or main: inputs `Pre, Post` (4 ports).
    Mix,
}

impl SlotKind {
    /// Number of input ports of a slot.
    pub fn ports(self) -> usize {
        match self {
            SlotKind::Input => 8,
            SlotKind::Mix => 4,
        }
    }
}

/// Send parameters of one source into this node.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SendParams {
    pub on: bool,
    pub lvl_db: f32,
    pub mode: SendMode,
    pub plink: u8,
    pub pan: f32,
}

impl Default for SendParams {
    fn default() -> Self {
        Self {
            on: false,
            lvl_db: law::FADER_OFF_DB,
            mode: SendMode::Post,
            plink: 0,
            pan: 0.0,
        }
    }
}

struct Slot {
    kind: SlotKind,
    /// First input port index.
    base: u16,
    p: SendParams,
    dirty: bool,
    /// One matrix per input signal (`Input`: Pre, PreLinked, Afl, Post; `Mix`: Pre, Post).
    ramps: [Ramp<4>; 4],
}

/// Destination kind of a summing node.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Dest {
    Bus,
    Main,
    Mtx,
}

/// One wired source of a summing node.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SlotSpec {
    /// Global source index (`0..48` ch/aux, `48..64` bus, `64..68` main).
    pub src: u8,
    pub kind: SlotKind,
}

/// Bus (`bus.N`), main (`main.N`) or matrix (`mtx.N`) node: sums its sends, runs the
/// [`StripHook`], then mute/fader and width/pan/`busmono`.
///
/// Inputs: per [`SlotSpec`] in order, [`SlotKind::ports`] ports. Outputs: [`MixSignal`] pairs.
/// Parameter blocks: [`block::OWN`] (`mute fdr pan wid busmono`), [`block::SEND_BASE`] + source
/// index for every source's send into this strip, `>=` [`block::HOOK_BASE`] for the hook.
pub struct MixStripNode {
    strip: Strip,
    dest: Dest,
    hook: Box<dyn StripHook>,
    fader: Fader,
    dirty: bool,
    slots: Box<[Slot]>,
    slot_of: [u8; SRC_COUNT],
    /// POST = matrix * PFL (fader, mute, width, pan, mono).
    post: Ramp<4>,
    /// PRE = gain * PFL (mute).
    pre: Ramp<1>,
    bus_law: PanLaw,
    send_law: PanLaw,
    grp_level: bool,
    logic: Option<LogicLink>,
}

impl MixStripNode {
    /// `strip` must be a bus, main or matrix. Allocates (control thread).
    pub fn new(
        strip: Strip,
        slots: &[SlotSpec],
        hook: Box<dyn StripHook>,
        cfg: &MixConfig,
    ) -> Self {
        let dest = match strip {
            Strip::Bus(_) => Dest::Bus,
            Strip::Main(_) => Dest::Main,
            Strip::Mtx(_) => Dest::Mtx,
            _ => panic!("MixStripNode for {strip:?}"),
        };
        let a = law::block_alpha(law::MIX_TAU_S, cfg.sample_rate as f64, cfg.block_size);
        let mut slot_of = [u8::MAX; SRC_COUNT];
        let mut base = 0u16;
        let slots: Box<[Slot]> = slots
            .iter()
            .enumerate()
            .map(|(i, s)| {
                slot_of[s.src as usize] = i as u8;
                let slot = Slot {
                    kind: s.kind,
                    base,
                    p: SendParams::default(),
                    dirty: false,
                    ramps: [Ramp::new(a); 4],
                };
                base += s.kind.ports() as u16;
                slot
            })
            .collect();
        assert!(slots.len() < u8::MAX as usize);
        let mut n = Self {
            strip,
            dest,
            hook,
            fader: Fader::default(),
            dirty: true,
            slots,
            slot_of,
            post: Ramp::new(a),
            pre: Ramp::new(a),
            bus_law: cfg.bus_pan,
            send_law: cfg.send_pan,
            grp_level: cfg.grp_uses_level,
            logic: None,
        };
        n.update_targets();
        n
    }

    /// Follow the DCA / mute-group / SIP state of strip index `idx` ([`Strip::index`]).
    pub fn with_logic(mut self, shared: Arc<MixShared>, idx: usize) -> Self {
        self.logic = Some(LogicLink {
            shared,
            idx,
            seen: u32::MAX,
        });
        if LogicLink::poll(&mut self.logic, &mut self.fader) {
            self.update_targets();
        }
        self
    }

    pub fn strip(&self) -> Strip {
        self.strip
    }

    /// Number of input ports the node expects.
    pub fn num_inputs(&self) -> usize {
        self.slots.iter().map(|s| s.kind.ports()).sum()
    }

    /// Whether source `src` is wired into this node.
    pub fn has_source(&self, src: u8) -> bool {
        self.slot_of
            .get(src as usize)
            .is_some_and(|&s| s != u8::MAX)
    }

    /// Send parameters of source `src` (if wired).
    pub fn send(&self, src: u8) -> Option<&SendParams> {
        let s = *self.slot_of.get(src as usize)?;
        self.slots.get(s as usize).map(|s| &s.p)
    }

    pub fn hook_mut(&mut self) -> &mut dyn StripHook {
        self.hook.as_mut()
    }

    /// Processing latency (samples) of the strip processing incl. engaged insert FX.
    pub fn latency(&self) -> usize {
        self.hook.latency()
    }

    /// Jump every gain to its target on the next block.
    pub fn settle(&mut self) {
        if LogicLink::poll(&mut self.logic, &mut self.fader) {
            self.dirty = true;
        }
        self.apply(&ParamMsg::Commit);
        self.post.settle();
        self.pre.settle();
        for s in self.slots.iter_mut() {
            for r in s.ramps.iter_mut() {
                r.settle();
            }
        }
    }

    fn slot_targets(&self, s: &Slot) -> [Matrix; 4] {
        let mut t = [ZERO; 4];
        let p = &s.p;
        if !p.on {
            return t;
        }
        let g = law::fader_gain(p.lvl_db);
        match s.kind {
            SlotKind::Mix => {
                // bus/main senders: `pre` = pre-fader tap, else post (fader + pan) [I]
                let k = if p.mode == SendMode::Pre { 0 } else { 1 };
                t[k] = law::scale(IDENTITY, g);
            }
            SlotKind::Input => match (self.dest, p.mode) {
                (Dest::Main, SendMode::Pre) => t[1] = law::scale(IDENTITY, g),
                (Dest::Main, _) => t[3] = law::scale(IDENTITY, g),
                (_, SendMode::Pre) if p.plink > 0 => t[1] = law::scale(IDENTITY, g),
                (_, SendMode::Pre) => t[0] = law::scale(law::pan(self.send_law, p.pan), g),
                (_, SendMode::Post) if p.plink > 0 => t[3] = law::scale(IDENTITY, g),
                (_, SendMode::Post) => t[2] = law::scale(law::pan(self.send_law, p.pan), g),
                (_, SendMode::Grp) => {
                    t[3] = law::scale(IDENTITY, if self.grp_level { g } else { 1.0 })
                }
            },
        }
        t
    }

    fn update_targets(&mut self) {
        let f = &self.fader;
        let mut m = law::pan_width(self.bus_law, f.pan, f.wid);
        if f.busmono {
            m = law::mul(law::MONO, m);
        }
        self.post.set(law::scale(m, f.gain()));
        self.pre.set([f.on_gain()]);
        self.dirty = false;
    }

    /// Apply a parameter message (same as [`Processor::set_param`]). RT-safe.
    pub fn apply(&mut self, msg: &ParamMsg) {
        match *msg {
            ParamMsg::Set {
                block: block::OWN,
                param,
                value,
            } => self.dirty |= self.fader.set(param, value),
            ParamMsg::Set {
                block: b,
                param: p,
                value: v,
            } if b < block::HOOK_BASE => {
                let src = (b - block::SEND_BASE) as usize;
                let Some(&k) = self.slot_of.get(src) else {
                    return;
                };
                let Some(s) = self.slots.get_mut(k as usize) else {
                    return; // not wired (e.g. a bus->bus send against the bus order)
                };
                match p {
                    param::ON => s.p.on = v != 0.0,
                    param::LVL => s.p.lvl_db = v,
                    param::MODE => s.p.mode = SendMode::from_index(v),
                    param::PRE => {
                        s.p.mode = if v != 0.0 {
                            SendMode::Pre
                        } else {
                            SendMode::Post
                        }
                    }
                    param::PLINK => s.p.plink = v.clamp(0.0, 255.0) as u8,
                    param::SEND_PAN => s.p.pan = v,
                    _ => return,
                }
                s.dirty = true;
            }
            ParamMsg::Set { .. } | ParamMsg::Model { .. } => self.hook.set_param(msg),
            ParamMsg::Commit => {
                if self.dirty {
                    self.update_targets();
                }
                for i in 0..self.slots.len() {
                    if self.slots[i].dirty {
                        let t = self.slot_targets(&self.slots[i]);
                        let s = &mut self.slots[i];
                        s.dirty = false;
                        for (r, m) in s.ramps.iter_mut().zip(t) {
                            r.set(m);
                        }
                    }
                }
                self.hook.set_param(msg);
            }
        }
    }
}

impl Processor for MixStripNode {
    fn process(&mut self, _ctx: &ProcessCtx, io: &mut NodeIo<'_>) {
        if LogicLink::poll(&mut self.logic, &mut self.fader) {
            self.update_targets();
        }
        let ins = io.inputs();
        let b = io.block_size();
        let [pfl_l, pfl_r, pre_l, pre_r, post_l, post_r] = channels::<6>(io.outputs(), b);
        pfl_l.fill(0.0);
        pfl_r.fill(0.0);
        for s in self.slots.iter_mut() {
            let n = match s.kind {
                SlotKind::Input => 4,
                SlotKind::Mix => 2,
            };
            for k in 0..n {
                let (m0, m1) = s.ramps[k].step();
                if m0 == ZERO && m1 == ZERO {
                    continue;
                }
                let i = s.base as usize + 2 * k;
                if i + 1 < ins.len() {
                    law::matrix_add(pfl_l, pfl_r, ins.get(i), ins.get(i + 1), m0, m1);
                }
            }
        }
        self.hook.process(pfl_l, pfl_r);
        let (g0, g1) = self.pre.step();
        law::gain_write(pre_l, pfl_l, g0[0], g1[0]);
        law::gain_write(pre_r, pfl_r, g0[0], g1[0]);
        let (m0, m1) = self.post.step();
        law::matrix_write(post_l, post_r, pfl_l, pfl_r, m0, m1);
        self.hook.process_post(post_l, post_r);
    }

    fn set_param(&mut self, msg: &ParamMsg) {
        self.apply(msg);
    }

    fn telemetry(&mut self, out: &mut TelemetryWriter<'_>) {
        self.hook.telemetry(out);
    }
}
