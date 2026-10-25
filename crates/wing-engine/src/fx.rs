//! FX rack (T050) and insert hosting (T044): 16 stereo FX slots, the insert points of every strip that
//! run them, the FX SEND legs and the channel automixer (post-insert AUTO_X / AUTO_Y).
//!
//! # WING rules mirrored (signal-flow.md §8, models.md §5/§6, schema)
//!
//! * `fx/1..16 = {mdl, fxmix, 1..40}`. Slots 1-8 ("full", model group `fx/1`) offer all 64 models,
//!   slots 9-16 ("lite", group `fx/9`) the 38 without the reverb / modulation / delay / pitch families.
//!   Each [`FxSlot`] preallocates one instance per model **its** group lists (`NONE` = pass-through), so a
//!   lite slot structurally cannot run a premium model and a model switch is an index change plus a
//!   crossfade ([`ModelSlot`]), never an allocation.
//! * An FX slot is only ever used as an **insert** (`preins`/`postins` `ins = FXn` of a ch/aux/bus/main/mtx;
//!   aux has `preins` only). The schema has no FX source group (`conn.grp` stops at MTX, user signals at
//!   MTX): a "send effect" is an FX inserted on a bus that the channels send to; its return is the bus
//!   (to the mains, bus->bus, or re-input as a channel/aux source `BUS.n`). There is no separate FX return.
//! * A slot serves **one insert at a time**. Every insert that selects `FXn` (in FX mode, on or off) claims
//!   it; the owner is the claiming insert with the lowest id (strip order ch, aux, bus, main, mtx; Ins1
//!   before Ins2), so the result is independent of the order a scene loads [I]. The others pass through
//!   and report `$stat` N/A ([`FxRack::insert_state`]); the owner is the slot's `$a_chn`/`$a_pos`
//!   ([`FxRack::owner`]). A slot whose owner changes is reset (no stale tail).
//! * Per slot: `fxmix` = wet/dry of the [`ModelSlot`] wrapper (models output wet only). The wrapper's `on`
//!   and `gain` have no schema key for FX slots: `on` is fixed on (an unassigned or bypassed insert simply
//!   does not run the slot), `gain` stays 0 dB; both are reachable as [`param::ON`]/[`param::GAIN`].
//!   Tempo / tap: the schema has no global tempo or tap key; time-based models carry their own `time` and
//!   note `fact`or (ST-DL), so nothing global is bound.
//! * **FX SEND legs** (`SEND` output group, 32 legs = 16 stereo): the signal entering each slot while it is
//!   processed (silence otherwise), routable to any output (e.g. USB to process in a DAW; the EXT model's
//!   return, `egrp/ein`, is T05C).
//! * **Automix** (channel `postins.mode` AUTO_X / AUTO_Y, weight `w`, groups enabled by `cfg/amix/x|y`):
//!   Dugan-style gain sharing. Each member's level is a smoothed mean-square power `P_i` (5 ms attack,
//!   100 ms release) weighted by `10^(w/10)`; its gain is `sqrt(Pw_i / sum_j Pw_j)` (constant total power:
//!   one open mic at 0 dB, N equal mics at -10 log N dB each). A disabled group passes through.
//!
//! # Scheduling (why inline)
//!
//! Insert FX run **inline in the host strip's node**, at the insert point. The alternative, a graph node per
//! slot, would have to split every strip at both insert points (ch: anywhere in the `proc` order) into
//! separate nodes, multiplying nodes and levels, and a slot that moves between a channel and a bus would
//! need a graph rebuild. Inline is cycle-free by construction: a slot has exactly one owner, so it runs
//! once per cycle at its owner's level and never feeds an earlier node. Slot state lives in a shared
//! [`FxRack`]; mutation is race-free by the engine's phase discipline (see `SlotCell`).
//!
//! The [`FxRackNode`] itself sits after the matrices (parallel to the monitor section) and
//! * receives the `fx/*` and `cfg/amix` parameters (driver thread, between cycles),
//! * publishes the FX SEND legs captured during the cycle as its 32 outputs, and
//! * sums the automix group levels for the next cycle. Automix gains therefore use the other members'
//!   levels of the previous block (1.3 ms at 48 kHz/64; the audio itself is not delayed) - this keeps the
//!   40 channels parallel without a cross-channel barrier and is deterministic.
//!
//! # Latency
//!
//! [`FxRack::slot_latency`] reports the audible model's latency per slot; strips add the latency of their
//! engaged, owned inserts ([`crate::channel::ChannelStrip::latency_with`], [`crate::mix::MixStrip::latency`],
//! `InputStripNode::latency`/`MixStripNode::latency`). Like the console, nothing is compensated
//! automatically (signal-flow.md §11).

use std::cell::UnsafeCell;
use std::sync::atomic::{AtomicBool, AtomicU16, AtomicU32, AtomicU64, AtomicU8, Ordering::Relaxed};
use std::sync::Arc;

use wing_blocks::model::BlockKind;

use crate::channel::{InsertHost, InsertPos};
use crate::graph::{NodeId, NodeIo, ProcessCtx, Processor};
use crate::params::std_models::model_factory;
use crate::params::strip::{ModelSlot, RtModel};
use crate::params::{BindError, BlockLayout, ParamMapBuilder, ParamMsg, TARGET_BASE};
use crate::routing::{Strip, STRIP_COUNT};

/// Number of FX slots.
pub const FX_SLOTS: usize = 16;
/// First "lite" slot (models of group `fx/9`).
pub const FIRST_LITE_SLOT: u8 = 9;
/// Outputs of the [`FxRackNode`]: FX SEND legs 1..32 (2k-1 / 2k = L / R of slot k).
pub const SEND_OUTPUTS: u32 = 2 * FX_SLOTS as u32;
/// Insert ids: `2 * strip index + pos`.
pub const INSERT_IDS: usize = 2 * STRIP_COUNT;
/// Input strips that can automix (channels 1..40).
const AMIX_STRIPS: usize = 40;
const NO_OWNER: u16 = u16::MAX;
const CLAIM_WORDS: usize = INSERT_IDS.div_ceil(64);

/// Parameter blocks of the [`FxRackNode`]: slot `n` (1..16) is block `n - 1`.
pub mod block {
    /// `cfg/amix` (`x`, `y`).
    pub const AMIX: u8 = 16;
}

/// Parameter ids.
pub mod param {
    /// Slot wrapper on (not in the schema; fixed on).
    pub const ON: u16 = 0;
    /// `fxmix` %.
    pub const MIX: u16 = 1;
    /// Slot output gain dB (not in the schema).
    pub const GAIN: u16 = 2;
    /// `cfg/amix/x`.
    pub const AMIX_X: u16 = 0;
    /// `cfg/amix/y`.
    pub const AMIX_Y: u16 = 1;
}

/// `fx/N`: `fxmix` + model group `mdl` with the 40 model slots.
pub const FX_LAYOUT: BlockLayout = BlockLayout {
    params: &[("fxmix", param::MIX)],
    targets: &[
        "1", "2", "3", "4", "5", "6", "7", "8", "9", "10", "11", "12", "13", "14", "15", "16",
        "17", "18", "19", "20", "21", "22", "23", "24", "25", "26", "27", "28", "29", "30", "31",
        "32", "33", "34", "35", "36", "37", "38", "39", "40",
    ],
};

/// `cfg/amix`: group enables.
pub const AMIX_LAYOUT: BlockLayout = BlockLayout {
    params: &[("x", param::AMIX_X), ("y", param::AMIX_Y)],
    targets: &[],
};

/// Bind `fx/1..16` and `cfg/amix` to the rack node `node`.
pub fn bind_fx_params(b: &mut ParamMapBuilder, node: NodeId) -> Result<(), BindError> {
    for n in 1..=FX_SLOTS as u8 {
        b.bind_block(&format!("fx/{n}"), node, n - 1, &FX_LAYOUT)?;
    }
    b.bind_block("cfg/amix", node, block::AMIX, &AMIX_LAYOUT)
}

/// Model group of slot `n` (1..16).
pub fn group_key(n: u8) -> &'static str {
    if n < FIRST_LITE_SLOT {
        "fx/1"
    } else {
        "fx/9"
    }
}

/// Insert `$stat`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum InsertStat {
    /// `-`: no FX slot assigned.
    None,
    /// `OK`: the insert owns its slot.
    Ok,
    /// `N/A`: the slot is used by another insert.
    NotAvailable,
}

// ---- slot -----------------------------------------------------------------------------------------

/// One FX slot: a [`ModelSlot`] over the slot's model group plus the SEND feed capture.
pub struct FxSlot {
    n: u8,
    slot: ModelSlot,
    feed_l: Vec<f32>,
    feed_r: Vec<f32>,
    feed_len: usize,
    fed: bool,
}

impl FxSlot {
    /// Slot `n` (1..16) with every model of its group from `factory(enum index, name)` (`NONE` is
    /// always pass-through). Allocates.
    pub fn new(
        n: u8,
        fs: f64,
        max_block: usize,
        mut factory: impl FnMut(u16, &'static str) -> Option<Box<dyn RtModel>>,
    ) -> Self {
        assert!((1..=FX_SLOTS as u8).contains(&n), "FX slot {n}");
        let mut slot = ModelSlot::from_group(group_key(n), fs, max_block, |i, name| {
            (name != "NONE").then(|| factory(i, name)).flatten()
        });
        slot.set_wrapper(param::ON, 1.0);
        slot.select(0);
        slot.settle();
        let b = max_block.max(1);
        Self {
            n,
            slot,
            feed_l: vec![0.0; b],
            feed_r: vec![0.0; b],
            feed_len: 0,
            fed: false,
        }
    }

    /// Slot number (1..16).
    pub fn number(&self) -> u8 {
        self.n
    }

    pub fn model_slot(&self) -> &ModelSlot {
        &self.slot
    }

    pub fn model_slot_mut(&mut self) -> &mut ModelSlot {
        &mut self.slot
    }

    /// Latency of the audible model (samples).
    pub fn latency(&self) -> usize {
        self.slot.latency()
    }

    /// Commit and settle every ramp/fade, clearing signal state (owner change, tests).
    pub fn reset(&mut self) {
        self.slot.settle();
    }

    /// Apply a parameter message (RT-safe).
    pub fn apply(&mut self, msg: &ParamMsg) {
        match *msg {
            ParamMsg::Set { param, value, .. } => {
                if param >= TARGET_BASE {
                    self.slot.set_target(param - TARGET_BASE, value);
                } else {
                    self.slot.set_wrapper(param, value);
                }
            }
            ParamMsg::Model { model, .. } => self.slot.select(model),
            ParamMsg::Commit => self.slot.commit(),
        }
    }

    /// Run the slot in place on an insert signal, capturing its input for the SEND legs (RT-safe).
    pub fn process(&mut self, l: &mut [f32], r: &mut [f32]) {
        let n = l.len().min(self.feed_l.len());
        self.feed_l[..n].copy_from_slice(&l[..n]);
        self.feed_r[..n].copy_from_slice(&r[..n]);
        self.feed_len = n;
        self.fed = true;
        self.slot.process(l, r);
    }

    /// Input captured during the current cycle, if the slot ran.
    pub fn feed(&self) -> Option<(&[f32], &[f32])> {
        self.fed
            .then(|| (&self.feed_l[..self.feed_len], &self.feed_r[..self.feed_len]))
    }
}

// ---- shared rack ----------------------------------------------------------------------------------

/// Interior-mutable slot storage.
///
/// SAFETY (why `Sync` is sound): a slot is mutated only
/// * on the driver thread between cycles (parameter dispatch, claims, telemetry), when no worker runs;
/// * during a cycle by exactly one node: the strip node of the insert that owns the slot (the owner
///   table only changes between cycles), and later in the cycle by the [`FxRackNode`], which reads/clears
///   the SEND capture after every strip level has completed (it depends on the matrices, the last strip
///   level). The engine's level barriers order these accesses (happens-before), exactly as they order
///   the graph's output buffers.
struct SlotCell(UnsafeCell<FxSlot>);

unsafe impl Sync for SlotCell {}

struct Shared {
    slots: Box<[SlotCell]>,
    owner: [AtomicU16; FX_SLOTS],
    claims: [[AtomicU64; CLAIM_WORDS]; FX_SLOTS],
    claim_of: [AtomicU8; INSERT_IDS],
    latency: [AtomicU32; FX_SLOTS],
    model: [AtomicU16; FX_SLOTS],
    instances: [u8; FX_SLOTS],
    amix_on: [AtomicBool; 2],
    amix_group: [AtomicU8; AMIX_STRIPS],
    /// Weighted power of each member (f32 bits), written by the member every block.
    amix_level: [AtomicU32; AMIX_STRIPS],
    /// Group sums of the previous cycle (f32 bits).
    amix_total: [AtomicU32; 2],
    fs: f64,
}

impl Shared {
    /// # Safety
    /// See [`SlotCell`]: only the slot's owner during a cycle, or the driver thread between cycles.
    #[allow(clippy::mut_from_ref)]
    unsafe fn slot_mut(&self, k: usize) -> &mut FxSlot {
        &mut *self.slots[k].0.get()
    }

    fn sync_info(&self, k: usize) {
        // SAFETY: driver thread between cycles (parameter phase).
        let s = unsafe { self.slot_mut(k) };
        self.latency[k].store(s.latency() as u32, Relaxed);
        self.model[k].store(s.model_slot().model_index().unwrap_or(0), Relaxed);
    }

    fn recompute_owner(&self, k: usize) {
        let mut owner = NO_OWNER;
        for (w, word) in self.claims[k].iter().enumerate() {
            let v = word.load(Relaxed);
            if v != 0 {
                owner = (w * 64 + v.trailing_zeros() as usize) as u16;
                break;
            }
        }
        if self.owner[k].swap(owner, Relaxed) != owner {
            // SAFETY: claims change only between cycles (driver thread / construction).
            unsafe { self.slot_mut(k) }.reset();
        }
    }

    /// Insert `id` claims slot `fx` (0 = release). Between cycles only.
    fn claim(&self, id: usize, fx: u8) {
        let fx = fx.min(FX_SLOTS as u8);
        let old = self.claim_of[id].swap(fx, Relaxed);
        if old == fx {
            return;
        }
        let bit = 1u64 << (id % 64);
        if old != 0 {
            let k = old as usize - 1;
            self.claims[k][id / 64].fetch_and(!bit, Relaxed);
            self.recompute_owner(k);
        }
        if fx != 0 {
            let k = fx as usize - 1;
            self.claims[k][id / 64].fetch_or(bit, Relaxed);
            self.recompute_owner(k);
        }
    }

    #[inline]
    fn owns(&self, id: usize, fx: u8) -> bool {
        (1..=FX_SLOTS as u8).contains(&fx) && self.owner[fx as usize - 1].load(Relaxed) == id as u16
    }
}

/// Handle to the FX rack shared by its node and the strips' insert hosts. Cheap to clone.
#[derive(Clone)]
pub struct FxRack {
    shared: Arc<Shared>,
}

impl std::fmt::Debug for FxRack {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("FxRack")
            .field(
                "owners",
                &(1..=FX_SLOTS as u8)
                    .map(|n| self.owner(n))
                    .collect::<Vec<_>>(),
            )
            .finish()
    }
}

impl FxRack {
    /// The 16 slots with every catalog model of wing-blocks (implemented or stub). Allocates.
    pub fn new(fs: f64, max_block: usize) -> Self {
        let mut f = model_factory(BlockKind::Fx, fs, max_block);
        Self::with_factory(fs, max_block, |_slot, i, name| f(i, name))
    }

    /// The 16 slots with models from `factory(slot 1..16, enum index, name)` (`None` = pass-through).
    /// Allocates.
    pub fn with_factory(
        fs: f64,
        max_block: usize,
        mut factory: impl FnMut(u8, u16, &'static str) -> Option<Box<dyn RtModel>>,
    ) -> Self {
        let slots: Box<[SlotCell]> = (1..=FX_SLOTS as u8)
            .map(|n| {
                SlotCell(UnsafeCell::new(FxSlot::new(n, fs, max_block, |i, m| {
                    factory(n, i, m)
                })))
            })
            .collect();
        let mut instances = [0u8; FX_SLOTS];
        for (k, c) in slots.iter().enumerate() {
            // SAFETY: not shared yet.
            instances[k] = unsafe { &*c.0.get() }.model_slot().num_instances() as u8;
        }
        let shared = Shared {
            slots,
            owner: std::array::from_fn(|_| AtomicU16::new(NO_OWNER)),
            claims: std::array::from_fn(|_| std::array::from_fn(|_| AtomicU64::new(0))),
            claim_of: std::array::from_fn(|_| AtomicU8::new(0)),
            latency: std::array::from_fn(|_| AtomicU32::new(0)),
            model: std::array::from_fn(|_| AtomicU16::new(0)),
            instances,
            amix_on: [AtomicBool::new(true), AtomicBool::new(true)],
            amix_group: std::array::from_fn(|_| AtomicU8::new(0)),
            amix_level: std::array::from_fn(|_| AtomicU32::new(0)),
            amix_total: std::array::from_fn(|_| AtomicU32::new(0)),
            fs,
        };
        Self {
            shared: Arc::new(shared),
        }
    }

    /// The insert host for `strip` (give it to the strip's node). Allocates.
    pub fn host(&self, strip: Strip) -> Box<dyn InsertHost + Send> {
        Box::new(RackHost::new(self.shared.clone(), strip))
    }

    /// The graph node (parameters, SEND legs, automix totals). Add it after every strip node.
    pub fn node(&self) -> FxRackNode {
        FxRackNode {
            shared: self.shared.clone(),
        }
    }

    /// Insert owning slot `n` (`$a_chn`/`$a_pos`), if any.
    pub fn owner(&self, n: u8) -> Option<(Strip, InsertPos)> {
        let k = (n as usize).checked_sub(1).filter(|&k| k < FX_SLOTS)?;
        let o = self.shared.owner[k].load(Relaxed);
        if o == NO_OWNER {
            return None;
        }
        let pos = if o.is_multiple_of(2) {
            InsertPos::Pre
        } else {
            InsertPos::Post
        };
        Some((Strip::from_index(o as usize / 2)?, pos))
    }

    /// `$stat` of an insert point.
    pub fn insert_state(&self, strip: Strip, pos: InsertPos) -> InsertStat {
        let Some(id) = insert_id(strip, pos) else {
            return InsertStat::None;
        };
        match self.shared.claim_of[id].load(Relaxed) {
            0 => InsertStat::None,
            fx if self.shared.owns(id, fx) => InsertStat::Ok,
            _ => InsertStat::NotAvailable,
        }
    }

    /// Latency (samples) of slot `n`'s audible model, as of the last committed parameter change.
    pub fn slot_latency(&self, n: u8) -> usize {
        self.shared.latency[n as usize - 1].load(Relaxed) as usize
    }

    /// Selected model (WING enum index of the slot's group) of slot `n`.
    pub fn slot_model(&self, n: u8) -> u16 {
        self.shared.model[n as usize - 1].load(Relaxed)
    }

    /// Number of preallocated model instances of slot `n` (= models its group offers, without `NONE`).
    pub fn slot_instances(&self, n: u8) -> usize {
        self.shared.instances[n as usize - 1] as usize
    }

    /// Run `f` on slot `n` (1..16).
    ///
    /// # Safety
    /// Only while no engine cycle is running (between cycles on the driver thread, or before the engine
    /// starts / after it stopped): during a cycle the owning strip may be mutating the slot.
    pub unsafe fn with_slot<R>(&self, n: u8, f: impl FnOnce(&mut FxSlot) -> R) -> R {
        let k = n as usize - 1;
        let r = f(self.shared.slot_mut(k));
        self.shared.sync_info(k);
        r
    }
}

/// Insert id of `strip`'s insert `pos`.
pub fn insert_id(strip: Strip, pos: InsertPos) -> Option<usize> {
    Some(strip.index()? * 2 + pos as usize)
}

// ---- strip host -------------------------------------------------------------------------------------

/// Automix floor (weighted power) so a silent group shares equally instead of dividing by zero.
const AMIX_FLOOR: f32 = 1e-10;
const AMIX_ATTACK_S: f64 = 0.005;
const AMIX_RELEASE_S: f64 = 0.1;

/// The [`InsertHost`] of one strip: runs the slots this strip owns, forwards claims and automixes.
struct RackHost {
    shared: Arc<Shared>,
    strip: usize,
    /// Automix state (channels only).
    group: u8,
    env: f32,
    weighted: f32,
    gain: f32,
    /// Per-block one-pole coefficients at the last block length.
    coef_len: usize,
    att: f32,
    rel: f32,
    fs: f64,
}

impl RackHost {
    fn new(shared: Arc<Shared>, strip: Strip) -> Self {
        let fs = shared.fs;
        Self {
            shared,
            strip: strip.index().expect("strip"),
            group: 0,
            env: 0.0,
            weighted: 0.0,
            gain: 1.0,
            coef_len: 0,
            att: 1.0,
            rel: 1.0,
            fs,
        }
    }

    fn id(&self, pos: InsertPos) -> usize {
        self.strip * 2 + pos as usize
    }
}

impl InsertHost for RackHost {
    fn process_insert(&mut self, fx: u8, l: &mut [f32], r: &mut [f32]) {
        self.process_insert_at(InsertPos::Pre, fx, l, r);
    }

    fn process_insert_at(&mut self, pos: InsertPos, fx: u8, l: &mut [f32], r: &mut [f32]) {
        if self.shared.owns(self.id(pos), fx) {
            // SAFETY: this insert owns the slot for the whole cycle (see SlotCell).
            unsafe { self.shared.slot_mut(fx as usize - 1) }.process(l, r);
        }
    }

    fn assign(&mut self, pos: InsertPos, fx: u8) {
        self.shared.claim(self.id(pos), fx);
    }

    fn automix_group(&mut self, group: u8) {
        if self.strip >= AMIX_STRIPS {
            return;
        }
        self.group = group.min(2);
        self.shared.amix_group[self.strip].store(self.group, Relaxed);
        self.shared.amix_level[self.strip].store(0, Relaxed);
        self.env = 0.0;
        self.weighted = 0.0;
        self.gain = 1.0;
    }

    fn automix(&mut self, weight_db: f32, l: &mut [f32], r: &mut [f32]) {
        let n = l.len();
        if self.group == 0 || self.strip >= AMIX_STRIPS || n == 0 {
            return;
        }
        let g = self.group as usize - 1;
        if n != self.coef_len {
            self.coef_len = n;
            let blk = n as f64 / self.fs;
            self.att = (1.0 - (-blk / AMIX_ATTACK_S).exp()) as f32;
            self.rel = (1.0 - (-blk / AMIX_RELEASE_S).exp()) as f32;
        }
        let p = l
            .iter()
            .zip(r.iter())
            .map(|(a, b)| a * a + b * b)
            .sum::<f32>()
            / (2 * n) as f32;
        let c = if p > self.env { self.att } else { self.rel };
        self.env += c * (p - self.env);
        let w = self.env * 10f32.powf(weight_db.clamp(-12.0, 12.0) * 0.1) + AMIX_FLOOR;
        // others' levels of the previous block + this member's current level
        let total_prev = f32::from_bits(self.shared.amix_total[g].load(Relaxed));
        let total = (total_prev - self.weighted).max(0.0) + w;
        self.weighted = w;
        self.shared.amix_level[self.strip].store(w.to_bits(), Relaxed);
        let target = if self.shared.amix_on[g].load(Relaxed) {
            (w / total).sqrt().min(1.0)
        } else {
            1.0
        };
        let (g0, step) = (self.gain, (target - self.gain) / n as f32);
        for (i, (a, b)) in l.iter_mut().zip(r.iter_mut()).enumerate() {
            let gi = g0 + step * (i + 1) as f32;
            *a *= gi;
            *b *= gi;
        }
        self.gain = target;
    }

    fn insert_latency(&self, pos: InsertPos, fx: u8) -> usize {
        if self.shared.owns(self.id(pos), fx) {
            self.shared.latency[fx as usize - 1].load(Relaxed) as usize
        } else {
            0
        }
    }
}

// ---- node -------------------------------------------------------------------------------------------

/// The FX rack's graph node; see the module doc. Outputs: [`SEND_OUTPUTS`] FX SEND legs. Parameter
/// blocks: slot `n` = block `n - 1` ([`FX_LAYOUT`]), [`block::AMIX`] ([`AMIX_LAYOUT`]).
pub struct FxRackNode {
    shared: Arc<Shared>,
}

impl FxRackNode {
    /// End-of-cycle work: publish nothing, just clear the SEND captures and sum the automix groups
    /// (used by [`Processor::process`]; tests driving strips without a graph call it directly).
    pub fn end_cycle(&mut self) {
        for k in 0..FX_SLOTS {
            // SAFETY: runs after every strip level (see SlotCell).
            unsafe { self.shared.slot_mut(k) }.fed = false;
        }
        self.sum_automix();
    }

    fn sum_automix(&self) {
        let s = &self.shared;
        let mut total = [0.0f32; 2];
        for i in 0..AMIX_STRIPS {
            let g = s.amix_group[i].load(Relaxed);
            if g != 0 {
                total[g as usize - 1] += f32::from_bits(s.amix_level[i].load(Relaxed));
            }
        }
        for (g, t) in total.iter().enumerate() {
            s.amix_total[g].store(t.to_bits(), Relaxed);
        }
    }
}

impl Processor for FxRackNode {
    fn process(&mut self, _ctx: &ProcessCtx, io: &mut NodeIo<'_>) {
        let b = io.block_size();
        let n_out = io.num_outputs().min(SEND_OUTPUTS as usize);
        let out = io.outputs();
        for k in 0..FX_SLOTS {
            if 2 * k + 1 >= n_out {
                break;
            }
            // SAFETY: runs after every strip level (see SlotCell).
            let slot = unsafe { self.shared.slot_mut(k) };
            let (ol, rest) = out[2 * k * b..(2 * k + 2) * b].split_at_mut(b);
            match slot.feed() {
                Some((fl, fr)) if fl.len() == b => {
                    ol.copy_from_slice(fl);
                    rest.copy_from_slice(fr);
                }
                _ => {
                    ol.fill(0.0);
                    rest.fill(0.0);
                }
            }
        }
        self.end_cycle();
    }

    fn set_param(&mut self, msg: &ParamMsg) {
        let s = &self.shared;
        match *msg {
            ParamMsg::Set {
                block: block::AMIX,
                param,
                value,
            } => {
                if let Some(on) = s.amix_on.get(param as usize) {
                    on.store(value != 0.0, Relaxed);
                }
            }
            ParamMsg::Set { block, .. } | ParamMsg::Model { block, .. }
                if (block as usize) < FX_SLOTS =>
            {
                // SAFETY: driver thread between cycles.
                unsafe { s.slot_mut(block as usize) }.apply(msg);
            }
            ParamMsg::Commit => {
                for k in 0..FX_SLOTS {
                    // SAFETY: driver thread between cycles.
                    unsafe { s.slot_mut(k) }.apply(msg);
                    s.sync_info(k);
                }
            }
            _ => {}
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use wing_model::Schema;

    #[test]
    fn slot_groups_and_restrictions() {
        let rack = FxRack::new(48_000.0, 64);
        let s = Schema::get();
        for n in 1..=16u8 {
            let g = s.groups().iter().find(|g| g.key == group_key(n)).unwrap();
            // every model of the group but NONE is preallocated
            assert_eq!(rack.slot_instances(n), s.models(g).len() - 1, "FX{n}");
        }
        assert_eq!(rack.slot_instances(1), 64);
        assert_eq!(rack.slot_instances(9), 38);
    }

    #[test]
    fn lowest_insert_owns_and_release_hands_over() {
        let rack = FxRack::new(48_000.0, 64);
        let mut ch3 = rack.host(Strip::Ch(3));
        let mut bus1 = rack.host(Strip::Bus(1));
        bus1.assign(InsertPos::Post, 5);
        assert_eq!(rack.owner(5), Some((Strip::Bus(1), InsertPos::Post)));
        ch3.assign(InsertPos::Pre, 5);
        assert_eq!(rack.owner(5), Some((Strip::Ch(3), InsertPos::Pre)));
        assert_eq!(
            rack.insert_state(Strip::Ch(3), InsertPos::Pre),
            InsertStat::Ok
        );
        assert_eq!(
            rack.insert_state(Strip::Bus(1), InsertPos::Post),
            InsertStat::NotAvailable
        );
        assert_eq!(
            rack.insert_state(Strip::Bus(1), InsertPos::Pre),
            InsertStat::None
        );
        ch3.assign(InsertPos::Pre, 0);
        assert_eq!(rack.owner(5), Some((Strip::Bus(1), InsertPos::Post)));
        bus1.assign(InsertPos::Post, 6);
        assert_eq!(rack.owner(5), None);
        assert_eq!(rack.owner(6), Some((Strip::Bus(1), InsertPos::Post)));
    }
}
