//! The RT-side dispatcher: drains wing-model's change queue and feeds processors.

use std::fmt;
use std::sync::atomic::{AtomicU64, Ordering::Relaxed};
use std::sync::Arc;
use std::time::Duration;

use wing_model::{Desc, NodeId as ParamId, ParamStore, ParamType, RtConsumer, Schema};

use super::map::{Kind, ParamMap, NONE, NO_MODEL, TARGET_BASE};
use super::telemetry::{self, TelemetryRx, TelemetryWriter};
use super::{ParamMsg, ProcessorAccess};
use crate::graph::NodeId;
use crate::sched::Engine;

/// Pipeline tuning.
#[derive(Debug, Clone, Copy)]
pub struct PipelineConfig {
    /// Maximum queue entries popped per cycle (each expands to 1 message, or `1 + targets` for a
    /// model switch). Default 1024: the `params` bench (40 strips, M-series) measures ~16 ns with
    /// nothing queued, ~0.11 us for 10 and ~9.5 us for 1000 changes, i.e. a full budget is < 1 % of
    /// a 1.33 ms cycle (48 kHz / 64); even the pathological all-selectors case (20 messages per
    /// entry) stays well inside the cycle.
    pub budget: usize,
    /// Poll telemetry sources every N cycles (default 16 = 47 Hz at 48 kHz / 64).
    pub telemetry_every: u32,
    /// Telemetry ring capacity (rounded up to a power of two).
    pub telemetry_capacity: usize,
}

impl Default for PipelineConfig {
    fn default() -> Self {
        Self {
            budget: 1024,
            telemetry_every: 16,
            telemetry_capacity: 4096,
        }
    }
}

/// Construction errors.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PipelineError {
    /// The store's RT consumer was already taken.
    ConsumerTaken,
}

impl fmt::Display for PipelineError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str("the ParamStore RT consumer was already taken")
    }
}

impl std::error::Error for PipelineError {}

/// What one `apply` did.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct ApplyReport {
    /// Queue entries popped.
    pub changes: usize,
    /// Messages delivered (excluding commits).
    pub messages: usize,
    /// Nodes committed.
    pub nodes: usize,
    /// The budget was exhausted (more changes may be pending).
    pub saturated: bool,
}

/// Lock-free counters readable from any thread.
#[derive(Debug, Default)]
pub struct PipelineStats {
    changes: AtomicU64,
    messages: AtomicU64,
    saturated_cycles: AtomicU64,
    max_changes_per_cycle: AtomicU64,
}

/// Plain copy of [`PipelineStats`].
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct StatsSnapshot {
    pub changes: u64,
    pub messages: u64,
    pub saturated_cycles: u64,
    pub max_changes_per_cycle: u64,
}

impl PipelineStats {
    pub fn snapshot(&self) -> StatsSnapshot {
        StatsSnapshot {
            changes: self.changes.load(Relaxed),
            messages: self.messages.load(Relaxed),
            saturated_cycles: self.saturated_cycles.load(Relaxed),
            max_changes_per_cycle: self.max_changes_per_cycle.load(Relaxed),
        }
    }
}

/// Control -> RT parameter dispatcher plus RT -> control telemetry publisher. Lives on the RT
/// driver thread; [`ParamPipeline::apply`] before and [`ParamPipeline::publish_telemetry`] after
/// each cycle (or [`ParamPipeline::run_cycle`]). See the module docs for the design.
pub struct ParamPipeline {
    schema: &'static Schema,
    rx: RtConsumer,
    map: ParamMap,
    mirror_raw: Box<[u32]>,
    mirror_desc: Box<[Option<&'static Desc>]>,
    cur_model: Box<[u16]>,
    touched: Vec<u32>,
    touched_flag: Box<[bool]>,
    cfg: PipelineConfig,
    stats: Arc<PipelineStats>,
    telem: Arc<telemetry::Shared>,
    messages: usize,
}

/// Engineering value of a raw store value under `desc` (`None` for strings).
#[inline]
pub(crate) fn convert(desc: &Desc, raw: u32) -> Option<f32> {
    Some(match desc.ty {
        ParamType::LinF | ParamType::LogF | ParamType::Fader => f32::from_bits(raw),
        ParamType::Int => raw as i32 as f32,
        ParamType::Enum => raw as f32,
        ParamType::FEnum => desc.as_f64(raw) as f32,
        ParamType::Str => return None,
    })
}

impl ParamPipeline {
    /// Take `store`'s RT consumer and prepare the mirrors from the store's current state.
    /// Allocates (control thread). Call [`ParamPipeline::sync_all`] before the first cycle to push
    /// the full state into the processors.
    pub fn new(
        store: &ParamStore,
        map: ParamMap,
        cfg: PipelineConfig,
    ) -> Result<Self, PipelineError> {
        let rx = store
            .take_rt_consumer()
            .ok_or(PipelineError::ConsumerTaken)?;
        let mirror_raw = map.mirror_ids.iter().map(|&id| store.get_raw(id)).collect();
        let mirror_desc = map.mirror_ids.iter().map(|&id| store.desc(id)).collect();
        let n_nodes = map.num_graph_nodes;
        Ok(Self {
            schema: map.schema,
            rx,
            cur_model: vec![NO_MODEL; map.insts.len()].into(),
            mirror_raw,
            mirror_desc,
            touched: Vec::with_capacity(n_nodes),
            touched_flag: vec![false; n_nodes].into(),
            stats: Arc::new(PipelineStats::default()),
            telem: telemetry::Shared::new(cfg.telemetry_capacity),
            cfg,
            map,
            messages: 0,
        })
    }

    /// The dispatch tables.
    pub fn map(&self) -> &ParamMap {
        &self.map
    }

    /// Change the per-cycle budget (>= 1).
    pub fn set_budget(&mut self, budget: usize) {
        self.cfg.budget = budget.max(1);
    }

    /// Shared statistics (clone to a control thread).
    pub fn stats(&self) -> Arc<PipelineStats> {
        self.stats.clone()
    }

    /// The control-side telemetry receiver (only once).
    pub fn telemetry_rx(&self) -> Option<TelemetryRx> {
        (!self.telem.rx_taken.swap(true, Relaxed)).then(|| TelemetryRx {
            shared: self.telem.clone(),
        })
    }

    /// Current model (WING enum index) the RT side holds for the `i`-th bound model block, if any.
    pub fn current_model(&self, i: usize) -> Option<u16> {
        self.cur_model.get(i).copied().filter(|&m| m != NO_MODEL)
    }

    #[inline]
    fn send<P: ProcessorAccess + ?Sized>(&mut self, procs: &mut P, node: u32, msg: ParamMsg) {
        procs.processor(NodeId(node)).set_param(&msg);
        self.messages += 1;
        let f = &mut self.touched_flag[node as usize];
        if !*f {
            *f = true;
            // capacity == number of graph nodes and each node is pushed once: never reallocates
            self.touched.push(node);
        }
    }

    /// Dispatch one change. RT-safe.
    #[inline]
    fn dispatch<P: ProcessorAccess + ?Sized>(
        &mut self,
        procs: &mut P,
        id: u16,
        desc: &'static Desc,
        raw: u32,
    ) {
        let b = match self.map.node_map.get(id as usize) {
            Some(&b) if b != NONE => self.map.bindings[b as usize],
            _ => return,
        };
        match b.kind {
            Kind::Plain { param } => {
                if let Some(value) = convert(desc, raw) {
                    self.send(
                        procs,
                        b.node,
                        ParamMsg::Set {
                            block: b.block,
                            param,
                            value,
                        },
                    );
                }
            }
            Kind::Target { inst, pos } => {
                let i = &self.map.insts[inst as usize];
                let ix = i.base as usize + pos as usize;
                self.mirror_raw[ix] = raw;
                self.mirror_desc[ix] = Some(desc);
                let cur = self.cur_model[inst as usize];
                if cur == NO_MODEL {
                    return;
                }
                let t = &self.map.tables[i.table as usize];
                // Forward only when stored under the descriptor the current model expects at this
                // position; otherwise the selector change is still pending (it will pick the value
                // up from the mirror).
                if t.expected(cur, pos as usize)
                    .is_some_and(|e| std::ptr::eq(e, desc))
                {
                    if let Some(value) = convert(desc, raw) {
                        self.send(
                            procs,
                            b.node,
                            ParamMsg::Set {
                                block: b.block,
                                param: TARGET_BASE + pos,
                                value,
                            },
                        );
                    }
                }
            }
            Kind::Selector { inst } => self.select(procs, inst as usize, raw),
        }
    }

    /// Model selector change: `Model` first, then the new model's complete parameter set.
    fn select<P: ProcessorAccess + ?Sized>(&mut self, procs: &mut P, inst: usize, raw: u32) {
        let i = &self.map.insts[inst];
        let (node, block, base, table) = (i.node, i.block, i.base as usize, i.table as usize);
        if !self.map.tables[table].is_valid(raw) || self.cur_model[inst] == raw as u16 {
            return;
        }
        let model = raw as u16;
        self.cur_model[inst] = model;
        self.send(procs, node, ParamMsg::Model { block, model });
        let n = self.map.tables[table].n_targets;
        for pos in 0..n {
            let t = &self.map.tables[table];
            let Some(exp) = t.expected(model, pos) else {
                continue;
            };
            let ix = base + pos;
            let r = match self.mirror_desc[ix] {
                Some(d) if std::ptr::eq(d, exp) => self.mirror_raw[ix],
                _ => t.default(model, pos),
            };
            if let Some(value) = convert(exp, r) {
                self.send(
                    procs,
                    node,
                    ParamMsg::Set {
                        block,
                        param: TARGET_BASE + pos as u16,
                        value,
                    },
                );
            }
        }
    }

    fn commit<P: ProcessorAccess + ?Sized>(&mut self, procs: &mut P) -> usize {
        let n = self.touched.len();
        for k in 0..n {
            let node = self.touched[k];
            self.touched_flag[node as usize] = false;
            procs.processor(NodeId(node)).set_param(&ParamMsg::Commit);
        }
        self.touched.clear();
        n
    }

    /// Drain up to `budget` queued changes and deliver them, then `Commit` every touched node.
    /// RT-safe: no allocation, locks or syscalls. Call on the driver thread before the cycle.
    pub fn apply<P: ProcessorAccess + ?Sized>(&mut self, procs: &mut P) -> ApplyReport {
        self.messages = 0;
        let mut changes = 0;
        while changes < self.cfg.budget {
            let Some(c) = self.rx.pop() else { break };
            changes += 1;
            if c.desc == u16::MAX {
                continue;
            }
            let desc = self.schema.desc_at(c.desc);
            self.dispatch(procs, c.id, desc, c.raw);
        }
        let messages = self.messages;
        let nodes = self.commit(procs);
        let saturated = changes == self.cfg.budget;
        if changes > 0 {
            let s = &self.stats;
            s.changes.fetch_add(changes as u64, Relaxed);
            s.messages.fetch_add(messages as u64, Relaxed);
            s.max_changes_per_cycle.fetch_max(changes as u64, Relaxed);
            if saturated {
                s.saturated_cycles.fetch_add(1, Relaxed);
            }
        }
        ApplyReport {
            changes,
            messages,
            nodes,
            saturated,
        }
    }

    /// Push the store's complete current state for every bound parameter (selectors first) and
    /// commit. Use before starting audio or after re-binding. Allocation-free, but O(bound params).
    pub fn sync_all<P: ProcessorAccess + ?Sized>(&mut self, store: &ParamStore, procs: &mut P) {
        for k in 0..self.map.mirror_ids.len() {
            let id = self.map.mirror_ids[k];
            self.mirror_raw[k] = store.get_raw(id);
            self.mirror_desc[k] = store.desc(id);
        }
        self.cur_model.fill(NO_MODEL);
        for k in 0..self.map.insts.len() {
            let sel = self.map.insts[k].selector;
            self.select(procs, k, store.get_raw(sel));
        }
        for k in 0..self.map.bound.len() {
            let id: ParamId = self.map.bound[k];
            if let Some(d) = store.desc(id) {
                self.dispatch(procs, id.0, d, store.get_raw(id));
            }
        }
        self.commit(procs);
    }

    /// Poll telemetry sources if `cycle` is a polling cycle. RT-safe. Call after the cycle.
    pub fn publish_telemetry<P: ProcessorAccess + ?Sized>(&mut self, procs: &mut P, cycle: u64) {
        let every = self.cfg.telemetry_every.max(1) as u64;
        if !cycle.is_multiple_of(every) {
            return;
        }
        for k in 0..self.map.telemetry_nodes.len() {
            let node = self.map.telemetry_nodes[k];
            let mut w = TelemetryWriter {
                shared: &self.telem,
                node: node.0,
                cycle: cycle as u32,
            };
            procs.processor(node).telemetry(&mut w);
        }
    }

    /// `apply`, one engine cycle, `publish_telemetry`. RT-safe.
    pub fn run_cycle(&mut self, engine: &mut Engine) -> Duration {
        self.apply(engine);
        let dt = engine.process_cycle();
        let c = engine.cycle();
        self.publish_telemetry(engine, c);
        dt
    }
}
