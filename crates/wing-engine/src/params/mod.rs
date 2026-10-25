//! Control -> RT parameter pipeline and RT -> control telemetry (T03A).
//!
//! # Data flow
//!
//! ```text
//!  control threads                      RT driver thread (before each cycle)        processors
//!  ParamStore::set*  --coalescing-->    ParamPipeline::apply                        set_param(ParamMsg)
//!  (wing-model)        change queue      pop <= budget changes                        Set / Model / Commit
//!                      (RtConsumer)      node_map[id] -> (graph node, block, param)
//!                                        raw -> f32 engineering value (desc-driven)
//!                                        model ordering + mirror (see below)
//!                                        Commit once per touched node
//!  TelemetryRx::pop  <--SPSC ring---    ParamPipeline::publish_telemetry (after cycle) telemetry(writer)
//! ```
//!
//! * **Tables** ([`ParamMapBuilder`], off the RT thread): one `u32` per wing-model node (43k) mapping
//!   to a binding `(graph node, block index, kind)`; kinds are a plain parameter, a model selector or a
//!   model-dependent target. Per model group one table `(model, target position) -> (expected
//!   descriptor, model default)`. Everything is allocated in `build`/`ParamPipeline::new`.
//! * **Conversion**: the dispatcher turns the raw store encoding into an `f32` engineering value
//!   using the *descriptor the change was stored under* (floats as-is in dB/Hz/ms/%, ints, enum
//!   indices, float-enum values such as compressor ratios). Block-specific mapping (enum index ->
//!   typed enum, dB -> linear for faders) happens in the processor's `set_param` (see
//!   [`strip::ParamStrip`]); typed `set_params` of a block runs once per cycle on `Commit`.
//! * **Smoothing**: blocks that already smooth (input gains/delay, filter cutoffs, dynamics
//!   mix/makeup) get raw targets; the pipeline adds smoothing where nothing else does: fader/mute
//!   ([`strip::FaderGain`], 5 ms one-pole on the linear gain), model-slot on/mix/gain (5 ms linear
//!   ramps) and model swaps (crossfade).
//!
//! # Ordering guarantee (selector before model parameters)
//!
//! A model switch in wing-model stores the selector, swaps the targets' descriptors and stores the
//! new model's defaults, each one queued. Because the queue coalesces by node and the drain is
//! bounded, the RT side can see a target before its selector, or the selector in one cycle and
//! the targets in a later one. The dispatcher therefore keeps a *mirror* (raw + descriptor) of every
//! bound target and forwards a target value only if its descriptor is the one the block's current
//! model expects at that position. On a selector change it sends `Model` first, then the full
//! parameter set of the new model: mirrored values whose descriptor matches, model defaults for the
//! rest. A block therefore never interprets a value under the wrong model, and once the queue is
//! drained its state equals the store's.
//!
//! # Bounded work and overflow
//!
//! `apply` pops at most [`PipelineConfig::budget`] changes per cycle. The wing-model queue holds
//! each parameter at most once and its capacity is the node count, so it **cannot overflow**:
//! anything over budget simply stays queued (coalescing further: only the latest value is ever
//! applied) and is applied in the next cycles. A saturated cycle is counted in [`PipelineStats`].
//! A selector change expands to at most `1 + targets` messages (19 for channel slots), so the
//! worst-case work per cycle is `budget * 20` messages plus one `Commit` per touched node.
//!
//! # Snapshot recall
//!
//! Choice: **stage in the store and apply incrementally over several cycles** (not a whole
//! precomputed parameter-frame swap). Rationale:
//! * The store already *is* the staging area: a recall writes ~30k values into atomics on the
//!   control thread and enqueues node ids; nothing can overflow and nothing allocates.
//! * The per-cycle budget gives a hard bound on RT work regardless of recall size (30k changes at
//!   the default 1024/cycle complete in 30 cycles = 40 ms at 48 kHz/64), so a recall cannot cause
//!   an xrun. A frame swap is O(1) on the RT side but needs a second full copy of every block's typed
//!   state built off-thread, a hand-over protocol per processor, and still has to smooth/crossfade
//!   every changed parameter to avoid clicks; it buys atomicity we do not need.
//! * Cost: a recall is not sample-atomic (it lands over a few tens of ms). Each parameter is
//!   smoothed and model swaps crossfade, so the transition is click-free; per-block consistency
//!   (e.g. a model with all its parameters) is guaranteed by the ordering rules above.
//!
//! # Model hot-swap
//!
//! The swap mechanism depends only on the [`strip::RtModel`] trait (`load_defaults`,
//! `set_param(pos, value)`, `param`, `commit`, `reset`, `process(l, r)`, `gain_reduction_db`);
//! [`std_models`] implements it for wing-blocks' `Box<dyn Model>` (T045) and provides
//! `model_factory` (every catalog model, stubs included). Model slots ([`strip::ModelSlot::from_group`]) take a factory and
//! preallocate one instance per model of the wing-model group at construction (models without an
//! implementation pass audio through). A `Model` message resets the incoming instance, loads the
//! model defaults (then the dispatcher's parameter set) and crossfades
//! old -> new over [`strip::MODEL_XFADE_S`] (linear, equal-gain: both paths process the same input, so
//! their outputs are strongly correlated). Swaps during a crossfade reverse it (back to the previous
//! model) or are deferred until it completes. No allocation anywhere on these paths.
//!
//! # Telemetry
//!
//! Processors registered as telemetry sources get `telemetry(writer)` every
//! [`PipelineConfig::telemetry_every`] cycles after the cycle ran; records go into a fixed-capacity
//! lock-free ring used single-producer/single-consumer ([`TelemetryRx`]). A full ring drops the
//! record and counts it (telemetry is latest-value data; the control side must keep up).

mod map;
mod pipeline;
pub mod std_models;
pub mod strip;
mod telemetry;

pub use map::{BindError, BlockLayout, ParamMap, ParamMapBuilder, TARGET_BASE};
pub use pipeline::{
    ApplyReport, ParamPipeline, PipelineConfig, PipelineError, PipelineStats, StatsSnapshot,
};
pub use telemetry::{kind as telemetry_kind, Telemetry, TelemetryRx, TelemetryWriter};

use crate::graph::{Graph, NodeId, Processor};
use crate::sched::Engine;

/// A parameter message delivered to a processor's [`Processor::set_param`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ParamMsg {
    /// A parameter value in engineering units (dB, Hz, ms, %, 0/1, enum index, float-enum value).
    /// `param < TARGET_BASE`: fixed parameter id from the block's [`BlockLayout::params`];
    /// `param >= TARGET_BASE`: model target `param - TARGET_BASE` of the block's current model.
    Set { block: u8, param: u16, value: f32 },
    /// The block's model selector changed to the WING enum index `model`. Always precedes the new
    /// model's parameter values.
    Model { block: u8, model: u16 },
    /// End of this cycle's messages for the node: apply accumulated changes.
    Commit,
}

/// Access to processors by graph node id (implemented for [`Engine`] and [`Graph`]; tests may
/// implement it for plain collections).
pub trait ProcessorAccess {
    fn processor(&mut self, id: NodeId) -> &mut dyn Processor;
}

impl ProcessorAccess for Engine {
    #[inline]
    fn processor(&mut self, id: NodeId) -> &mut dyn Processor {
        self.processor_mut(id)
    }
}

impl ProcessorAccess for Graph {
    #[inline]
    fn processor(&mut self, id: NodeId) -> &mut dyn Processor {
        self.processor_mut(id)
    }
}
