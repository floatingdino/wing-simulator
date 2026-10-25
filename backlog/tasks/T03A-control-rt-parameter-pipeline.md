---
id: T03A
title: Control->RT parameter pipeline
stage: S3
status: review
owner: agent-T03A
depends: [T010, T022]
branch: task/T03A
---

## Goal
Lock-free param change transport into RT engine with per-parameter smoothing and model hot-swap without clicks.

## Acceptance criteria
- [x] Zipper-noise test
- [x] No RT alloc on model swap (preallocated)

## Log
- 2026-10-25 coordinator: created.
- 2026-10-25 agent-T03A: implemented in `crates/wing-engine/src/params/` (new module; routing code untouched).
  - Design (full rationale in `params/mod.rs` docs): `ParamMapBuilder` (off-RT) builds `node -> (graph node, block,
    plain|selector|target)` tables plus per-model-group `(model, pos) -> (expected Desc, default)` tables.
    `ParamPipeline::apply` (RT driver thread, before each cycle) pops <= `budget` (default 1024) changes from
    wing-model's coalescing queue, converts raw -> f32 engineering value via the change's descriptor, delivers
    `ParamMsg::{Set, Model, Commit}` through new default methods `Processor::set_param` / `Processor::telemetry`
    (graph.rs, 2 default methods), and commits each touched node once. `run_cycle(engine)` = apply + cycle + telemetry.
  - Overflow: the wing-model queue holds each node once (capacity = node count) so it cannot overflow; over-budget
    changes stay queued and coalesce; saturated cycles counted in `PipelineStats`.
  - Ordering: per-target mirror (raw + Desc); a target is forwarded only if its Desc is what the RT's current model
    expects; on a selector change the dispatcher sends `Model` then the new model's full set (mirror if Desc
    matches, else model default). Never interprets a value under the wrong model; converges to store state.
  - Recall: chosen "stage in store, apply incrementally with a per-cycle budget" over a precomputed-frame swap
    (store already stages; bounded RT work; frame swap needs a second copy of all block state + per-processor
    hand-over and still needs smoothing). Recall lands over ~30 cycles (40 ms), click-free via smoothing/xfades.
  - Model hot-swap (generic per coordinator note): `params::strip::RtModel` trait is the ONLY interface the swap
    mechanism needs -- `load_defaults()`, `set_param(pos: u16, value: f32)` (0-based wing-model slot position,
    engineering units; enums = index, float-enums = value), `param(pos) -> Option<f32>` (introspection, default
    None), `commit()` (block rate), `reset()`, `process(l, r)` (stereo in place, <= max_block), `gain_reduction_db()`.
    `ModelSlot::from_group(group_key, fs, max_block, factory(enum_index, name) -> Option<Box<dyn RtModel>>)`
    preallocates one instance per model of the wing-model group (None = pass-through), crossfades 5 ms linear
    (reverse if re-selecting the fading-out model, defer otherwise), smoothed on/mix/gain wrapper (5 ms).
    `params/std_models.rs` = interim adapters (GATE/DUCK/COMP/EXP) + `std_factory`; T045 should replace it with its
    registry implementing `RtModel`.
  - `ParamStrip` (reference host for tests/bench: input -> filter -> gate slot -> dyn slot -> fader) and
    `bind_channel()` + layouts (`INPUT/FILTER/SLOT/FADER_LAYOUT`); `FaderGain` = fader+mute as 5 ms one-pole on
    linear gain (exact 0 once settled). Telemetry: `TelemetryWriter`/`TelemetryRx` over a fixed-capacity lock-free
    ring (wing-model `BoundedQueue`, used SPSC), drop+count when full, polled every 16 cycles (GR per slot).
  - No wing-model / wing-blocks edits. wing-engine now depends on wing-model, wing-blocks, wing-dsp.
  - Files: crates/wing-engine/{Cargo.toml, src/lib.rs, src/graph.rs (Processor default methods),
    src/params/{mod,map,pipeline,telemetry,strip,std_models}.rs, tests/params.rs, tests/params_alloc.rs,
    benches/params.rs}.
  - Tests (`cargo test -p wing-engine --test params --test params_alloc`): ordering with budget 1 (target queued
    before selector never delivered under old model), split selector/defaults uses new model defaults, sync_all ==
    store, scene recall (full-tree recall + second recall mid-apply: 56.7k sets, 35.4k queue entries, 35 cycles <=
    ceil(nodes/budget)=43, RT state == store for 40 ch), zipper (fader -40->0 dB in 50 steps, 1 kHz: HF>3 kHz
    -96.3 dB smoothed vs -61.8 dB unsmoothed), model swap click (GATE/EXP/DUCK/LA/COMP/...: max 2nd-diff 1.00x
    steady state with xfade vs 7.55x hard switch), telemetry GR, Engine integration, zero RT allocations (apply
    during recall, model swaps on 40 ch x 2 slots, process, telemetry; counting allocator).
  - Bench (`cargo bench -p wing-engine --bench params`, 40 strips, M-series): 0 changes 16 ns, 10 -> 113 ns,
    1000 -> 9.5 us, all 1480 bound -> 14.8 us (2 cycles); one full-budget cycle 9.8 us (< 1 % of 1.33 ms).
  - `make check` passes.
  - Proposed follow-ups: wire `ParamPipeline` into the daemon's LiveClock/CycleIo path (T023/app); bind gatesc/dynsc
    (side-chain filter/source), EQ (StdEq now on main), pan/width/sends when T035/T036 land; replace std_models with
    T045 registry; meters (T039) via the same telemetry ring or per-meter atomics.
