---
id: T035
title: Channel strip assembly + process order
stage: S3
status: done
owner: agent-T045-T035
depends: [T031, T032, T033, T034]
branch: task/T045-T035
---

## Goal
Assemble channel strip with WING process order options (e.g. pre/post EQ dyn, insert positions), bypass semantics, stereo-linked processing.

## Acceptance criteria
- [x] Order switch tests
- [x] 40ch strip bench at 64 samples

## Log
- 2026-10-25 coordinator: created.
- 2026-10-25 agent-T045-T035: done, status review (branch task/T045-T035).
  - Placement: `crates/wing-engine/src/channel.rs` (not wing-blocks): the strip binds schema paths through
    T03A's `ParamMapBuilder`/`ParamMsg`, hosts models in the engine's crossfading `ModelSlot`, takes keys and
    insert FX from the graph and publishes telemetry; wing-blocks stays schema-free DSP.
  - `ChannelStrip` (kind Channel / Aux) + `bind_channel_strip(ch, node)` / `bind_aux_strip(aux, node)`:
    ch = in/set, flt (LC/HC + filter tool slot `tf`/`mdl`/1/2), gate (+gatesc), peq, eq (+mix up to 125 %),
    dyn (+dynsc), preins, postins (mode FX/AUTO_X/AUTO_Y, target `ins`), `proc`, `ptap`; aux = in/set, preins,
    eq, dyn (+dynsc). Order: input -TAP IN-> LC -> HC -> tool -TAP FILT-> preins -> 4 blocks in `proc` order
    (taps 3/4/5/PFL) ; aux fixed input -> preins -> EQ -> dyn. `PROC_ORDERS` checked against the schema enum.
  - Bypass: per-block switches; model slots fade 5 ms and are skipped when off; full-wet path is bit-exact.
    Stereo-linked (always stereo, linked detectors). Side-chain: key filter in the slot, external key buffers
    (graph inputs 2/3 gate, 4/5 dyn) used only when `src` != SELF. Inserts call an `InsertHost` (FX rack T050;
    `NoInserts` passthrough; automix modes pass through). Telemetry: tap peaks (peak-hold, `TAP_PEAK` kind
    added) + gate/dyn GR. `ptap` capture in `tap_signal()` / outputs 2/3 (AFL/POST captured as PFL until T036).
  - Tests: `crates/wing-engine/tests/channel_strip.rs` (strip == hand-built block chain bit-exact for all 24
    orders; mid-stream order switch; bypass; every schema model of ch gate/dyn/eq/flt and aux dyn selectable
    with an instance; external key/key filter; ptap/tap peaks; schema binding through the pipeline incl.
    unidyn COMP in the gate slot, aux models, telemetry), `tests/channel_no_alloc.rs` (all model switches,
    params, order changes, processing: 0 allocations).
  - Bench `cargo bench -p wing-engine --bench strip`: 40 ch + 8 aux, all blocks on (trim, LC24+HC, tool stub,
    GATE, PRE EQ, STD EQ 5 bands, COMP at 80 % mix), 64 samples, one thread: 107.9 us/cycle = 8.1 % of
    1333 us (2.24 us/strip), M-series.
  - Note: main's Cargo.lock had committed conflict markers (T03A/T023 merge); fixed on this branch.
  - Proposed follow-ups: dynxo crossover (frequency-selective dyn) not implemented (no block yet); automix
    AUTO_X/Y (T036/T038); sc `tap` -> key buffer resolution in routing; mono-source fast path (process one
    side) if CPU matters; `$g` binding vs source preamp gain should be revisited with T024 routing.
