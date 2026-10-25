---
id: T082
title: BUG: metering for FX blocks and non-default dynamics models doesn't work
stage: S4
status: review
owner: agent-T082
depends: []
branch: task/T082
---

## Goal
User: "FX block / non-default dynamics models monitoring largely doesn't work". Make WING-Edit's meters work for: gain-reduction + key meters of every gate/dyn model (not only GATE/DUCK/COMP/EXP), dynamic-EQ band activity if WING reports it, FX slot input/output meters and any model-specific meters WING-Edit requests (check the meter request sets in docs/research/wing-edit-session.md / protocol.md and what WING-Edit asks for when an FX or dyn page is open — capture from the user's live log scratch/wing-sim-v0.4.log or later logs: ch3 meter report definitions). Wire telemetry from every Model into LevelBank. Tests per model family.

## Acceptance criteria
- [x] GR meter moves for every dyn/gate model under compression (test)
- [x] FX in/out meters populated; any FX-specific meters WING-Edit requests populated or documented
- [x] Zero RT alloc

## Log
- 2026-10-26 coordinator: created from user report.
- 2026-10-26 agent-T082: done, status review.
  - **Findings.** `scratch/wing-sim-v0.4.log` only logs meter counts per report, not the types, so the requests
    could not be decoded from it; the server now logs the selections (`report 7 defined (3 meters: ChannelV2
    1-2, Fx 5)`) for future captures. Decoded instead from WING-Edit 3.3.4 (`nrp::MeterHandler::rxMeters`,
    FX page listeners, `DynEqGraph::updateCurve`; static analysis):
    1. The gate/dyn gain word is a **GR-meter position × 256 using the selected model's law** (WING-Edit's
       `*_gain_<model>`: GATE/DUCK 60 dB, most comps 20 dB, E88/B560/D241C/NSTR capped >1, 76LA/L100/RIDE
       negative, B160/SBUS VU needle `-0.3 g`, ONEC `g/ln10`, WAVE 1, WARM 0). We sent a slot-based scale
       (gate 60 dB, dyn 20 dB), so every model whose law differs (and every comp in a gate slot) read wrong.
       DEQ2 uses the gain + LED word as two band gains `256 (g_k - 1)`; DEQ's word is band dB / 15.
    2. FX `A6` blocks: words 0-3 = in/out levels (were constant silence), words 4-9 = model state read by
       *EVEN*/*SOUL*/*VINTAGE*/*BUS* (dynamics LEDs, `2048 g`; 0 = full LEDs!), DEQ3 (bands `2048 (g-1)`),
       C5-CMB (band GR `dB*2048/6`), LIMITER/*MASTER*/DE-S2 (GR, scale [H] `GR*2048/6`).
  - **Changes.**
    - `crates/wing-blocks/src/meter.rs` (new): `DynMeterLaw` / `FxMeterLaw` per model id (table + sources in
      the module doc); `Model::meter_bands` (signed band gains) implemented by DEQ/DEQ2 (`dyn_eq.rs`), DEQ3,
      C5-CMB, channel-strip FX (per dynamics stage).
    - `crates/wing-engine/src/meters.rs`: LevelBank gains band cells (per strip, latest value, `LevelBank::set`)
      and FX cells (per slot: in/out peaks, GR, 6 bands); `FxMeter`, `StripMeter::slot_bands`, `is_value_cell`.
      `RtModel::meter_bands` / `ModelSlot::meter_bands` (`params/strip.rs`, `std_models.rs`).
    - `fx.rs`: `FxSlot` meters input/output/GR/bands when its owner runs it; `FxRack::attach_meters`, called by
      `mix/build.rs` when `MixConfig::meters` is set. `channel.rs` / `mix/strip.rs`: one line each publishing
      the slot bands next to the existing GR write (meter path only).
    - `apps/wing-sim/src/meters.rs`: `RealMeters::with_store` resolves each strip's gate/dyn model and each FX
      slot's model from the store (as WING-Edit decodes by its own `mdl`) and applies the laws; FX blocks filled.
    - `wing-proto`: `describe_selections` + richer meter-definition log line. `docs/research/protocol.md` §4.2 note.
  - **Tests.** `wing-blocks/tests/meter_laws.rs`: laws vs `dyn-models.json` (30 models), every DYN model moves
    its GR word (WAVE/WARM constant by design), every FX model with state words moves them.
    `wing-engine/tests/strip_model_meters.rs`: every model of `ch/*/gate` and `ch/*/dyn` drives its GR/band cells
    in a ChannelStrip. `fx::tests::slot_meters_levels_and_gr`. `wing-sim` `per_model_gr_laws_and_fx_blocks`.
    `fx_alloc` now runs with meters attached (zero RT alloc). `make check` passes.
  - **Not populated (documented in `wing_blocks::meter`):** PCORR, SPKMAN, SUB-M, V-IMG state words (need
    pitch/level/correlation telemetry); FX state scales for LIMITER/*MASTER*/DE-S2 are [H].
  - **Proposed follow-ups:** verify the [H] FX GR scale and the DEQ word against hardware or a WING-Edit capture
    with the new log line; expose pitch/level/correlation telemetry for PCORR/SUB-M/V-IMG/SPKMAN pages.
