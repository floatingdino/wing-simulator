---
id: T05C
title: Channel-strip FX: *EVEN*, *SOUL*, *VINTAGE*, *BUS*, *MASTER* + EXT
stage: S5
status: done
owner: agent-T05C
depends: [T050, T04G, T04C]
branch: task/T05C
---

## Goal
Compose existing blocks per models.md §6; EXT = external send/return routing.

## Acceptance criteria
- [x] Tests per model

## Log
- 2026-10-25 coordinator: created by decomposing epics T040-T044/T051 after T004.
- 2026-10-25 T05C agent: done, status review. Files: crates/wing-blocks/src/models/fx_strip.rs (+ mod.rs, model.rs
  registration, Cargo.toml bench), benches/fx_strip.rs, tests/fx_strip.rs, tests/fx_strip_no_alloc.rs;
  crates/wing-engine: routing/table.rs (EXT routes `RoutingTable::ext`), routing/nodes.rs (router +32 EXT outputs,
  `InputRouter::ext_port`, `ROUTER_OUTPUTS`), routing/mod.rs, fx.rs (`FxRackNode::with_returns`, slot return capture,
  keyed insert), mix/build.rs (wiring), tests/fx_ext.rs, tests/fx_ext_alloc.rs, tests/fx.rs (2 tests adapted, see below).
  * Strips are `create_model` compositions: *EVEN* E88 gate -> E88 EQ -> ECL33; *SOUL* 9000G -> SOUL EQ -> 9000C;
    *VINTAGE* 76LA -> PULSAR -> LA; *BUS* WARM -> E84 -> SBUS (d_gain = make-up after the comp); *MASTER* TAPE -> MACH4 ->
    ENHANCE -> LIMITER. Every FX slot maps 1:1 to a sub-block slot (route tables, cross-checked against wing-tree.json
    for type/range/defaults in tests/fx_strip.rs). Stage `*_on` switches crossfade 5 ms; settled = bare block (bit-exact
    vs manual chain, tested per strip). EQ `mix` = `set_gain_scale`.
  * EXT: model passes the side-chain key (= return) times trim; egrp OFF = pass-through [I]; emode M/ST/M-S. Engine: the
    routing table resolves `fx/N/{mdl,1,2,3}` (egrp/ein/emode) into an EXT route per slot, the input router outputs the
    return legs, `FxRackNode` captures them at cycle end, the owning strip hands them to the slot as key next cycle
    (one block late, like re-input taps). Only USB In / OSC are live sources in the sim (others silent, as for channels).
  * `lat` is stored but not reported as latency (slot latency = max over models, T03E; default 45 would delay every slot).
  * MASTER depends on T05E (TAPE, ENHANCE) and T05Q (LIMITER), which are not merged: it composes them through
    `create_model` so it picks them up automatically, until then those stages are the catalog's pass-through stubs
    (the bit-exact test compares against the same catalog blocks). MASTER slot 7 (`40`) is absent from the schema
    export; routed to MACH4 40 Hz.
  * Side effect: *VINTAGE* (76LA, 57 samples) and *BUS* (WARM, 39) make the FX slot latency (max over models) 57 for
    every slot (T03E design), so tests/fx.rs `channel_insert_runs_slot_with_fxmix` / `bus_insert_send_fx_...` now add
    `r.fx.slot_latency(n)` to the expected delay (and the switch-off crossfade there is evaluated at the slot output).
  * Tests: `cargo test -p wing-blocks --test fx_strip --test fx_strip_no_alloc`, `cargo test -p wing-engine --test fx_ext
    --test fx_ext_alloc --test fx`; bench `cargo bench -p wing-blocks --bench fx_strip`. `make check`: see below.
  * Proposed follow-ups: swap in real LIMITER/TAPE/ENHANCE results once T05E/T05Q merge (add MASTER to a hardware-verify
    list: stage order, ENHANCE g/solo, TAPE out); EXT `lat` compensation of the dry path against the measured loop.
