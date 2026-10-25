---
id: T050
title: FX rack framework (16 slots, insert/send, stereo)
stage: S5
status: review
owner: claude-T050-T044
depends: [T036, T03A]
branch: task/T050-T044
---

## Goal
FX slot hosting, routing as insert or via sends, mix/bypass, tempo/tap, preallocated model instances.

## Acceptance criteria
- [x] Slot routing tests (insert vs bus-insert "send" use, FX return to main and aux re-input, SEND legs on the output patch, one insert per slot, lite-slot restrictions, latency, no-alloc swaps)

## Log
- 2026-10-25 coordinator: created.
- 2026-10-25 claude-T050-T044: done together with T044 on branch task/T050-T044.
  - Design (crates/wing-engine/src/fx.rs module doc): 16 `FxSlot`s, each a `ModelSlot` over its schema group
    (`fx/1` for FX1-8: 64 models; `fx/9` for FX9-16: 38 lite models, no reverb/mod/delay/pitch) with one
    preallocated instance per model (`NONE` = pass-through), `fxmix` = wrapper wet/dry, swaps crossfade, no
    alloc. Schema has no FX source group and no global tempo/tap: FX are inserts only; a "send FX" is an
    FX on a bus insert, returned via the bus (mains, bus->bus, or re-input `BUS.n` on a ch/aux). The `SEND`
    output group (32 legs) carries each slot's input while it runs.
  - Scheduling: insert FX run inline in the owning strip's node (no strip splitting, no cycles: a slot has
    exactly one owner per cycle). Shared slot state behind `UnsafeCell` with the phase-discipline safety
    argument; `FxRackNode` after the matrices takes `fx/*` + `cfg/amix` params, outputs SEND legs and sums
    automix levels. Ownership: lowest insert id among claiming inserts (order-independent) [I].
  - Proof model: `ST-DL` (crates/wing-blocks/src/models/fx_delay.rs): wet-only stereo delay, time x factor x
    L:R pattern + offset, ST/X(ping-pong)/M, feedback HP1/LP1, output LC/HC, 50 ms time glide. All other FX
    stay stubs (owners now T05A..T05V in `owning_task`).
  - Files: crates/wing-engine/src/fx.rs (new), channel.rs (InsertHost: process_insert_at/assign/
    automix_group/automix/insert_latency, InsertPos, sync_inserts, latency_with), mix/{build,mod,nodes,strip}.rs
    (rack wiring, SEND taps, MixConfig::fx_rack, MixGraph::fx, MixNodes::fx/send_port, StripHook
    set_insert_host/latency), params/strip.rs (ModelSlot enum table 128 for the 65-entry FX enum),
    crates/wing-blocks/src/{model.rs,models/fx_delay.rs,models/mod.rs}, benches wing-blocks/fx_delay.rs +
    wing-engine/fx.rs, tests wing-engine/tests/{fx.rs,fx_alloc.rs}, tests/common/rig.rs (Rig::fx).
  - Test: `make check` (passes). `cargo test -p wing-engine --test fx --test fx_alloc`,
    `cargo bench -p wing-engine --bench fx`: 16 stub slots 0.24 us, 8 ST-DL + 8 stubs 5.1 us per 64-sample
    block (0.4 % of a 1.33 ms cycle); `cargo bench -p wing-blocks --bench fx_delay`: ST-DL 0.61 us/block.
  - Proposed follow-ups: write `$stat` / `$a_chn` / `$a_pos` back to the store (API exists:
    `FxRack::insert_state/owner`); EXT return `egrp/ein` + `lat` (T05C); insert on/off is a hard switch
    (no crossfade); tap tempo for delays has no schema key (T05D to decide); verify on a console: owner rule
    for double assignment, ST-DL pattern/offset mapping, SEND group meaning.
