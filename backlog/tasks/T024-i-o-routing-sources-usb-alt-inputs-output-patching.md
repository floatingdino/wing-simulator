---
id: T024
title: I/O routing: sources, USB alt inputs, output patching
stage: S2
status: done
owner: agent-T024
depends: [T010, T022]
branch: task/T024
---

## Goal
Implement WING io model from snapshot (io/in, io/out, altsw, channel main/alt sources) so USB inputs feed channels exactly as the scene defines, and output patches (incl. USB out 1-48) take the right signals.

## Acceptance criteria
- [x] Scene routing tests from PUBLIC.snap
- [x] altsw toggles main/alt sources

## Log
- 2026-10-25 coordinator: created.
- 2026-10-25 agent-T024: started. Plan: `crates/wing-engine/src/routing/` (source address space, tap enum,
  table builder from ParamStore, lock-free table swap, oscillator + input/output router processors); tests from
  fixtures/PUBLIC.snap via a test-only JSON loader (T011 not on main yet).
- 2026-10-25 agent-T024: done, status review.
  Files: `crates/wing-engine/src/routing/{mod,addr,table,swap,nodes}.rs` (module doc = the model: which groups
  are live), `src/lib.rs` (`pub mod routing`), `Cargo.toml` (wing-engine -> wing-model dep; serde_json
  workspace dev-dep, already in Cargo.lock via criterion), `benches/routing.rs`, `tests/common/mod.rs`
  (test-only snap loader for io + ch/aux in), `tests/routing_snap.rs`, `tests/routing_alloc.rs`.
  Design: `SrcGroup`/`SourceAddr` = all 18 schema source groups as 540 mono legs (USB live = driver ext
  inputs; OSC live = internal `Oscillators`; USR 1-24 strip taps, USR 25-56 source aliases, BUS/MAIN/MTX/SEND/MON
  mix legs -> `Tap`/`TapId` (396 placeholder taps, engine supplies `Port`s via `TapPorts` once T036 exists);
  everything else silent but routable). `RoutingBuilder` resolves param ids once and builds an immutable
  `RoutingTable` (48 input strips: main/alt/use_alt/width/legs with gain incl. preamp g, mute, pol; 374 outputs
  of all 11 output groups; osc params). Alt = `altsrc || (srcauto && io/altsw)`. ST/M-S pairs are odd-aligned
  (either leg selects the pair) [I]; mix-output channel sources always stereo [I]. `RoutingControl` = builder +
  store hook (dirty flag on watched params) + `TableSender`; RT readers (`TableReader`) adopt new tables via
  two atomic pointer slots without alloc/free. Processors: `Oscillators` (sine/pink/white, lvl = dBFS RMS [I]),
  `InputRouter` (USB+osc -> 48 stereo strips, M/S decode), `UsbOutRouter` (-> USB Out 1-48, taps, L+R = 0.5*(L+R)).
  Tests: `make check` green. routing_snap: altsw off/on channel->USB maps, per-channel altsrc override, all 48
  USB outs, other output groups, signals through a 2-worker engine with live altsw/pol/mute toggles via store hook,
  oscillator RMS. routing_alloc: counting allocator (thread-local arm) proves zero heap ops on the RT thread over
  >=20k cycles while a control thread publishes >=500 tables. Bench: routing/cycle_64 ~0.9 us (serial, M-series).
  Proposed follow-ups: (1) T036: wire `TapPorts` and a one-block-delayed re-input for BUS/MAIN/MTX/USR channel
  sources (InputRouter currently reads taps as silence); (2) replace the test-only loader with T011's loader;
  (3) `io/autoaltovr` semantics and `ch/*/in/set/$mode|$g|$vph` mirrors not modelled; (4) route-change
  crossfade (currently hard switch); (5) verify [I] items on a console (pair alignment, L+R scaling, osc level).
