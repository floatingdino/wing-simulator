---
id: T036
title: Pan, fader, mute, sends, busses, mains, matrices
stage: S3
status: done
owner: engineer-T036
depends: [T035]
branch: task/T036
---

## Goal
Pan laws/width, fader taper, mutes, 16 bus sends (pre/post taps, sub-modes), 4 mains, 8 matrices, aux channels, summing.

## Acceptance criteria
- [x] Routing/gain tests from PUBLIC.snap

## Log
- 2026-10-25 coordinator: created.
- 2026-10-25 engineer-T036: done, status review.
  - Files: `crates/wing-engine/src/mix/{mod,law,nodes,build}.rs` (new), `src/routing/feedback.rs` (new,
    one-block re-input taps), `src/routing/{mod,nodes}.rs` (InputRouter reads feedback taps,
    `add_input_stage_with_feedback`), `src/lib.rs`, `tests/mix.rs`, `tests/mix_alloc.rs`,
    `tests/common/rig.rs`, `benches/mix.rs`; daemon hook: `apps/wing-sim/src/audio/console.rs` (provisional
    main-1 sum replaced by `build_mix_graph`; host-bridge taps main/bus/matrix 1..n), `apps/wing-sim/tests/audio.rs`
    (expectations: -3 dB centre pan, mono USB source, 60-block mute settle).
  - Design: `build_mix_graph(&ParamStore, &mut RoutingControl, MixConfig)` -> GraphBuilder + ParamMapBuilder
    (every ch/aux strip, fader, send, bus/main/mtx param bound) + MixNodes + tap ports. Levels: osc, router,
    48 input strips, bus tiers, 4 mains, 8 mtx, feedback writer + USB-out router. Per-bus/main/mtx summing
    nodes; send params bound to the *receiving* node (block = 1 + source index). StripHook trait = T037 slot
    (blocks >= 128). Gains are 2x2 matrices ramped by a 5 ms one-pole per block + linear interpolation.
  - Semantics [I] (documented in mix/mod.rs): constant-power -3 dB pan for ch/aux and send pans, balance
    (0 dB centre) for bus/main/mtx; width = M/S scaling; busmono after pan; mute also kills PRE sends;
    GRP = POST at unity (PUBLIC.snap assigns drums to their group at -oo); plink>0 = follow channel pan;
    `tapwid` on the PRE tap; `busmode`/`ind` are resolved into per-send `mode` by wing-model migration;
    bus->bus only to higher-numbered busses, tiers from the scene's active sends at build time
    (`BusOrder::{FromScene,Chain,Flat}`). Strip `in/set/$mode`/`$g` pinned to ST/0 dB (router owns them;
    forwarding mono-summed stereo sources).
  - Tests: law unit tests, `tests/mix.rs` (laws, mute/ext mute/mute safe, send taps/modes/plink/ptap, bus->bus
    tiers, mains, matrices, busmono, smoothing, USB-out taps, re-input latency, PUBLIC.snap main-1 gains
    for ch26/ch21/ch1 + muted ch23 hand-computed, parallel == serial bit-exact), `tests/mix_alloc.rs`
    (zero RT heap ops with live param changes). `make check` passes.
  - Bench (`cargo bench -p wing-engine --bench mix`, M-series, 64 smp): PUBLIC scene 7 workers ~24-32 us
    (~2 % of 1333 us), serial 63 us (4.7 %); dense all-sends 7 workers ~25 us, serial 33 us.
  - Proposed follow-ups: matrix direct inputs (`mtx/*/dir`, AES/MON sources); `pon` and `plink`=2
    semantics; `cfg/mainlink`; verify pan/width laws on hardware (T03C/T072); live bus->bus send changes
    that violate the built tiers are ignored until the graph is rebuilt (daemon could rebuild on scene load);
    side-chain key routing from other strips; per-strip telemetry for mix nodes (T039).

