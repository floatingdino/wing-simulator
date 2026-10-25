---
id: T080
title: BUG: dynamics/gate sidechain key source selection has no effect
stage: S4
status: review
owner: T080-agent
depends: []
branch: task/T080
---

## Goal
User: "dynamics sidechain/key source handling doesn't seem to affect anything". Implement WING key-source routing end to end for gate and dyn slots on channels/aux/bus/main/mtx: key source selection (self / other channel / bus / any source per schema keys, e.g. ch/N/gate/... and ch/N/dyn/... sc source, `dynsc`, key filter, key listen), cross-strip keys (needs a graph dependency or a one-block-late tap — choose; document latency), key filters applied per model, external keys reaching every model (incl. alternate models that currently ignore the key, e.g. 3CMP). Regression tests: a ducker on ch1 keyed from ch2 ducks; compressor keyed from a bus reacts; key listen solos the key.

## Acceptance criteria
- [x] Key source selection works for every source type in the schema, all models
- [x] Tests; zero RT alloc

## Log
- 2026-10-26 coordinator: created from user report.
- 2026-10-26 T080-agent: **Root cause** (two independent defects + one model bug):
  1. The mix graph never fed external keys: `InputStripNode::process` always passed `StripKeys::default()` to
     `ChannelStrip::process_with` (the strip's own "external" flag from `src` gated a key that never arrived), so
     `ch/N/gatesc/src` / `dynsc/src` changes were ignored and the slot kept keying from itself.
  2. `MixStrip` (bus/main/mtx) dropped `dynsc/src` outright (`_ => return, // 3 = src: external keys are not
     resolved yet`) and always called `dyn_.process_keyed(l, r, None)`; `tap` and `$solo` were not bound anywhere.
  3. `3CMP` ignored its `sidechain` argument (`_sidechain`).
  **Evidence**: user log `scratch/wing-sim-v0.4.log` lines 2383-2408: WING-Edit sets `bus/7/dynsc/src = BUS.5`,
  `bus/7/dynsc/tap = POST FDR` (+ `bus/4/dynsc/tap`); all accepted by the store, no audible effect. Failing tests
  written first (`crates/wing-engine/tests/sidechain.rs`, all 6 failed before the fix: e.g. "ch2 key should duck
  ch1 by ~20 dB: alone -51.6 dB, with ch2 -51.9 dB", "bus 5 key should compress bus 7: alone -40.7, keyed -40.6";
  `crates/wing-blocks/tests/key_all_models.rs` failed on 3CMP: "level diff 0.000 dB").
  **Fix / design** (choice: one-block-late tap, not graph edges): new `crates/wing-engine/src/mix/keys.rs`:
  `KeyBank` (per strip x 10 key slots, double-buffered by cycle parity, `AtomicU32` sample bits, demand
  refcounts so only held taps are written), `KeyIn` (consumer route + scratch), `KeyOut` (source taps),
  `key_route` (schema `src`/`tap` enum -> strip + slot; enum tables verified against wing-tree.json).
  Every external key is exactly **one engine block late** (64 samples = 1.33 ms @ 48 kHz) regardless of
  strip order; cycles (ch keyed by a bus it feeds) are legal; graph stays static. SELF keys unchanged (no
  latency). Channels publish IN/FILT/3/4/5/PFL (strip) + AFL/POST (node); bus/main/mtx publish BUS(IN)/INS1/EQ/
  DYN/PFL (hook) + AFL (node, post fader+pan before output delay [I]); aux taps for the bus-style enum: BUS->IN,
  DYN->TAP5, EQ->TAP4, INS2->PFL [I]. `SC_LAYOUT` now binds `type f q src tap $solo` (`SideChain` struct);
  the key filter (ModelSlot) applies to external keys for every model. Key listen: `gatesc/$solo`,
  `dynsc/$solo` -> strip publishes the slot's detected key (filtered external/own key) to a listen slot;
  `MixLogic` watches them and sets new flags `KEY_GATE`/`KEY_DYN` + solo-monitor bits (counts for
  `$stat/solo`, no `$sololed`, no SIP muting [I]); the monitor plays the key (same cycle, unity gain, no
  pfldim) instead of PFL. 3CMP splits the external key through a second LR4 crossover so each band detects on
  its key band. WARM is the only key-less dynamics model (a preamp, no detector) - documented exemption.
  **Files**: crates/wing-engine/src/mix/{keys.rs (new), mod.rs, nodes.rs, strip.rs, build.rs, logic.rs,
  monitor.rs}, crates/wing-engine/src/channel.rs, crates/wing-blocks/src/models/comp_multi.rs,
  crates/wing-engine/tests/{sidechain.rs (new), mix_alloc.rs (key src/tap/filter/$solo churn)},
  crates/wing-blocks/tests/key_all_models.rs (new), docs/research/signal-flow.md.
  **Tests**: `cargo test -p wing-engine --test sidechain` (ducker ch1<-ch2; key filter + tap IN/AFL on
  channels; dyn slot ch<-ch; bus7 COMP<-BUS.5 AFL/BUS taps + SELF; main/mtx/aux <- bus; key listen gate/dyn,
  filtered, `$stat/solo`), `cargo test -p wing-blocks --test key_all_models` (every implemented dyn model
  reacts to an external key), keys.rs unit tests (schema enums, one-block latency, demand/release), mix_alloc
  (zero RT alloc incl. key routing changes). `make check` and `make check-full` pass.
  **Proposed follow-ups**: verify on hardware whether WING keys are latency-free (graph reordering pass for
  the acyclic cases would remove the 1-block offset); key listen interaction with exclusive solo and
  `$sololed`; exact aux/bus `tap` semantics (DYN = post-dyn?, INS2 vs PFL) and AFL pre/post pan on busses.
- 2026-10-26 T080-agent: merged main (T082 per-model GR/key metering + band meter cells, T083 additive/echo
  solo). One textual conflict (channel.rs imports: kept both `band` and `KeyOut`). Bus/main/mtx DYN_KEY meter
  uses the routed external key, alongside T082's `slot_bands`. Key-listen flags sit beside T083's additive
  solo in `MixLogic::refresh` (counts toward `$stat/solo`, not `$sololed`/exclusive). `make check` passes
  (incl. tests/sidechain.rs key listen).
