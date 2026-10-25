---
id: T037
title: Bus/main/matrix strip processing
stage: S3
status: done
owner: engineer-T037
depends: [T036]
branch: task/T037
---

## Goal
Strip processing on busses/mains/matrices (EQ, dyn, inserts, delay) per signal-flow.md.

## Acceptance criteria
- [x] Tests per strip type

## Log
- 2026-10-25 coordinator: created.
- 2026-10-25 engineer-T037: started (worktree .claude/worktrees/t037). Plan: `mix/strip.rs` MixStrip (StripHook) =
  trim/bal/inv -> Ins1 -> EQ slot (`<bus|main|mtx>/*/eq`, STD bus + alt + PIA) -> DYN slot (+dynsc filter, dynxo)
  -> Ins2; output delay (`dly`) via new `StripHook::process_post` on POST; bound at blocks >= HOOK_BASE.
- 2026-10-25 engineer-T037: done, status review (merged main incl. T039 meters).
  * `crates/wing-engine/src/mix/strip.rs` (new): `MixStrip` StripHook for bus/main/mtx = trim/bal/inv (5 ms ramp,
    skipped at unity) -> Ins1 -> EQ slot (`<kind>/*/eq`: STD bus variant w/ exact coefs + registry models; group
    gives PIA not MACH4; mix 0..125 % scales band gains) -> DYN slot (`<kind>/*/dyn`, all 32 models, dynsc
    type/f/q key filter, dynxo) -> Ins2 (InsertHost placeholder) ; output delay `dly` (on + mode M/FT/MS/SMP +
    dly, wing-blocks InputStage delay) on POST via new `StripHook::process_post`, so PRE/PFL are undelayed.
    Fixed order: the schema has no `proc` for bus/main/mtx. `bind_mix_strip` binds hook blocks >= HOOK_BASE and
    registers telemetry (TAP_PEAK per MixTap IN/INS1/EQ/DYN/PFL/POST, GR). T039 hook filled: StripMeter writes
    IN (after trim), DYN_KEY, DYN_GR; `PortMeters::for_mix` now writes only PRE from PFL for mix strips.
  * `mix/nodes.rs` (process_post call), `mix/mod.rs` (`hooks: None` = standard MixStrip; `no_processing()`),
    `mix/build.rs` (default hook + meters + bindings), `meters.rs`, `channel.rs` (`slot_key_peak` pub(crate)).
  * Tests: `tests/mix_strip.rs` (EQ and EQ+COMP bit-exact vs standalone models on bus/main/mtx; order/bypass via
    insert mock + taps; balance/polarity; delay accuracy SMP/MS/M/FT/500 ms; schema->graph bus EQ+COMP vs
    standalone; delay POST-only through graph; level-bank meters), `tests/mix_alloc.rs` extended (bus/main/mtx
    params, EQ/dyn model swaps, dly mode, zero RT alloc). `tests/mix.rs::public_snap_main_mix` uses
    `no_processing()` (it checks mix laws; the scene's bus EQ/dyn would change the gains).
  * Bench (`make bench`-style, `cargo bench -p wing-engine --bench mix`, M-series): public_busproc (PUBLIC.snap,
    all 28 bus/main/mtx strips with trim+STD EQ 8 bands+COMP+delay) 39.5 us with 7 workers = 3.0 % of 1333 us
    (serial 122 us = 9.1 %); public (scene processing) 29.5 us; one MixStrip all-on 64-sample block 2.5 us.
  * `make check` passes.
  * Proposed follow-ups: external `dynsc.src`/`tap` keys for bus/main/mtx (graph wiring, like channels);
    matrix direct inputs `dir`; FX rack behind InsertHost (T050); apps/wing-sim doc comments (meters.rs table,
    audio/console.rs) still say bus processing/GR is pending T037; ModelSlot runs an off model for one
    wrapper ramp after a parameter change (GR is reported 0 while off here; channel meters can show a blip).
