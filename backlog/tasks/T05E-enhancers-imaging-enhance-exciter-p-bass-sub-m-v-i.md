---
id: T05E
title: Enhancers/imaging: ENHANCE, EXCITER, P-BASS, SUB-M, V-IMG, TAPE
stage: S5
status: review
owner: claude-t05e
depends: [T050]
branch: task/T05E
---

## Goal
Per models.md §6.

## Acceptance criteria
- [x] Behaviour tests (harmonic content, aliasing, imager mono compatibility, tape character, zero alloc, bench)

## Log
- 2026-10-25 coordinator: created by decomposing epics T040-T044/T051 after T004.
- 2026-10-25 claude-t05e: implemented all six models in `crates/wing-blocks/src/models/fx_enhance.rs` (all WING params, schema slot order, defaults per schema): `UltraEnhancer` (ENHANCE), `Exciter`, `PsychoBass` (P-BASS), `SubMonster` (SUB-M), `VelvetImager` (V-IMG), `TapeMachine` (TAPE, plus non-schema `TapeExtras`: wow/flutter/hiss, default off). Registered in model.rs (+ implemented list in its test), `pub mod fx_enhance;` in models/mod.rs, `[[bench]] fx_enhance`.
  - Latency: ENHANCE / EXCITER / TAPE = **46 samples** (4x oversampler 45.5 rounded up, dry aligned); P-BASS, SUB-M, V-IMG = 0. Consequence: every FX slot hosts these models, so `FxRack::slot_latency` is now 46 for all slots (ModelSlot latency = max over hosted models, T03E). `crates/wing-engine/tests/fx.rs` updated for that (strip latency with an engaged insert; the switch-off crossfade runs at the slot output, i.e. after the strip pad).
  - Bench (64-sample stereo blocks, M-series, release): ENHANCE 9.0 us, EXCITER 9.7 us, P-BASS 1.1 us, SUB-M 1.0 us, V-IMG 0.45 us, TAPE 8.0 us (budget 1333 us).
  - Aliasing (0.8 sine, worst non-harmonic line): ENHANCE all bands 100 % -122 dBc @9.7k, EXCITER defaults 100 % wet -87 dBc, TAPE +12 dB drive -89 dBc; all < -80 dBc (tested at 9.7 kHz and 5.9 kHz). Known limit: driven into a hard clip (EXCITER HARM/PEAK/ZFILL 100 %, TAPE +25 dB) aliasing is -70 / -62 dBc at 9.7 kHz, ~-62 dBc at 13 kHz carriers (4x + ADAA1; 2x measured 10 dB worse). Follow-up idea: ADAA2 or 8x for those two models.
  - V-IMG is mono compatible by construction (all effects live in S, `L+R = 2 g M`), tested over width/stereo/mode/deep.
  - Inferred mappings (BASS/HIGH FREQ and MID Q maps, LMF SPRD, SUB-M band layout, TAPE bump/loss numbers, ...) listed in docs/VERIFY-ON-HARDWARE.md (rows tagged T05E).
  - Files: crates/wing-blocks/src/models/{fx_enhance.rs,mod.rs}, src/model.rs, Cargo.toml, benches/fx_enhance.rs, tests/fx_enhance_no_alloc.rs, crates/wing-engine/tests/fx.rs, docs/VERIFY-ON-HARDWARE.md.
  - Test: `cargo test -p wing-blocks fx_enhance`, `cargo test -p wing-blocks --test fx_enhance_no_alloc`, `cargo bench -p wing-blocks --bench fx_enhance`, `make check` (passes).
  - Proposed follow-ups: ADAA2/8x for hard-clip aliasing; confirm on hardware the items in VERIFY-ON-HARDWARE; TAPE hysteresis memory (Chowdhury-style) if the simple saturator is judged too static.
