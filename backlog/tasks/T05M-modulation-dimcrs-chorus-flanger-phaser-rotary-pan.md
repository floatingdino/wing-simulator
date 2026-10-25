---
id: T05M
title: Modulation: DIMCRS, CHORUS, FLANGER, PHASER, ROTARY, PANNER, MOOD
stage: S5
status: done
owner: engineer-T05M
depends: [T050]
branch: task/T05M
---

## Goal
Per models.md §6; tempo sync.

## Acceptance criteria
- [x] Behaviour tests

## Log
- 2026-10-25 coordinator: created by decomposing epics T040-T044/T051 after T004.
- 2026-10-25 engineer-T05M: implemented all 7 models in `crates/wing-blocks/src/models/fx_mod.rs` (DimensionChorus,
  ModDelay::{chorus,flanger}, Phaser, Rotary, Panner, MoodFilter; shared Lfo / wave / AhrEnvelope), every schema
  parameter mapped at its schema slot with WING-Edit defaults (test cross-checks data/schema/wing-tree.json).
  Registered in model.rs (`fx_implemented` list), `pub mod fx_mod`, `[[bench]] fx_mod`.
  - Tempo sync: the schema has no tempo/sync/note keys for these models (rates are Hz), so LFOs free-run.
  - Character: DIMCRS = 2 BBDs (5 ms) swept in antiphase by a triangle, 10 kHz BBD filters + soft-saturating
    buckets, SDD-320 matrix L = dry + g(A-B), R = dry + g(B-A) (wet cancels in mono); buttons 1-3 0.25 Hz rising
    depth, 4 = 0.5 Hz deepest, combos sum depth/max rate, none = bypass. ROTARY = 800 Hz crossover, horn/drum
    rotors with asymmetric inertia (HORN/DRUM ACCEL), Doppler (15 cm horn), AM, horn beaming LP, 2 opposed mics,
    distance -> AM depth/width, cabinet colour. PHASER = 2..12 first-order allpasses, feedback reso, env mod,
    stage changes cross-faded between taps. MOOD = TPT SVF 12/24 dB, LP/HP/BP/NOTCH cross-faded, drive,
    AHR envelope + 7 LFO waves (RND = S&H). Inferred mappings are marked [I] in the module doc.
  - Tests (`models::fx_mod::tests`, 18): LFO rate exactness (chorus/phaser/panner/mood/rotary AM period),
    modulation depth (chorus/flanger delay swing, phaser/mood cutoff sweep, panner depth, env>speed/depth),
    flanger comb notch/feedback peak, phaser notch count = stages/2 + reso, rotary spin-up/down + Doppler depth,
    DIMCRS mono compatibility, MOOD slopes/types, extreme-setting stability + silent tails, no clicks on abrupt
    param changes (verified the check fails when cross-fades are disabled), schema defaults; zero-alloc in
    `crates/wing-blocks/tests/fx_mod_no_alloc.rs`.
  - Bench `cargo bench -p wing-blocks --bench fx_mod` (64-sample stereo, M-series): DIMCRS 0.89 us, CHORUS 1.34,
    FLANGER 1.51, PHASER(12 stg) 1.62, ROTARY 1.68, PANNER 0.36, MOOD(24 dB, drive) 1.23 us.
  - `make check` passes.
  - Proposed follow-ups: tune [I] mappings (DIMCRS depths, PHASER range axis, MOOD env/LFO octaves, ROTARY
    distance) against console recordings once available.
