---
id: T05S
title: Special reverbs: GATED, REVERSE, DEL/REV, SHIMMER
stage: S5
status: done
owner: engineer-T05S
depends: [T05R, T05D]
branch: task/T05S
---

## Goal
Per models.md §6.

## Acceptance criteria
- [x] Behaviour tests (gate timing, reverse envelope, shimmer partials, DEL/REV routing, stability, zero alloc)

## Log
- 2026-10-25 coordinator: created by decomposing epics T040-T044/T051 after T004.
- 2026-10-25 T05S: GATED, REVERSE, DEL/REV, SHIMMER in crates/wing-blocks/src/models/fx_reverb/special.rs on the T05R 8-line FDN core; all WING params mapped (table in module doc, [I] = inferred). Registered in model.rs (+ fx_implemented list).
  - GATED: own pre-delay -> peak trigger (-40 dBFS edge) -> hold = DCY -> gate (att fade-in, 4 ms close) over dense FDN (T60 6x DCY); dens/diff/sprd/lc/hsf/hsg mapped.
  - REVERSE: FDN wet captured in double buffer windows W = DCY, played back reversed (RISE = fade-in fraction, 3 ms cut). Wet latency = PRE DLY + W (0.14..1 s), `latency()` reports 0 (wet-only; documented, `Reverse::wet_latency_samples`).
  - DEL/REV: stereo delay (glided TIME, FEED<=99.5 %, FHC one-pole in loop) parallel to FDN; DLY LVL to out, DLY->REV and IN->REV sends.
  - SHIMMER: local 2-tap grain shifter (+12 / +7, W = 4096 smp, taps read >= 64 smp behind write head) in the outer feedback loop, run in 32-sample sub-blocks around the core; SHIM = loop gain (<=0.5, tanh-limited, 120 Hz HP), SHINE = +7 mix and LP 4-12 kHz. Grain shifter splits a pure tone into +-fs/W sidebands (no coherent carrier), so tests measure per band.
  - Files: special.rs, special_tests.rs, fx_reverb/mod.rs (pub mod + test mod), model.rs, tests/fx_reverb_special_no_alloc.rs, benches/fx_reverb_special.rs, Cargo.toml bench block.
  - Bench (M1, stereo 64 smp): GATED 5.2 us, REVERSE 5.0 us, DEL/REV 4.7 us, SHIMMER 7.0 us. Test: `cargo test -p wing-blocks special`.
  - Follow-ups: calibrate trigger threshold / gate release and shimmer feedback against console measurements; wire pitch +12/+7 selection if WING exposes it.
