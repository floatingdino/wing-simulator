---
id: T030
title: wing-dsp primitives
stage: S3
status: in_progress
owner: agent-T030
depends: [T001, T007]
branch: task/T030
---

## Goal
Biquad/SVF (TPT) with coefficient smoothing, dB/gain tables, parameter smoothing, envelope/peak/RMS detectors, delay lines, oversampling (2x/4x polyphase), SIMD-friendly channel layout, denormal handling.

## Acceptance criteria
- [ ] Unit tests vs analytic responses
- [ ] Criterion benches

## Log
- 2026-10-25 coordinator: created.
- 2026-10-25 agent-T030: started on branch task/T030. Implemented modules gain, smooth, biquad (RBJ + BW/LR
  cascades, Biquad/BiquadF64/SmoothedBiquad/Cascade), bank (SoA 8-lane BiquadBank), svf (TPT), detector,
  delay, oversample (2x/4x half-band polyphase), denormal (FPCR.FZ / MXCSR FTZ+DAZ guard). 34 unit tests pass.
  Next: zero-alloc integration test, criterion benches, `make check`.
