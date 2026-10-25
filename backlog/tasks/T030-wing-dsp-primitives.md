---
id: T030
title: wing-dsp primitives
stage: S3
status: review
owner: agent-T030
depends: [T001, T007]
branch: task/T030
---

## Goal
Biquad/SVF (TPT) with coefficient smoothing, dB/gain tables, parameter smoothing, envelope/peak/RMS detectors, delay lines, oversampling (2x/4x polyphase), SIMD-friendly channel layout, denormal handling.

## Acceptance criteria
- [x] Unit tests vs analytic responses
- [x] Criterion benches

## Log
- 2026-10-25 coordinator: created.
- 2026-10-25 agent-T030: started on branch task/T030. Implemented modules gain, smooth, biquad (RBJ + BW/LR
  cascades, Biquad/BiquadF64/SmoothedBiquad/Cascade), bank (SoA 8-lane BiquadBank), svf (TPT), detector,
  delay, oversample (2x/4x half-band polyphase), denormal (FPCR.FZ / MXCSR FTZ+DAZ guard). 34 unit tests pass.
  Next: zero-alloc integration test, criterion benches, `make check`.
- 2026-10-25 agent-T030: **ready for review.** No new dependencies (all primitives hand-written; criterion
  remains dev-only).
  Files: crates/wing-dsp/src/{lib,gain,smooth,biquad,bank,svf,detector,delay,oversample,denormal}.rs,
  crates/wing-dsp/tests/no_alloc.rs, crates/wing-dsp/benches/dsp.rs.
  How to test: `make check` (passes); `make bench`. 35 tests: RBJ LP/HP/BP/notch/AP/peak/shelves + 1st-order
  designs vs analog prototypes at warped freq (mag < 0.01 dB, phase < 1e-6 rad), Butterworth 1-8 / LR2-8 vs
  analytic, LR4/LR8 sum flat, RBJ BW-Q (-3 dB edges within 0.002 oct) and shelf-slope Q, runtime
  Biquad/BiquadF64/Cascade impulse-response DTFT vs design, TPT SVF (8 modes) vs RBJ responses, smoother 63%
  at tau + linear ramp exactness, detector attack/release 63%/36.8% at tau, program-dependent release, RMS
  window exact, delay interpolation error bounds (linear w^2/8, Lagrange3 9w^4/384, Thiran 0.06 w^3),
  half-band specs (ripple < 0.001 dB to 20 kHz, stopband < -96 dB) + image/alias rejection (< -90 dB) +
  round-trip latency (2x: 39 samples, 4x: 45.5 samples), FTZ guard flushes/restores, fast dB/log/exp error
  bounds, zero allocations on all process paths (counting global allocator, self-checked).
  Bench (Apple silicon, 64-sample blocks): biquad f32 165 ns (2.6 ns/sample), f64 164 ns; BW8 cascade 189 ns;
  SVF 268 ns, SVF modulated (re-designed every sample) 411 ns; 48ch bank SoA planar 1 stage 1.75 us vs scalar
  4.14 us (2.4x), 4 stages 3.77 us vs scalar 16.8 us (4.5x), interleaved 4 stages 4.76 us; oversampler 2x
  round trip 692 ns (10.8 ns/sample), 4x 1.40 us; envelope (program-dependent) 281 ns; RMS window 78 ns;
  Lagrange3 delay 154 ns; smoother fill 108 ns; fast dB->lin 82 ns.
  Gotchas: f32 TDF-II deviates > 0.01 dB in the deep stopband of LF HPFs (< 35 Hz) -> Cascade uses f64
  sections; use BiquadF64 for LF filters. f32 one-pole smoothers stall ~1e-5 short of target -> smoother state
  is f64. 4x oversampler latency is inherently half-integer (stage B runs at 4x); delay comp must handle it.
  Proposed follow-ups: minimum-phase / IIR half-band option for low-latency saturators; ADAA waveshaper
  helpers; f64-lane variant of BiquadBank if LF EQ bands need it; SoA bank of modulated SVFs.
