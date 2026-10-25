---
id: T05A
title: Amps: RACKAMP, UKROCK, ANGEL, JAZZC, DELUXE, BODY
stage: S5
status: done
owner: claude-T05A
depends: [T050]
branch: task/T05A
---

## Goal
Amp/cab sims with oversampling per models.md §6.

## Acceptance criteria
- [x] Aliasing + behaviour tests

## Log
- 2026-10-25 coordinator: created by decomposing epics T040-T044/T051 after T004.
- 2026-10-25 claude-T05A: implemented in `crates/wing-blocks/src/models/fx_amp.rs` (+ `fx_amp/tests.rs`); status
  review. Own grey-box models, no third-party code / IRs / neural weights.
  - Amps (`Amp` + `AmpKind`): base-rate pre-EQ -> 4x oversampled section (`Oversampler4x`) with 1-4 preamp
    stages (coupling HP -> gain -> asymmetric cubic soft clipper with **ADAA2** and the linear part passed
    untouched -> 2-pole Miller LP), gain pot + gain-dependent bright-cap shelf, exact 3rd-order TMB tone stack,
    presence/depth shelves, 2-pole band limit, master, push-pull power stage (ADAA2) with feed-forward rail
    **sag** (anti-aliased clipped rectifier detector) -> base-rate post-EQ, IR-free 6-biquad **cab** sims
    (10 ms on/off crossfade), out gain, DC blocker. BODY = linear body-resonance EQ scaled by BODY.
  - Tone stack: transfer function derived by nodal analysis (FMV topology per Yeh & Smith; mid pot as rheostat
    m*R3, which is how Bassman/JCM800/AB763 wire it; my recollection of Yeh's printed coefficients was
    off for m < 1, so the coefficients are re-derived) with component sets BASSMAN/JCM800/DELUXE (AB763,
    fixed 6.8k mid) exact, ANGEL/JC120 inferred [I].
  - Every WING key of all 6 models mapped (schema slot order, ranges, defaults; test against wing-tree.json).
  - **Latency: 47 samples (0.98 ms @ 48 kHz) for every amp** = oversampler 45.5 + 6 samples at 4x (one per
    ADAA2 curve + pure alignment delay) / 4; BODY 0. Note: since slot latency = max over hosted models
    (T03E), every FX slot now reports 47.
  - Aliasing (4 kHz bin-exact tone, -6 dBFS, all amps at full gain, worst non-harmonic line 0..20 kHz):
    cab on <= -96 dBc; cab off -84 dBc (UKROCK, worst) .. -118 dBc. History: ADAA1 at 4x gave -62 dBc, the
    plain |x| sag detector -60 dBc (rectifier harmonics folded to ~DC and modulated onto every harmonic);
    ADAA2 + 2-pole inter-stage LPs + anti-aliased detector fixed both.
  - CPU (criterion `fx_amp`, 64-sample stereo block, M-series, machine shared with other agents): ANGEL
    ~18 us, UKROCK ~16, DELUXE ~13, RACKAMP ~12, JAZZC ~10 (oversampler ~3 of each), BODY 0.4. Key: the
    4x section runs stage-by-stage over the chunk with L/R interleaved (`os_section`); the first fused
    per-sample chain was latency-bound at 35 us.
  - Files: `crates/wing-blocks/src/models/fx_amp.rs`, `crates/wing-blocks/src/models/fx_amp/tests.rs`,
    `crates/wing-blocks/src/models/mod.rs` (pub mod), `crates/wing-blocks/src/model.rs` (6 registrations +
    catalog test list), `crates/wing-blocks/Cargo.toml` ([[bench]] fx_amp), `crates/wing-blocks/benches/fx_amp.rs`,
    `crates/wing-blocks/tests/fx_amp_no_alloc.rs`, `crates/wing-engine/tests/fx.rs` (expected strip latency /
    switch-off crossfade now account for the non-zero slot latency).
  - Tests (`cargo test -p wing-blocks fx_amp`): schema mapping/defaults/clamping, curve antiderivatives +
    ADAA2 transparency/1-sample delay, first-order sections, tone stack analytic == independent nodal solve
    (rel err < 1e-9) and digital filter == analytic (0.01 dB warped, 0.05 dB unwarped <= 6 kHz), control
    directions, 47-sample alignment, aliasing < -80 dBc, gain -> THD, ANGEL saturation, default levels within
    +/-6 dB of input, silence in -> silence out, cab removes fizz + click-free switch, tone/presence/bright/
    mid boost/bottom/buzz/punch/crunch act, sag compresses (-5 / -8 dB), no NaN/inf under extreme params and
    NaN/inf/1e30 input, odd block sizes, BODY passthrough at 0 + resonance; zero-alloc integration test.
    `make check` and `make check-full` pass. Bench: `cargo bench -p wing-blocks --bench fx_amp`.
  - Proposed follow-ups: (1) voice constants (stage gains, cab sections, ANGEL/JC120 stacks, BODY curve,
    PRESENCE/BRIGHT ranges, make-up gains) are [I]: tune against console captures when available;
    (2) optional SIMD (f64x2 L/R) of the 4x section if FX CPU budget gets tight; (3) engine: a per-slot
    latency equal to the *selected* model's would avoid padding every FX insert by 47 samples (T03E design
    decision, not changed here).
