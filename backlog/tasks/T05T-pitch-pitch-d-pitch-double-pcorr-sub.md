---
id: T05T
title: Pitch: PITCH, D-PITCH, DOUBLE, PCORR, SUB
stage: S5
status: review
owner: T05T-agent
depends: [T050]
branch: task/T05T
---

## Goal
Pitch shifting/correction per models.md §6 (evaluate permissive libs per dsp-reuse.md).

## Acceptance criteria
- [x] Pitch accuracy tests, latency reported

## Log
- 2026-10-25 coordinator: created by decomposing epics T040-T044/T051 after T004.
- 2026-10-25 T05T-agent: implemented PITCH, D-PITCH, DOUBLE, PCORR, SUB in
  `crates/wing-blocks/src/models/fx_pitch.rs` (+ `fx_pitch/tests.rs`), registered in model.rs / models/mod.rs,
  bench `benches/fx_pitch.rs`, no-alloc test `tests/fx_pitch_no_alloc.rs`. Params mapped per schema (the schema's
  D-PITCH has no 1k..16k EQ and PCORR no PHASE/WAVE, unlike the models.md table).
  - Algorithm: own dual-head delay-line shifter with correlation-chosen splice jumps (coarse + full-res +
    parabolic) and raised-cosine cross-fades; no third-party code (Signalsmith Stretch = tens of ms latency,
    Rubber Band GPL). PCORR: YIN on a ~12 kHz decimated mono sum every ~2.7 ms + full-rate parabolic
    refinement, 75..1000 Hz, scale snap with hysteresis (no notes enabled = chromatic [I]), AMOUNT = fraction
    corrected [I], SPEED = retune tau 400^((100-SPD)/99) ms [I]; splices by whole detected periods
    symmetrically around a fixed centre delay. SUB: OC-2 style Schmitt flip-flop /2 and /4 polarity switching
    of the band-limited input, RANGE LOW/MID/HIGH = 200/400/800 Hz [I], dry kept at unity [I].
  - Latency (coordinator note): PCORR reports **360 samples @48 kHz** (7.5 ms; 331 @44.1k, 720 @96k), exact
    (output = input delayed by latency() with no correction; mean head delay = latency under correction).
    PITCH / D-PITCH / DOUBLE / SUB report 0: dry is not delayed; the shifted voice carries DELAY + shifter
    window (<= ~31 ms, mean ~12-19 ms) as part of the effect (`StereoPitch::wet_delay_range`). Because PCORR is
    allowed in FX1-16, every FX slot's constant latency is now 360 samples (strips with FX inserts run at
    max(natural, own + 360)). WING's real pitch FX latency is unknown (signal-flow.md §11, not documented).
  - wing-engine/tests/fx.rs: two insert tests assumed zero-latency slots; now compute strip latency with the
    slot latency, wait for the strips' latency to settle, and model the switch-off ramp at slot-output time.
  - Tests: pitch within 5 cents on sines (+/-12 st, cents) and 5-harmonic tones; D-PITCH pan/level/shift;
    DOUBLE modes/spread; PCORR detection < 1 cent, scale snap (chromatic, C major, C only, A4 = 432, harmonic
    tone), amount 50 %, speed; exact latency; SUB tracks /2 and /4 within 5 cents and follows pitch changes;
    zero alloc; extreme settings finite. `make check` and `make check-full` pass.
  - Bench (`cargo bench -p wing-blocks --bench fx_pitch`, M-series, stereo 64-sample block): PITCH 1.6 us,
    D-PITCH 1.0 us, DOUBLE 1.5 us, PCORR 8.0 us, SUB 0.7 us.
  - Proposed follow-ups: (1) per-model latency for FX slots (or a lower-latency PCORR mode) if 7.5 ms on every
    FX insert is unacceptable; (2) measure WING's PITCH/PCORR latency and DOUBLE/SUB voicing on a console.
