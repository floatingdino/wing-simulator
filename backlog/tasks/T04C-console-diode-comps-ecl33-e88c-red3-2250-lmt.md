---
id: T04C
title: Console/diode comps: ECL33, E88C, RED3, 2250, LMT
stage: S4
status: review
owner: claude-T04C
depends: [T045, T034]
branch: task/T04C
---

## Goal
Neve 33609/88, Red3, API 2250-style, LMT per models.md §3.2.

## Acceptance criteria
- [x] Static curve + time-constant tests

## Log
- 2026-10-25 coordinator: created by decomposing epics T040-T044/T051 after T004.
- 2026-10-25 claude-T04C: implemented ECL33, E88C, RED3, 2250, LMT in `crates/wing-blocks/src/models/comp_console.rs`
  (`ConsoleComp`/`ConsoleKind`), registered in `model.rs` + `models/mod.rs`, bench `benches/comp_console.rs`
  (`[[bench]] comp_console`), integration test `tests/comp_console.rs` (GR meter vs `dyn-models.json`, factory,
  zero alloc). Shared core: one `Stage` (peak/RMS detector, FF static curve or FB loop gain k = R-1 with attack
  scaled by 1+k so closed-loop times match the specs, dB attack/release, two-stage auto release); reuses
  `comp_vca` constants (DBU_AT_0DBFS, GR_MAX_DB, AUTO_* two-stage release). The comp_vca slot/knee helpers are
  private, so a small slot table is duplicated rather than editing comp_vca (avoids merge conflicts with T04D/E).
  Sources per constant:
  - ECL33: AMS Neve 33609 spec (https://ams-neve.com/33609/): limiter thr +4..+15 dBu, "< 0.1 dB output change
    for 10 dB input step" (=> ratio 200), limiter attack fast 2 / slow 4 ms, recovery 50..800 ms; compressor
    ratios 1.5..6, attack fast 3 / slow 6 ms, recovery 100..1500 ms; A1 = 100 ms / 2 s, A2 = 50 ms / 5 s; THD
    < 0.2 % compressing (6:1, 20 dB), < 0.45 % limiting (colour calibrated to 0.1 % at 20 dB GR). Feedback
    topology + diode soft knee: models.md ("feedback-ish", soft knee). Step lists/enums/defaults: schema.
  - E88C: models.md + schema (thr +20..-10 dBu, THR -20 pull, ratio 1..10 "LIM", rel 10..3000 "AUTO");
    FAST 0.1 ms from Neve 88R dynamics ("extra fast attack circuit 0.1 ms", Thomann 88R LBC listing); UAD
    88RS docs confirm fixed fast/slow attack, 0.01..3 s release + auto, continuous ratio.
  - RED3: schema ranges; soft knee (Focusrite Red Plugin Suite manual); release 0.1..4 s + programme-dependent
    auto (Focusrite user guide / hardware listing). The hardware's separate limiter is not exposed by WING.
  - 2250: API 225L (https://apiaudio.com/?p=3797): thr +10..-20 dBu, ratio 1..infinity, HARD/SOFT knee, NEW =
    feed-forward RMS, OLD = feedback; attack switch Fast 2 / Medium 18 / Slow 75 ms and release 50 ms..3 s
    (SoundPure 225L listing; matches WING's FAST/MED/SLOW + 50..3000 ms). WING exposes no THRUST control, so
    none is modelled.
  - LMT: models.md (SSL 4000E LMC, fixed fast high-ratio FF comp + transient section); SSL LMC docs: fixed
    attack/release. All LMT numbers are assumptions.
  - GR meter: dyn-models.json `comp_gain_*`: GR/20 capped at 1.0 for all five.
  Assumptions (all listed in docs/VERIFY-ON-HARDWARE.md): threshold calibrations, knee widths, ECL33 FB topology
  / no SC HPF / colour law, E88C SLOW attack, 2250 knob laws + RMS 1 ms + OLD cap 31:1, auto-release charge,
  all LMT constants.
  Tests: `cargo test -p wing-blocks comp_console` (9 unit: static curves vs published ratios/knees for every
  model, ECL33 attack/recovery/A1/A2 vs AMS spec, limiter < 0.1 dB per 10 dB, THD within spec, E88C
  pull/LIM/AUTO, RED3 soft knee/times/auto, 2250 attack switch 2/18/75 ms + NEW/OLD + knob laws, LMT
  amount/transients, stereo link/sidechain/params, telemetry; 3 integration). `make check` passes.
  Bench (M1, 64-sample stereo block, defaults): ECL33 505 ns, E88C 315 ns, RED3 319 ns, 2250 362 ns, LMT 437 ns.
  Proposed follow-ups: verify LMT/E88C constants on hardware or WING-Edit captures; ECL33 100 Hz SC HPF (the AMS
  spec mentions a 100 Hz first-order HPF next to the compressor attack figure) if confirmed; LMC colouration
  once documented; bench with the limiter/transient sections enabled.
