---
id: T072
title: Fidelity test suite
stage: S7
status: review
owner: agent-T072
depends: [T03C]
branch: task/T072
---

## Goal
Automated measurement tests of every model against documented curves; reference data in fixtures.

## Acceptance criteria
- [x] CI target: `cargo test -p wing-blocks --test fidelity_all` (sampled, in make check); full grids in make check-full
- [x] docs/fidelity-report.md generated (`make fidelity-report`)
- [x] tools/verify-hardware kit (wing-verify gen/compare, capture.sh, README) + scene table and results block in docs/VERIFY-ON-HARDWARE.md

## Log
- 2026-10-25 coordinator: created.
- 2026-10-25 agent-T072: done, status review. Branch task/T072.
  - (1) `crates/wing-blocks/tests/fidelity_all.rs`: every model with WING-Edit-derived data (LC/HC + tool filters, STD EQ ch/aux/bus, 7 alt EQs, GATE/DUCK/COMP/EXP static laws (analytic + processed DC), key filter/crossover, FX GEQ, 7 FX EQs, SPKMAN linear part) with per-scope max error and tolerance. The `source_of` table classifies every catalog model (exact port / reference curve / published spec / inferred) and panics for an unclassified one, so new models must be added there. `fidelity_all_sampled` (~7 s, in make check), `fidelity_all_full` (#[ignore], make check-full), `committed_report_lists_every_model` (docs/fidelity-report.md must list every catalog model). `make fidelity-report` writes docs/fidelity-report.md (full grids, release, ~75 s). Result: all measured models PASS, the rest UNVERIFIED (no reference data: published spec / inferred). Worst measured: EXP processed static 0.11 dB (tol 0.2); analytic checks <= 0.001 dB. Gotcha: STD EQ bands below 60 Hz with Q 10 ring for seconds, a 16k-sample impulse response under-reports them by 0.27 dB, so those curves use a 64k IR (the older fidelity_reference.rs only samples every 7th curve and never hit it).
  - (2) `apps/wing-render/src/verify/` + bin `wing-verify` (`gen`, `compare`, `docs`) and `tools/verify-hardware/{README.md,verify.sh,capture.sh}`. One 49 s mono suite (pilot, sweep, staircase, steps, noise bursts, 1 kHz/100 Hz tones, end pilot), 36 scenes (13 in the quick tier), each = factory + base routing (ch1 from USB 1 to MAIN, MAIN L/R on USB out 1/2) + the block under test; parameter paths are validated when `gen` runs. `compare` renders each scene with wing-render, aligns the recording on the pilot (device latency is reported separately), checks drift with the end pilot, and reports fr / stairs / steps / bursts / tone deviations in dB with tolerances, to stdout, report.md and (--update-doc) the Results block of docs/VERIFY-ON-HARDWARE.md. The doc also has a generated scene -> assumption table and a list of what the kit cannot check.
  - Tests: `cargo test -p wing-render --test verify_kit` (fast: gen all scenes; synthetic recording aligned to the frame, gain / saturation / slow-envelope perturbations flagged); `--release -- --include-ignored` (check-full, ~75 s): every scene rendered, silence/clip guard (peak 0.01..0.95), compared with a delayed copy of itself (< 0.2 dB), +3 dB flagged. Added `[profile.dev.package.wing-render] opt-level = 2` (FFTs).
  - `make check` passes. Not run on a real WING (none available to the agent). Untested assumptions: the sox play/record commands in capture.sh, the snapshot import path on the console, and that `ch/1/in/conn/grp = USB` + `io/out/USB/N = MAIN.N` behave as in the sim.
  - Proposed follow-ups: (a) gate scenes use model defaults, so thresholds may sit outside the staircase (-60..-4 dBFS): tune per-model thr/ratio after the first hardware run; (b) DS902/DEQ/DEQ2 scenes need band-limited key signals (add a "sweep with level steps" section); (c) 44.1 kHz variant of the kit; (d) LIMITER/DE-S2/DEQ3/C5-CMB have no reference data and no scene yet.
