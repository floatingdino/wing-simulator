---
id: T034
title: Dynamics standard compressor
stage: S3
status: done
owner: agent-T034
depends: [T030, T004]
branch: task/T034
---

## Goal
Standard WING compressor/expander dynamics incl. knee, ratio, auto, detector modes, sidechain/key filter, mix, gain.

## Acceptance criteria
- [x] Static curve + time-constant tests

## Log
- 2026-10-25 coordinator: created.
- 2026-10-25 coordinator: scope = models COMP + EXP (models.md §3.2).
- 2026-10-25 agent-T034: started on task/T034 (worktree .claude/worktrees/t034). data/reference-curves/ absent (T00A in progress) -> implement per models.md formulas.
- 2026-10-25 agent-T034: (resumed after the original worktree was lost; redone in .claude/worktrees/t034.) Done -> review.
  - Files: crates/wing-blocks/src/dynamics.rs (new; `Dynamics` + `DynParams`, `static_gain_db`), src/lib.rs (`pub mod dynamics;` only),
    Cargo.toml (dep wing-dsp, dev-dep criterion, `[[bench]] dynamics`), tests/dynamics_no_alloc.rs, benches/dynamics.rs, Cargo.lock.
  - Implemented: COMP + EXP feed-forward log-domain gain computer (thr, 17-value ratio enum, quadratic soft knee), PEAK/RMS detector,
    attack/hold/release smoothing in dB (expander: attack = opening), env LOG (exponential) / LIN (constant dB/s), AUTO (dual release:
    slow stage charge 250 ms, release 4x rel clamped 0.3..8 s), dynsc key: SELF or external buffer (mono key feeds both channels) +
    key filter OFF/LP12/HP12/BP/NOTCH (SVF, f, q 0.44..10), dyn.mix parallel 0..100 %, dyn.gain make-up -6..+12 dB (ramped 5 ms),
    stereo link (max level), GR metering (`gain_reduction_db`, per channel, block peak). No allocation anywhere; latency 0.
  - Tests (`cargo test -p wing-blocks`, 16 unit + 1 no-alloc, all pass; `make check` passes): static curve vs f64 reference over
    thr/ratio/knee grid (< 0.01 dB) and end-to-end DC (< 0.1 dB) for COMP and EXP; RMS sine reading; attack/hold/release tau (+-1 %);
    LIN rate; AUTO programme dependence; expander open/close; mix/make-up; param ramps; key filter types; external key; link on/off.
  - Bench (`cargo bench -p wing-blocks --bench dynamics`, M-series): COMP mono 1x64 ~567 ns, stereo+key filter ~851 ns,
    EXP mono ~332 ns, 48 mono instances x64 ~20.4 us (~1.5 % of a 1.33 ms block on one core).
  - PENDING: data/reference-curves/ (T00A) did not exist at test time -> static curve NOT yet validated against WING-Edit.
    Unverified assumptions (named consts in dynamics.rs): KNEE_DB_PER_STEP = 2 dB, RMS_TAU_S = 10 ms, LIN_RELEASE_DB_PER_TAU = 10 dB,
    AUTO_* constants, EXP_MAX_RANGE_DB = 90, make-up applied to wet path only (mix before gain), AUTO keeps att/rel knobs active.
  - Proposed follow-ups: (1) validate static curve/knee against T00A curves once extracted (add a fidelity test reading the JSON);
    (2) `dynxo` crossover (frequency-selective dynamics: OFF/LO/HI/S6/S12/PEAK) is not implemented here, belongs in the dyn-slot
    wrapper (T035); (3) key listen (`dynsc.$solo`) routing in the engine (T038); (4) programme-dependent *attack* for AUTO if the
    reference shows one.
