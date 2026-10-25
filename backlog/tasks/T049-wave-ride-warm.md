---
id: T049
title: WAVE, RIDE, WARM
stage: S4
status: review
owner: engineering-agent
depends: [T045]
branch: task/T049
---

## Goal
Transient designer, auto rider, Soul warmth preamp per models.md §3.1.

## Acceptance criteria
- [x] Behaviour tests

## Log
- 2026-10-25 coordinator: created by decomposing epics T040-T044/T051 after T004.
- 2026-10-25 engineering-agent: implemented WAVE (`TransientDesigner`), RIDE (`AutoRider`), WARM (`WarmthPre`) in
  `crates/wing-blocks/src/models/dyn_special.rs` (tests in `models/dyn_special/tests.rs`), registered in
  `model.rs` + `models/mod.rs`, bench `benches/dyn_special.rs` (+ `[[bench]]` in Cargo.toml), zero-alloc test
  `tests/dyn_special_no_alloc.rs`. Assumptions appended to `docs/VERIFY-ON-HARDWARE.md` (WAVE/RIDE/WARM rows).
  - WAVE: differential envelope (fast follower vs slow follower of the fast one, dB difference, level
    independent: 24 dB louder input gives the same gain trajectory within 0.15 dB); attack = onset region,
    sustain = decay region faster than ~29 dB/s; steady tones / bass ripple pass at unity.
  - RIDE: RMS level (100 ms), error x (1 - 1/ratio), +-range, slew `spd` 2..200 dB/s, hold / INF below `thr`.
  - WARM: tanh(u+b)-tanh(b) shaper, bias sweeps 2nd (hrm -100) to 3rd (+100), 2x oversampling (latency 39) +
    ADAA on the nonlinear remainder only (linear part exact, no HF droop), colour tilt, wmix, trim. 10 kHz
    alias < -75 dBc at 125 % drive.
  - Bench (M1, stereo 64 block): WAVE 0.8 us, RIDE 0.14 us, WARM 5.2 us.
  - Test: `make check` (wing-blocks lib tests `dyn_special`, `cargo test -p wing-blocks --test dyn_special_no_alloc`).
  - Notes: WARM ignores the side-chain (preamp); fw 1.13 snapshot key `mix` vs `wmix` aliasing is the loader's
    job (slot 5 here). RIDE ratio accepts `Float`/`Int` as ratio value, `Enum` as list index.
  - Proposed follow-ups: telemetry for RIDE reports only gain reduction (boost not shown on the GR meter);
    engine-side GR meter mapping for RIDE/WAVE (`ride_gr_meter`, `wave_gr_meter` provided).
