---
id: T031
title: Input stage + filter block (standard)
stage: S3
status: done
owner: agent-T031-T032
depends: [T030, T004]
branch: task/T031-T032
---

## Goal
Gain/trim, polarity, input delay, mono/stereo handling, filter block standard (low cut, high cut, tilt/filter utility per models.md).

## Acceptance criteria
- [x] Frequency-response tests within 0.1 dB of spec

## Log
- 2026-10-25 coordinator: created.
- 2026-10-25 agent-T031-T032: started on branch task/T031-T032 (worktree .claude/worktrees/t031-t032; first worktree was lost to an API interruption before any commit).
- 2026-10-25 agent-T031-T032: done, status review. Files: `crates/wing-blocks/src/input.rs` (InputStage/InputParams:
  source gain `g` + `trim`, balance, `inv`, delay M/FT/MS/SMP up to 500 ms, source mode M/ST/M-S), `src/filter.rs`
  (FilterBlock/FilterParams: LC 6/12/18/24, HC 6/12, Butterworth via f64 `Cascade`), `src/lib.rs` (pub mods),
  `Cargo.toml` (wing-dsp dep, criterion bench), `tests/no_alloc_t031_t032.rs`, `benches/input_filter_gate.rs`.
  Smoothing: gains 5 ms one-pole (polarity crosses zero), delay 50 ms glide (linear interp), cutoff 20 ms in log2 Hz
  with coeff update every 16 samples, on/off 5 ms dry/wet crossfade; off = bit-exact bypass.
  Test: `cargo test -p wing-blocks` (LC/HC all slopes vs closed-form digital Butterworth within 0.1 dB from 10 Hz to
  21.5 kHz down to -60 dB, -3.01 dB at fc; delay impulse/mode conversion; smooth switching/sweeps; zero-alloc).
  `make check` passes. Bench (M-series, 64-sample stereo): input+delay 159 ns, LC24+HC12 389 ns.
  Scope notes: PRE EQ (`peq`) sits in the E block (signal-flow.md §2) -> left to T033; filter tool TILT/MAX/AP1/AP2
  is T046. Assumptions to verify (T072): Butterworth shape, balance = attenuate opposite side, M-S decode
  L=M+S/R=M-S, 343 m/s.
  Proposed follow-ups: fw 1.13 snapshot without `lcs` -> load as 12 dB/oct (snapshot loader, models.md note);
  source `pol`/`mute` live on the source object (routing task T024/T035).
