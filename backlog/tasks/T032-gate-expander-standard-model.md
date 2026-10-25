---
id: T032
title: Gate/Expander standard model
stage: S3
status: done
owner: agent-T031-T032
depends: [T030, T004]
branch: task/T031-T032
---

## Goal
Standard WING gate/expander incl. sidechain source/filter, range, hold, attack/release, modes.

## Acceptance criteria
- [x] GR curve and timing tests

## Log
- 2026-10-25 coordinator: created.
- 2026-10-25 coordinator: scope = models GATE + DUCK (models.md §3.1).
- 2026-10-25 agent-T031-T032: started on branch task/T031-T032 (worktree .claude/worktrees/t031-t032; first worktree was lost to an API interruption before any commit).
- 2026-10-25 agent-T031-T032: done, status review. File: `crates/wing-blocks/src/gate.rs` (Gate with GateParams
  {model GATE|DUCK, thr, range 3..60 (60 = oo/mute), att, hld (GATE 1..1000 / DUCK 30..30000 ms), rel, acc, ratio
  1:1.5..1:4|gate} + SidechainParams `gatesc` {type OFF/LP12/HP12/BP/NOTCH (TPT SVF), f, q, src SELF|CH.n, tap,
  key_listen}). External key buffer is passed to `process(l, r, Option<(kl, kr)>)` by the engine (used only when
  src != SELF); `key_signal()` exposes the filtered key for key listen / solo routing (T038). Stereo-linked peak
  detector (instant attack, 10 ms release), open/hold/release state machine, gain slewed linearly in dB (attack /
  release = time to traverse the full range). Expander: closed gain = (r-1)(level-thr) clamped to -range.
  ACCENT = hysteresis 0..10 dB (close at thr - acc*0.1 dB). `gain_db()` for GR metering.
  Test: `cargo test -p wing-blocks gate` (hard-gate static curve + ranges, oo = exact silence, expander curves for
  all ratios, hysteresis open/close, attack/hold/release timing to +-2 samples, ducker on external key incl. hold
  and SELF ignoring key, SC filter HP/LP + key listen, zero-alloc). `make check` passes. Bench: gate+SC HP 600 ns,
  ducker 304 ns per 64-sample stereo block.
  Assumptions to verify (T072): detector law/release, accent law, time definitions, DUCK `rel` units (fixture 8.02).
  Not included: gate-slot wrapper `{on, mdl, mix, gain}` (schema shows mix/gain on the gate slot in fw 3.x) -> T035/T045.
