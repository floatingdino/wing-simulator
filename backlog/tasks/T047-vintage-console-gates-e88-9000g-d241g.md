---
id: T047
title: Vintage console gates: E88, 9000G, D241G
stage: S4
status: done
owner: agent-T047
depends: [T045, T032]
branch: task/T047
---

## Goal
Gate models per models.md §3.1.

## Acceptance criteria
- [x] Static curve + timing tests (+ hysteresis, zero-alloc, bench)

## Log
- 2026-10-25 coordinator: created by decomposing epics T040-T044/T051 after T004.
- 2026-10-25 agent-T047: implemented E88, 9000G, D241G in `crates/wing-blocks/src/models/gate_vintage.rs` (one shared
  stereo-linked gate/expander core `VintageGate`; slots/ranges/defaults from `data/schema/wing-tree.json`), registered
  with 3 lines in `model.rs::create_model`, `pub mod gate_vintage;` in `models/mod.rs`, `[[bench]] gate_vintage`.
  Status -> review.
  - Sources per constant (no WING static curves exist for these models: `respf` is a stub in dyn-models.json):
    - Ranges/defaults/slot order: wing-tree.json + models.md 3.1 (E88: thr -40..0, hyst 0..25 "EXP", range 0..60, rel
      100..3000, fast, m40; 9000G: thr -40..0, range 0..40, hld 10..4000, rel 100..4000, fast, mode GATE/EXP; D241G: thr
      -80..0, slow).
    - E88 attack 50 us fast / ~2 ms, no hold, hysteresis as open/close gap: models.md 3.1 (88RS documentation).
    - 9000G attack ~100 us fast / ~1.5 ms, EXP ~1:2 slope, range 0..40, hold 0..4 s: models.md 3.1 (SSL E/G/J dynamics).
    - D241G: fixed fast attack, programme-dependent release, slow switch lengthens it, hard gate: models.md 3.1
      (Drawmer DL241/DS201 manual).
    - Key: external key via `sidechain`, stereo-linked max of both sides; key filters live in the slot wrapper (the
      hardware key filters are not exposed by these WING models).
    - GR meter `gr_meter()` = WING-Edit `gate_gain_<m>` mapping (E88 GR/20 cap 2.5, 9000G cap 1, D241G (GR-1)/11),
      checked against dyn-models.json.
    - Assumptions (all in VERIFY-ON-HARDWARE.md): detector release 20 ms; release time = traverse of 60/40/80 dB,
      linear in dB (hardware VCA ramps); E88 "EXP" end stop = 2:1 expander no hysteresis; 9000G/D241G 2 dB
      hysteresis; D241G hold/release constants and open-time charge.
  - Files: `src/models/gate_vintage.rs` (new), `src/models/mod.rs`, `src/model.rs`, `Cargo.toml`,
    `benches/gate_vintage.rs`, `tests/gate_vintage.rs`, `docs/VERIFY-ON-HARDWARE.md`.
  - Tests: unit tests (static curve gate/expander incl. range and m40, hysteresis window/no-reopen, attack fast/slow,
    release traverse timing, SSL hold, D241G exact-zero gate, fixed attack, programme-dependent and slow release,
    stereo link + external key, param clamping), integration (reference GR meter, factory telemetry, zero alloc).
    `make check` passes.
  - Bench (M1, stereo 64-sample block): E88 187 ns, 9000G 293 ns, D241G 365 ns.
  - Proposed follow-ups: measure real units/WING for hysteresis, attack and release laws (T072).
