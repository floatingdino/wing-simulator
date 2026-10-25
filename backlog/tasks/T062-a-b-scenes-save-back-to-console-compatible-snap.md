---
id: T062
title: A/B scenes + save back to console-compatible .snap
stage: S6
status: review
owner: agent-T062
depends: [T011, T026]
branch: task/T062
---

## Goal
A/B compare two states instantly; export current state as .snap for loading on a real WING.

## Acceptance criteria
- [x] A/B slots + OSC/CLI, diff-apply via ParamStore, native fan-out verified in loopback
- [x] Export validated (0 rejected, 3.x, round trip) + changes.md report; header choice documented
- [ ] Exported .snap loads in WING-Edit offline / real console (needs hardware or WING-Edit; unverified)

## Log
- 2026-10-25 coordinator: created.
- 2026-10-25 agent-T062: done, status review.
  - wing-model: `Snapshot::apply_diff(target, live, origin, skip)` (+ `ApplyReport`): sets only persisted params that
    differ, selectors first, through `set_raw`/`set_str` (hooks, native/OSC fan-out, RT queue).
  - wing-sim `src/ab.rs` (+ fields in `SceneLib`): A/B slots (`store_a/store_b/a/b/toggle`; leaving a side captures
    its edits; selecting the current side reverts it), `export` (validated, writes `.snap` + `.changes.md` via
    wing-diff vs "scene as loaded", tracked in `SceneLib::loaded`: set by --scene, state restore, library recall).
    OSC `/wingsim/ab/*`, `/wingsim/export ,s path` (wing-proto service.rs) and `$ctl/lib/$action` `AB:*`/`EXPORT:path`.
    CLI: `--ab-a/--ab-b FILE`, `--export PATH` (batch, exits), `--export-on-exit PATH`, `--export-header advertised|factory`.
  - Header: genuine 3.x headers kept; simulator-made/migrated scenes get the advertised firmware release header
    (default 3.3.4-0-g0c2b9d4a:release) + model/serial/name, `created` = now UTC, `type` snapshot.11 (see
    docs/research/snapshot-migration.md). Docs: docs/user/ab-compare-and-export.md.
  - Tests: `cargo test -p wing-sim --bin wing-sim ab::` (incl. OSC -> native TCP client loopback on ephemeral ports),
    `cargo test -p wing-proto osc::service`. `make check` passes.
  - Gotchas: unverified that a real console / WING-Edit accepts the advertised header or `snapshot.11` from 3.3.x
    firmware; the live user daemon (2222/2223) was never touched.
  - Proposed follow-ups: verify import on hardware/WING-Edit; A/B state in `$ctl` for a UI indicator; export to the
    library (`STORE`) in the same action; A/B of audio-path-only parameters (not persisted by snapshots) is not covered.
