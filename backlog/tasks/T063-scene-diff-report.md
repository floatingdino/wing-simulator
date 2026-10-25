---
id: T063
title: Scene diff report
stage: S6
status: review
owner: agent-T063
depends: [T011]
branch: task/T063
---

## Goal
CLI: human-readable diff between two .snap files (what changed in the mix).

## Acceptance criteria
- [ ] Diff output test

## Log
- 2026-10-25 coordinator: created.
- 2026-10-25 agent-T063: done. New `apps/wing-diff` (lib + bin, only dep wing-model). Usage:
  `wing-diff a.snap b.snap [--format text|markdown|json] [--filter ch/1..40,bus] [--ignore-ui]`.
  Both inputs load via `Snapshot::load` (1.x migrated to 3.x), entries diffed with `wing_model::diff`, grouped per
  strip (ch/aux/bus/main/mtx/dca/mgrp/fx/monitor, then I/O, Config, ...) using strip names from the scenes; values via
  the schema display formatting of each side's store (units, `-oo dB`, `2k00 Hz`); bools as on/off.
  Choices: model change (`dyn/mdl COMP -> 76LA`) collapses that block's model-specific params into one count line
  (non-model params like on/mix still listed); `--ignore-ui` drops `$ctl/**`, strip `led`, `cfg/mtr/**`, `cfg/rta/**`
  (names/colours/icons are kept; documented in lib.rs module docs). Unknown (extras) keys shown as raw JSON.
  Exit code 0 on success regardless of differences, 2 on usage/load error.
  Tests: `cargo test -p wing-diff` (synthetic pair with exact text, model collapse, filter/ignore-ui, formats,
  factory vs PUBLIC.snap smoke, 1.x vs its 3.x re-save = 0 diffs). `make check` passes.
  Proposed follow-ups: `--summary` per-strip counts for huge diffs.
