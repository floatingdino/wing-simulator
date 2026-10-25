---
id: T074
title: Fast `make check` (< 5 min) + `make check-full`
stage: S7
status: todo
owner: -
depends: []
branch: -
---

## Goal
`make check` now takes > 1 h in debug under load (full-grid fidelity/reference tests, long drift sims, 30 s+ model tests). Split: `make check` = fmt + clippy + fast tests (sampled grids, short sims, release-only heavy tests ignored via `#[ignore]` or a `slow` feature) in < 5 min on this machine; `make check-full` = everything (run tests with `--release` where it speeds up). Keep coverage meaningful in the fast set (e.g. every model at a few grid points). Update CLAUDE.md and backlog/README.md so agents run `make check` routinely and `make check-full` before finishing a DSP task.

## Acceptance criteria
- [ ] `make check` < 5 min wall time; `make check-full` passes
- [ ] Docs updated

## Log
- 2026-10-25 coordinator: created; T062 agent reported > 1 h make check.
