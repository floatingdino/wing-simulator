---
id: T074
title: Fast `make check` (< 5 min) + `make check-full`
stage: S7
status: review
owner: T074 agent
depends: []
branch: task/T074
---

## Goal
`make check` now takes > 1 h in debug under load (full-grid fidelity/reference tests, long drift sims, 30 s+ model tests). Split: `make check` = fmt + clippy + fast tests (sampled grids, short sims, release-only heavy tests ignored via `#[ignore]` or a `slow` feature) in < 5 min on this machine; `make check-full` = everything (run tests with `--release` where it speeds up). Keep coverage meaningful in the fast set (e.g. every model at a few grid points). Update CLAUDE.md and backlog/README.md so agents run `make check` routinely and `make check-full` before finishing a DSP task.

## Acceptance criteria
- [x] `make check` < 5 min wall time; `make check-full` passes
- [x] Docs updated

## Log
- 2026-10-25 coordinator: created; T062 agent reported > 1 h make check.
- 2026-10-25 T074 agent: measured each test binary on current main (dev profile, wing-blocks/hostio already opt-level 2 from T062): whole workspace ~100 s sequential; slowest: wing-blocks lib 26 s, wing-engine mix_alloc 15 s, monitor_alloc 14 s, cpp_selftest 11 s, filter_tool_reference 9 s, meters 6 s, fidelity_reference 5 s; everything else <= 3 s. `make check` (fmt+clippy+test) = 1m46s total, so no test needed `#[ignore]` now; the >1 h was pre-T062 opt-level fix. Added `make check-full` (fmt, clippy, `cargo test --workspace --release -- --include-ignored`): passes, 1m58s. Documented the #[ignore = "slow: ..."] policy (> ~10 s dev tests) in CLAUDE.md and backlog/README.md. Files: Makefile, CLAUDE.md, backlog/README.md.
