---
id: T033
title: EQ standard model
stage: S3
status: in_progress
owner: agent-T033
depends: [T030, T004]
branch: task/T033
---

## Goal
Standard WING channel/bus/main EQ (shelves + parametric bands per models.md), incl. bypass and band types.

## Acceptance criteria
- [ ] Response tests vs spec (magnitude/Q)

## Log
- 2026-10-25 coordinator: created.
- 2026-10-25 coordinator: scope = model STD + 3-band pre-EQ (peq) (models.md §4).
- 2026-10-25 agent-T033: started (fresh worktree from main 6372f6a after API interruption). Plan: crates/wing-blocks/src/eq.rs (StdEq ch/aux/bus + PreEq), tests/eq_no_alloc.rs, benches/eq.rs.
