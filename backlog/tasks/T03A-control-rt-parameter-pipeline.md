---
id: T03A
title: Control->RT parameter pipeline
stage: S3
status: todo
owner: -
depends: [T010, T022]
branch: -
---

## Goal
Lock-free param change transport into RT engine with per-parameter smoothing and model hot-swap without clicks.

## Acceptance criteria
- [ ] Zipper-noise test
- [ ] No RT alloc on model swap (preallocated)

## Log
- 2026-10-25 coordinator: created.
