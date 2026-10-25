---
id: T009
title: Cross-check schema against libwing node definitions
stage: S1
status: todo
owner: -
depends: [T003]
branch: -
---

## Goal
Compare data/schema/wing-tree.json (from WING-Edit 3.3.4) with libwing's embedded node-definition records (fw 3.0.5, Apache-2.0): hashes, types, ranges, enums. Report diffs in docs/research/wing-edit-schema.md (append). Do not vendor libwing data into the repo.

## Acceptance criteria
- [ ] Diff report: matched/mismatched/missing counts with examples

## Log
- 2026-10-25 coordinator: created from T003 follow-up.
