---
id: T05D
title: Delays: ST-DL, TAP-DL, TAPE-DL, OILCAN, BBD-DL
stage: S5
status: in_progress
owner: claude-T05D
depends: [T050]
branch: task/T05D
---

## Goal
Per models.md §6; tap tempo sync.

## Acceptance criteria
- [ ] Delay-time accuracy, feedback stability

## Log
- 2026-10-25 coordinator: created by decomposing epics T040-T044/T051 after T004.
- 2026-10-25 claude-T05D: started on task/T05D. Schema has no global tempo/tap key (checked data/schema), so each
  model uses its own time params (ST-DL / TAP-DL `fact` note factors cover dotted and triplet values).
