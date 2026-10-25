---
id: T022
title: Engine RT host + parallel scheduler
stage: S2
status: todo
owner: -
depends: [T001]
branch: -
---

## Goal
RT engine driver: time-constraint thread, worker pool joined to os_workgroup, block DAG scheduler with atomic work claiming, CPU load + xrun stats. Driven either by shm (live) or a file clock (offline).

## Acceptance criteria
- [ ] Bench: 48 dummy strips + 16 dummy FX scheduled with <5% overhead
- [ ] No allocations on RT path (test with alloc counter)

## Log
- 2026-10-25 coordinator: created.
