---
id: T022
title: Engine RT host + parallel scheduler
stage: S2
status: in_progress
owner: agent-T022
depends: [T001]
branch: task/T022
---

## Goal
RT engine driver: time-constraint thread, worker pool joined to os_workgroup, block DAG scheduler with atomic work claiming, CPU load + xrun stats. Driven either by shm (live) or a file clock (offline).

## Acceptance criteria
- [ ] Bench: 48 dummy strips + 16 dummy FX scheduled with <5% overhead
- [ ] No allocations on RT path (test with alloc counter)

## Log
- 2026-10-25 coordinator: created.
- 2026-10-25 agent-T022: started. Plan: graph.rs (Processor/NodeIo/GraphBuilder, levels, preallocated planar arena), sched.rs (driver+workers, epoch-tagged atomic claim/done counters per level, spin-then-park), rt.rs (thread_policy_set time-constraint, os_workgroup join), clock.rs (Live/Offline), stats.rs (atomics). Deps: libc only.
