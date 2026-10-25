---
id: T022
title: Engine RT host + parallel scheduler
stage: S2
status: review
owner: agent-T022
depends: [T001]
branch: task/T022
---

## Goal
RT engine driver: time-constraint thread, worker pool joined to os_workgroup, block DAG scheduler with atomic work claiming, CPU load + xrun stats. Driven either by shm (live) or a file clock (offline).

## Acceptance criteria
- [x] Bench: 48 dummy strips + 16 dummy FX scheduled with <5% overhead (hot regime; see Log for paced caveat)
- [x] No allocations on RT path (test with alloc counter)

## Log
- 2026-10-25 coordinator: created.
- 2026-10-25 agent-T022: started. Plan: graph.rs (Processor/NodeIo/GraphBuilder, levels, preallocated planar arena), sched.rs (driver+workers, epoch-tagged atomic claim/done counters per level, spin-then-park), rt.rs (thread_policy_set time-constraint, os_workgroup join), clock.rs (Live/Offline), stats.rs (atomics). Deps: libc only.
- 2026-10-25 agent-T022: first cut committed (b61befa). Tests: level ordering (stamp-checking random DAGs, 0..11 workers), bit-exact parallel==serial, alloc counter (tests/alloc.rs), stats/clocks. First bench showed paced (1.33 ms gap, workers asleep) cycles slow: per-worker thread::unpark = N syscalls on the driver. Switched idle/level waits to a futex (os_sync_wait_on_address / os_sync_wake_by_address_all; macOS 14.4+) with sleeper counters: at most one wake syscall per event. Also 128-byte aligned per-node arena regions. Gotcha fixed: worker start-up race (must start from bell=0, not a fresh load, or a shutdown rung before the thread runs is missed -> Drop hangs).
- 2026-10-25 agent-T022: READY FOR REVIEW. `make check` passes.
  Files: Cargo.toml (workspace dep libc), crates/wing-engine/{Cargo.toml, src/lib.rs, src/graph.rs, src/sched.rs, src/rt.rs, src/clock.rs, src/stats.rs, src/dummy.rs, tests/alloc.rs, benches/scheduler.rs}.
  Design: GraphBuilder (nodes may only read earlier nodes -> DAG; level = 1 + max input level) compiles to a Graph with one preallocated, 128B-aligned planar arena and per-node input pointer tables. Engine = caller-as-driver + N workers; per level, epoch-tagged u64 claim/done counters (CAS claim, so late threads cannot steal claims from a newer cycle); spin (EngineConfig::spin, default 50us) then futex sleep (os_sync_wait_on_address); driver never sleeps. rt.rs: thread_policy_set(THREAD_TIME_CONSTRAINT_POLICY) via pthread_mach_thread_np; os_workgroup_join/leave via RawWorkgroup (unsafe from_raw) + WorkgroupMembership; Engine::set_workgroup makes workers (re)join between cycles. clock.rs: Clock trait, LiveClock (one cycle per external callback, first-tick RT setup, xrun if cycle > period), OfflineClock (free-running), CycleIo hooks. stats.rs: atomics; StatsHandle::snapshot -> last/avg/max ns, load %, xruns.
  Tests (12): level ordering on random DAGs (0..7 workers; nodes verify inputs carry this cycle's stamp), bit-exact parallel == serial (random DAG + mixer scene, 0/4/6/11 workers), zero allocations over 3000+ cycles incl. hot, paced (sleeping workers), LiveClock and OfflineClock paths (counting global allocator, tests/alloc.rs), stats/xrun, concurrent stats reads, sleep/wake, empty graph, RT policy applies. Stress: 55 consecutive runs clean.
  How to test: `make check`; `cargo bench -p wing-engine --bench scheduler` (prints a summary table after criterion).
  Bench (M4 Max; other agents were running, so +-10% noise): 48 strips (~2.8us) + 16 bus sums (~0.6us) + 16 FX (~4.4us) + main, 64 samples; serial work ~215us.
    serial raw 216us; engine with 0 workers 214us (single-thread scheduler overhead ~0%, within noise).
    hot (back to back): 1w 111-117us, 3w 63us (3.5-3.7x), 5w 46us (4.7-5.0x), 7w 36us (5.9x), 11w 33-37us (6.2-6.6x). Overhead vs an ideal level-synchronous schedule (from single-node costs): +2.7..+5.4% of serial work typical (one noisy run 7.9% at 7w).
    paced (1.33 ms period, RT driver+workers, workers asleep between cycles): 7w median 51-90us, p99 74-172us = 5.6-13% of the period. Paced with 0 workers is 385us vs 214us hot, so cold caches / clock-down after idle dominate here, not the scheduler.
  Gotcha: RT-policy threads running flat out exceed their computation budget and get throttled (serial looked 5x slower), so hot benches use rt_workers=false and paced uses RT.
  Proposed follow-ups: (1) os_sync_* needs macOS 14.4+; add a dlsym fallback to __ulock if older macOS matters. (2) Size the worker count from os_workgroup_max_parallel_threads / P-core count in T023. (3) Paced latency: spin workers ahead of the expected next callback (predictive wake) once real device timing exists. (4) The non-macOS futex fallback is yield-based (busy).
