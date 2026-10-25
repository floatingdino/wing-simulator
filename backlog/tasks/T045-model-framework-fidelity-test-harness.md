---
id: T045
title: Model framework + fidelity test harness
stage: S4
status: todo
owner: -
depends: [T030, T010]
branch: -
---

## Goal
In crates/wing-blocks: Model trait (prepare(sr,max_block), process(stereo/mono, sidechain), latency(), set_param by schema slot, reset), per-block model registries (filter/gate-dyn/eq/fx) with preallocated instances so model switches never allocate on the RT thread; stereo-link; a fidelity harness that measures magnitude response / static gain curves / time constants and compares to data/reference-curves (T00A) with tolerances.

## Acceptance criteria
- [ ] Trait + registry used by T031-T034 standard models (port them)
- [ ] Harness runs against at least one reference curve file

## Log
- 2026-10-25 coordinator: created by decomposing epics T040-T044/T051 after T004.
