---
id: T04F
title: ONEC + 3CMP multiband
stage: S4
status: done
owner: claude-agent
depends: [T045, T034]
branch: task/T04F
---

## Goal
One-knob comp and three-band comp per models.md §3.2.

## Acceptance criteria
- [x] Crossover sum-flat test, GR tests

## Log
- 2026-10-25 coordinator: created by decomposing epics T040-T044/T051 after T004.
- 2026-10-25 agent: ONEC + 3CMP implemented in `crates/wing-blocks/src/models/comp_multi.rs` (ONEC = `onec_map` 1-D mapping onto the standard COMP `Dynamics` engine; 3CMP = LR4 3-way split with allpass-compensated low band, 3 COMP engines, band gains). Registered in model.rs; bench `benches/comp_multi.rs`; zero-alloc test `tests/comp_multi_no_alloc.rs`; unit tests: crossover sum flat +-0.05 dB (3 crossover settings x 14 freqs), per-band GR isolation, band gain, ONEC mapping monotonic, GR grows with knob, dag make-up. model.rs stub tests now use `PSE` as the stub example. Assumptions in docs/VERIFY-ON-HARDWARE.md. WING's `wingcoef::xover` not used (LP/HP halves are different designs and do not sum flat). Test: `make check`; `cargo bench -p wing-blocks --bench comp_multi`.
