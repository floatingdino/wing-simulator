---
id: T024
title: I/O routing: sources, USB alt inputs, output patching
stage: S2
status: in_progress
owner: agent-T024
depends: [T010, T022]
branch: task/T024
---

## Goal
Implement WING io model from snapshot (io/in, io/out, altsw, channel main/alt sources) so USB inputs feed channels exactly as the scene defines, and output patches (incl. USB out 1-48) take the right signals.

## Acceptance criteria
- [ ] Scene routing tests from PUBLIC.snap
- [ ] altsw toggles main/alt sources

## Log
- 2026-10-25 coordinator: created.
- 2026-10-25 agent-T024: started. Plan: `crates/wing-engine/src/routing/` (source address space, tap enum,
  table builder from ParamStore, lock-free table swap, oscillator + input/output router processors); tests from
  fixtures/PUBLIC.snap via a test-only JSON loader (T011 not on main yet).
