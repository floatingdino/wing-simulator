---
id: T024
title: I/O routing: sources, USB alt inputs, output patching
stage: S2
status: todo
owner: -
depends: [T010, T022]
branch: -
---

## Goal
Implement WING io model from snapshot (io/in, io/out, altsw, channel main/alt sources) so USB inputs feed channels exactly as the scene defines, and output patches (incl. USB out 1-48) take the right signals.

## Acceptance criteria
- [ ] Scene routing tests from PUBLIC.snap
- [ ] altsw toggles main/alt sources

## Log
- 2026-10-25 coordinator: created.
