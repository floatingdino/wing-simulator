---
id: T082
title: BUG: metering for FX blocks and non-default dynamics models doesn't work
stage: S4
status: todo
owner: -
depends: []
branch: -
---

## Goal
User: "FX block / non-default dynamics models monitoring largely doesn't work". Make WING-Edit's meters work for: gain-reduction + key meters of every gate/dyn model (not only GATE/DUCK/COMP/EXP), dynamic-EQ band activity if WING reports it, FX slot input/output meters and any model-specific meters WING-Edit requests (check the meter request sets in docs/research/wing-edit-session.md / protocol.md and what WING-Edit asks for when an FX or dyn page is open — capture from the user's live log scratch/wing-sim-v0.4.log or later logs: ch3 meter report definitions). Wire telemetry from every Model into LevelBank. Tests per model family.

## Acceptance criteria
- [ ] GR meter moves for every dyn/gate model under compression (test)
- [ ] FX in/out meters populated; any FX-specific meters WING-Edit requests populated or documented
- [ ] Zero RT alloc

## Log
- 2026-10-26 coordinator: created from user report.
