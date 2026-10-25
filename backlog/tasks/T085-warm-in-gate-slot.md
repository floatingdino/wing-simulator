---
id: T085
title: BUG: Soul Warmth Pre drive inaudible in the GATE slot (VOX channels)
stage: S4
status: todo
owner: -
depends: [T081]
branch: -
---

## Goal
User (v0.5 build): WARM drive still inaudible when WARM is loaded in the gate slot on VOX channels (e.g. ch 21). Live log scratch/wing-sim-v0.5.log shows `ch/21/gate/1 = 125 %` etc. T081 only tested the dyn slot (`ch/7/dyn/1`). Reproduce exactly via `ch/21/gate/mdl = WARM` + `ch/21/gate/1..n` through the full pipeline, fix the gate-slot binding/processing (and check every model is processed correctly when hosted in the gate slot vs dyn slot — both slots offer the same 32 models), extend the T081 param audit to cover both slots for every dynamics model.

## Acceptance criteria
- [ ] Regression test fails before / passes after
- [ ] Param audit covers gate AND dyn slot for all 32 models

## Log
- 2026-10-26 coordinator: created from user v0.5 test.
