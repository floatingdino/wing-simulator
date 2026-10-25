---
id: T081
title: BUG: Soul Warmth Pre drive ineffective + audit every model's parameter binding
stage: S4
status: todo
owner: -
depends: []
branch: -
---

## Goal
User: WARM (Soul Warmth Pre) drive knob appears ineffective. Fix it, then add a systematic audit test so this class of bug can't recur: for EVERY model in every block kind (filter-tool, gate, dyn, EQ, FX), drive the parameter path exactly as WING-Edit does (schema path set via ParamStore → params pipeline → ModelSlot → model.set_param slot index), sweep each parameter across its range, and assert the output changes measurably (or the param is documented as having no audible effect, e.g. a display-only key). Report any params that don't reach the DSP (slot-index mismatches, wrong scaling, enum mapping) and fix them.

## Acceptance criteria
- [ ] WARM drive audibly changes output (test)
- [ ] Audit test covers all models/params; all dead params fixed or justified
- [ ] Report in task Log

## Log
- 2026-10-26 coordinator: created from user report.
