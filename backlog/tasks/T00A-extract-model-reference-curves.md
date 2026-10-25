---
id: T00A
title: Extract per-model response/gain-computer curves from WING-Edit
stage: S0
status: in_progress
owner: -
depends: [T003]
branch: -
---

## Goal
T003 found WING-Edit contains per-model response-curve and gain-computer functions (used for its EQ/dyn graphs). Using the same Unicorn emulation approach (tools/schema-extract/wingemu.py; never run the app natively or modify it), call these functions over parameter sweeps and export reference data: EQ magnitude curves per model/band type, dynamics/gate static gain curves per model, filter curves. Output data/reference-curves/*.json (+ generator script in tools/curve-extract/) and docs/research/reference-curves.md. These become fidelity test oracles for S3/S4.

## Acceptance criteria
- [ ] Curves for every EQ/filter/gate/dyn model at representative parameter grids
- [ ] Documented function list and calling convention

## Log
- 2026-10-25 coordinator: created from T003 surprise finding.
