---
id: T003
title: Research: WING-Edit binary analysis -> parameter schema
stage: S0
status: todo
owner: -
depends: []
branch: -
---

## Goal
Statically analyse WING-Edit.app (Mach-O, read-only; do not modify or re-sign it) to extract the complete parameter tree the app knows: node paths, node ids/hashes, types (float/int/enum/string/bool), ranges, units, tapers/steps, enum value lists (incl. every model name for filter, gate, EQ, dynamics, insert and FX blocks and each model's parameters), and any protocol constants. Tools: strings, otool/nm, python (lief/capstone if pip-installable into a venv in scratch). Output data/schema/wing-tree.json (machine readable, documented format) + docs/research/wing-edit-schema.md (method, coverage, gaps). Cross-check against fixtures/PUBLIC.snap keys: report schema coverage % of snapshot keys.

## Acceptance criteria
- [ ] data/schema/wing-tree.json exists with documented format
- [ ] Every model enum for every processing block listed with its parameter list
- [ ] Coverage report vs fixtures/PUBLIC.snap keys (>=95% target, gaps listed)
- [ ] Reproducible extraction script in tools/schema-extract/

## Log
- 2026-10-25 coordinator: created.
