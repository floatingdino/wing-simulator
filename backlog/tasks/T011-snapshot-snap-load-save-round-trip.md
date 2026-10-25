---
id: T011
title: Snapshot (.snap) load/save round-trip
stage: S1
status: todo
owner: -
depends: [T010]
branch: -
---

## Goal
Load fixtures/PUBLIC.snap into the store and save it back. Semantic round-trip equality; preserve unknown keys verbatim; write format accepted by a real WING (same type/version fields).

## Acceptance criteria
- [ ] Round-trip test on PUBLIC.snap: JSON semantically equal
- [ ] Unknown keys preserved
- [ ] Loader reports unmapped keys

## Log
- 2026-10-25 coordinator: created.
