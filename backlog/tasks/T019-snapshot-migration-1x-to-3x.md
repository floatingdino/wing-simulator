---
id: T019
title: Snapshot migration: firmware 1.x keys -> 3.x schema
stage: S1
status: todo
owner: -
depends: [T011]
branch: -
---

## Goal
fixtures/PUBLIC.snap (fw 1.13.9) has 1,709 keys absent from the 3.x schema (largest group: per-send ind/wid, 1,536 keys). Build a migration map so 1.x scenes load with correct mix intent, and document lossy cases. Exporting must write the 3.x format.

## Acceptance criteria
- [ ] PUBLIC.snap loads with 0 unexplained unmapped keys
- [ ] Mapping table documented with rationale per key family

## Log
- 2026-10-25 coordinator: created from T003 coverage report.
