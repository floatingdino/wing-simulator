---
id: T010
title: wing-model: generated parameter tree
stage: S1
status: todo
owner: -
depends: [T001, T003]
branch: -
---

## Goal
Codegen (build.rs or tool) from data/schema/wing-tree.json into a flat, index-addressed parameter store (Vec of atomics/f32 + metadata table) with path<->id<->hash lookup, defaults, clamping/quantisation, enum mapping and change notification queue.

## Acceptance criteria
- [ ] All schema nodes addressable by path/hash
- [ ] Unit tests for range/enum/taper conversions
- [ ] Lookup O(1) and allocation-free get/set

## Log
- 2026-10-25 coordinator: created.
