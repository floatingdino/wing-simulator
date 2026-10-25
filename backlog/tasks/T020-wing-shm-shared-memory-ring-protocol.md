---
id: T020
title: wing-shm: shared-memory ring protocol
stage: S2
status: todo
owner: -
depends: [T001, T006]
branch: -
---

## Goal
Productionize the driver<->daemon shared-memory layout: versioned header, 48x48 planar float rings, timestamps/sample-time, heartbeat, xrun counters, C header generated/kept in sync with Rust.

## Acceptance criteria
- [ ] Rust + C++ tests exchanging audio across processes
- [ ] Lock-free, wait-free on RT side

## Log
- 2026-10-25 coordinator: created.
