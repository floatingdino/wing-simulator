---
id: T015
title: Meter transport (synthetic)
stage: S1
status: done
owner: agent-T013-T015
depends: [T013]
branch: task/T013-T015
---

## Goal
Implement meter subscription and streaming per protocol.md with synthetic data so WING-Edit meters move; later fed by T039.

## Acceptance criteria
- [ ] WING-Edit meters animate from simulator

## Log
- 2026-10-25 coordinator: created.
- 2026-10-25 agent (T013+T015, branch task/T013-T015): `crates/wing-proto/src/server/meters.rs`: `MeterLayout` (counts from the schema; source/output group sizes from io/in and io/out [H]), `MeterBank` (one relaxed AtomicU16 per word, so T039 writes are lock-free and allocation-free), `MeterSubscriptions` (per-client port and reports, 5 s lifetime after define/renew, removed on disconnect), `MeterStreamer` (every 50 ms, `id.l + i16[]`, capped at WING-Edit's 0xFC8 rx buffer, sent from UDP 2222), `SyntheticMeters` (slow per-strip sines; outputs follow fader and mute). Report 0 = 371 words / 746 B. Unit and TCP tests pass; a smoke test against the daemon received packets from :2222. Not yet seen animating in WING-Edit (it did not reconnect, see docs/milestones/v0.1.md).
