---
id: T012
title: Discovery responder (UDP 2222)
stage: S1
status: in_progress
owner: -
depends: [T001, T002]
branch: -
---

## Goal
Answer WING-Edit discovery so the simulator appears in WING-Edit's console list with configurable name/serial/model/fw. Support binding to a chosen interface/IP (and loopback alias if WING-Edit needs a non-127 address — document).

## Acceptance criteria
- [ ] WING-Edit lists 'Wing Simulator' (manual/screenshot check logged)
- [ ] Unit test for packet format

## Log
- 2026-10-25 coordinator: created.
