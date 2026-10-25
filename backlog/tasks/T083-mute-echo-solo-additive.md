---
id: T083
title: BUG: mute toggles don't visually update in WING-Edit; solos exclusive instead of additive
stage: S3
status: todo
owner: -
depends: []
branch: -
---

## Goal
User: (1) mute toggles on channels/buses don't visually update in WING-Edit (audio mutes, UI button doesn't toggle) — likely the server doesn't echo/confirm the new value to the originating client (we fan out only to other clients), or WING-Edit sends a toggle value / expects an echo for some node types. Determine from protocol docs/captures what a real console replies (fixtures/protocol, docs/research/wing-edit-session.md, libwing), and implement (echo to sender for all sets, or for those node types). Check the same for other buttons (solo, on/off switches, enums). (2) Solos behave exclusive; on a WING they're additive by default — check cfg/solo exclusive/"excl" key semantics and our default; make additive the behaviour unless the scene's exclusive setting is on. Regression tests with the native protocol client.

## Acceptance criteria
- [ ] WING-Edit-style client sees its own mute/solo toggles confirmed (test)
- [ ] Solo additive by default, exclusive when the scene says so (test)

## Log
- 2026-10-26 coordinator: created from user report.
