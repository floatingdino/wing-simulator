---
id: T013
title: Native protocol server (TCP 2222)
stage: S1
status: todo
owner: -
depends: [T010, T012]
branch: -
---

## Goal
Implement the native binary protocol server: handshake, node reads/writes, tree dumps, subscriptions/notifications, multiple clients, keepalive. Mutations go to the parameter store and fan out to other clients.

## Acceptance criteria
- [ ] WING-Edit connects, syncs, and shows the loaded scene
- [ ] Edits in WING-Edit reflect in store (logged) and survive reconnect
- [ ] Protocol unit tests with recorded frames

## Log
- 2026-10-25 coordinator: created.
- 2026-10-25 coordinator: from T009 — libwing has 2,886 records absent from our tree (mostly $ctl/layer/*/$spill strips, likely created at runtime by CcontrolEngine); schema lacks a per-param read-only flag and cnslock maxlen is wrong (0 vs 19). Handle these when serving node definitions. Use docs/research/wing-edit-session.md (T00B) for the connect sequence.
