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
