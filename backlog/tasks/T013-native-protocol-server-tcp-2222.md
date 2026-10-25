---
id: T013
title: Native protocol server (TCP 2222)
stage: S1
status: review
owner: agent-T013-T015
depends: [T010, T012]
branch: task/T013-T015
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
- 2026-10-25 agent (T013+T015): implemented `crates/wing-proto/src/server/`: `session.rs` (pure per-connection protocol state: cursor/hash navigation with hash-0/unknown invalidation, sets via `values.rs` in every token form with enums by name, gets answered `D7 h v DE`, `DD` definition records from `defs.rs` plus `$songs`/`$scenes` lists via `LiveInfo`, ModelRequest answered `D7 985729a2 "wing-fullsize"` [H], `DA DC` root dump [H]); `mod.rs` (`Hub`: ParamStore hook with a per-client coalescing outbox and no echo, coexists with the T014 OSC hook; `NativeServer`: one thread per client, max 24, 10 s idle timeout, clean stop). `apps/wing-sim` is now a daemon: discovery + native + meters + OSC over one store; flags --name/--ip/--fw/--model/--serial/--scene (TODO T011)/--idle-timeout/--no-meters/--no-osc/--port/--osc-port; SIGINT/SIGTERM clean shutdown. Added `Responder::try_clone_socket` so meters are sent from UDP 2222. Merged main (T014).
  Tests: `crates/wing-proto/tests/server_replay.rs`. A1 replay: 29,914 sets, 1,392 hash-0 dropped, 0 rejected, 25 gets, 2 DD, store == pc-to-mixer-scene.json for all params. B replay: a reply per get, the model reply, DEs. TCP: native/native/OSC fan-out, UDP meters, idle timeout, max clients, shutdown. `make check` passes.
  Live: discovery OK (WING-Edit lists WINGSIM). WING-Edit did NOT reconnect: it shows a "Lost connection! Auto reconnect is active." modal from an earlier session and needs a click (not allowed). Details and the human checklist are in docs/milestones/v0.1.md.
  Proposed follow-ups: human-run connect check (T016); confirm the ModelRequest/DD/`DA DC` reply formats against WING-Edit; per-param read-only flag (T009 note).
