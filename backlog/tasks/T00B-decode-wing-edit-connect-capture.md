---
id: T00B
title: Decode WING-Edit connect capture -> session spec + test fixtures
stage: S1
status: todo
owner: -
depends: [T018, T003]
branch: -
---

## Goal
tools/protocol-probe/captures/2026-10-25-wing-edit-connect-end-mode.txt.gz is a live capture of WING-Edit 3.3.4 connecting 3 times to the fake console (tcp-mode 'end'); on one connect it pushed the full scene (PC -> Mixer) as `D7 <hash> <value>` sets. Decode it with the wing-proto codec (crates/wing-proto/src/nrp) and the schema (data/schema/wing-tree.json + tools/schema-extract/winghash.py): connection handshake (ch3 `d3 f0 07`, meter/`dc` requests), which channels/tokens are used, value encodings per param type, ordering, how many params pushed vs schema, why each connection ended (timeouts, missing replies), keepalive behaviour. Map every pushed hash to a path and value; report unmapped hashes. Produce: docs/research/wing-edit-session.md (precise sequence diagram + what a server must reply at each step, answering the open questions in protocol.md where possible), fixtures/protocol/wing-edit-connect-*.bin (raw per-connection client byte streams) and fixtures/protocol/pc-to-mixer-scene.json (path -> value) for T013 tests.

## Acceptance criteria
- [ ] Every client message in the capture is explained
- [ ] >=99% of pushed hashes mapped to schema paths
- [ ] Clear list of server responses needed to keep WING-Edit connected

## Log
- 2026-10-25 coordinator: created after the user ran the PC->Mixer connect.
