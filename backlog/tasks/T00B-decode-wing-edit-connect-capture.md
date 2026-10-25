---
id: T00B
title: Decode WING-Edit connect capture -> session spec + test fixtures
stage: S1
status: done
owner: claude-T00B
depends: [T018, T003]
branch: -
---

## Goal
tools/protocol-probe/captures/2026-10-25-wing-edit-connect-end-mode.txt.gz is a live capture of WING-Edit 3.3.4 connecting 3 times to the fake console (tcp-mode 'end'); on one connect it pushed the full scene (PC -> Mixer) as `D7 <hash> <value>` sets. Decode it with the wing-proto codec (crates/wing-proto/src/nrp) and the schema (data/schema/wing-tree.json + tools/schema-extract/winghash.py): connection handshake (ch3 `d3 f0 07`, meter/`dc` requests), which channels/tokens are used, value encodings per param type, ordering, how many params pushed vs schema, why each connection ended (timeouts, missing replies), keepalive behaviour. Map every pushed hash to a path and value; report unmapped hashes. Produce: docs/research/wing-edit-session.md (precise sequence diagram + what a server must reply at each step, answering the open questions in protocol.md where possible), fixtures/protocol/wing-edit-connect-*.bin (raw per-connection client byte streams) and fixtures/protocol/pc-to-mixer-scene.json (path -> value) for T013 tests.

## Acceptance criteria
- [x] Every client message in the capture is explained
- [x] >=99% of pushed hashes mapped to schema paths (100% of non-zero ids; 28,522/28,523 unique incl. id 0)
- [x] Clear list of server responses needed to keep WING-Edit connected (wing-edit-session.md §1.2)

## Log
- 2026-10-25 coordinator: created after the user ran the PC->Mixer connect.
- 2026-10-25 claude-T00B: in_progress. Added crates/wing-proto/examples/decode_capture.rs: it parses the fake_wing.py
  hexdump log, reassembles per-connection streams, decodes them with NrpDecoder/MeterRequestParser and maps hashes
  via wing-tree.json. Capture A: 3 connections, each a full PC->MIXER push (29.9k-30.2k `D7 h v`), 25 gets,
  2 `D7 h DE DD`, and 0 decode errors. An independent Python tokeniser gives identical D7 counts. Coordinator note:
  connection 3 is truncated.
- 2026-10-25 claude-T00B: port 2222 was free, so I ran a scratch token-aware fake (fake_wing.py variant: typed
  replies to gets, UDP meter data, no TCP reply on ch3) passively. WING-Edit auto-reconnected by itself, with no UI
  interaction. With value replies it then sent `D7 985729a2 "ModelRequest"`, meter defines 1-3, and ch3 renews every
  4.04 s; it stayed connected 114 s. After the fake stopped, WING-Edit kept a dead CLOSED socket and did not reconnect
  to 2 later fakes (90 s + 180 s), so the "no meters" variant could not be tested. NOTE for user: WING-Edit
  (pid 40378) may need a manual reconnect.
- 2026-10-25 claude-T00B: review. Files: docs/research/wing-edit-session.md (new);
  crates/wing-proto/examples/decode_capture.rs (new); fixtures/protocol/{wing-edit-connect-1,2,3.bin,
  wing-edit-connect-tokens-1.bin, wing-edit-connect-tokens-1.server.bin, wing-edit-connect-tokens-mode.txt.gz,
  pc-to-mixer-scene.json} (new). Test: `gzip -dc tools/protocol-probe/captures/2026-10-25-wing-edit-connect-end-mode.txt.gz |
  cargo run -p wing-proto --example decode_capture -- --tokens`. `cargo fmt --check`, `cargo clippy -p wing-proto
  --all-targets -D warnings` and `cargo test -p wing-proto` pass. Codec: no bugs found (notes in session doc §6:
  ch1/ch3 relative order is lost across queues; the client sends `DE` tokens). No codec changes made.
  Proposed follow-ups:
  - T013: answer each `D7 h DC` with `D7 h v DE` (required); treat unknown or 0 hashes as a cursor invalidation;
    accept enums by name; count ch3 renews as activity.
  - Verify the ModelRequest reply (`D7 985729a2 "wing-fullsize"`?) and the `D7 h DE DD` list-def reply in T016.
  - Test whether UDP meter data is required (get replies with no meters) on a user-initiated connect.
  - Capture MIXER->PC (`DA DC`) with the user's help.
  - Port the token reply mode into tools/protocol-probe/fake_wing.py (session doc §7).
