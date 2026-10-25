---
id: T018
title: Native protocol codec (framing, escaping, channels, tokens)
stage: S1
<<<<<<< HEAD
status: done
owner: -
=======
status: done
owner: agent-T012-T018
>>>>>>> task/T012-T018
depends: [T001, T002]
branch: task/T012-T018
---

## Goal
In crates/wing-proto implement a pure, allocation-light encoder/decoder for the WING native binary protocol per docs/research/protocol.md: DF-escaped channel multiplexing (DF D0..DD), channel-1 tokens (node hash D7, get/set/navigate, value encodings for int/float/string/raw, node definition records), channel-3 meter requests, and UDP meter report packets. Streaming decoder that handles partial reads. Reuse/port from libwing (Apache-2.0; keep NOTICE/attribution) where sensible.

## Acceptance criteria
- [x] Round-trip property tests (encode→decode) for every token type
- [x] Decoder tests with captured/libwing-derived byte sequences
- [x] Fuzz-ish test: random chunking of a stream decodes identically

## Log
- 2026-10-25 coordinator: created (split out of T013 so it can start before the schema lands).
- 2026-10-25 agent: done, status review. Files: crates/wing-proto/src/nrp/{mod,framing,token,nodedef,meter,queue}.rs,
  crates/wing-proto/NOTICE (libwing Apache-2.0 attribution; derived files carry header comments),
  tests/nrp_vectors.rs, tests/nrp_roundtrip.rs.
  - framing: `Demux` implements the Behringer rx routine exactly (DF DF stays escaped, DF xx literal, pre-select bytes
    dropped and counted); `Mux` emits `DF Dx` only on change and always escapes payload DF as `DF DE`.
  - token: zero-copy `Token<'a>` (all of 00-DF), `parse_token` (Ok(None)=incomplete, Err=skip 1 byte), streaming
    `TokenDecoder` (reused buffer), canonical encoder (`Token::int`, shortest string form). D2 index uses libwing's
    `u16+1` convention (NavIndex 1..=65536). `DF` records: u16 len or `0000`+u32 len, capped at 1 MiB.
  - nodedef: owned `NodeDef` parse/encode for all 8 types, unknown flag bits preserved (byte-exact round trip).
  - meter: `MeterToken`, `MeterTokenDecoder`, `MeterRequestParser` -> `MeterCommand::{SetPort, Renew(id) for every D4,
    Define{id, selections}}`, u32 report ids, UDP `MeterPacket` parse/encode (+ fixed-buffer variant), 1/256 dB helpers.
  - `NrpDecoder` (feed raw TCP bytes, drain ch1/ch3) and `NrpEncoder`.
  Tests: doc byte examples (spec 2.2, 3.2, 3.3, 4.1); 11 raw libwing propmap records (every node type, RO flags, a
  DF-containing hash) parsed and re-encoded byte-exact; libwing client byte sequences (set_string/int/float,
  request_meter + renew); 20k random-token round trips covering every token type (every prefix reports incomplete);
  5k random NodeDef round trips; meter token/packet round trips; 50 random interleaved ch1/ch3 streams decoded under
  random chunking (1..300 bytes) identical to whole-buffer decode; random garbage never panics and chunks identically.
  How to test: `make check` (passes). libwing's propmap dataset is NOT vendored (schema task).
