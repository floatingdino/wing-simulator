---
id: T018
title: Native protocol codec (framing, escaping, channels, tokens)
stage: S1
status: in_progress
owner: -
depends: [T001, T002]
branch: -
---

## Goal
In crates/wing-proto implement a pure, allocation-light encoder/decoder for the WING native binary protocol per docs/research/protocol.md: DF-escaped channel multiplexing (DF D0..DD), channel-1 tokens (node hash D7, get/set/navigate, value encodings for int/float/string/raw, node definition records), channel-3 meter requests, and UDP meter report packets. Streaming decoder that handles partial reads. Reuse/port from libwing (Apache-2.0; keep NOTICE/attribution) where sensible.

## Acceptance criteria
- [ ] Round-trip property tests (encode→decode) for every token type
- [ ] Decoder tests with captured/libwing-derived byte sequences
- [ ] Fuzz-ish test: random chunking of a stream decodes identically

## Log
- 2026-10-25 coordinator: created (split out of T013 so it can start before the schema lands).
