---
id: T014
title: OSC server (UDP 2223)
stage: S1
status: done
owner: T014-agent
depends: [T010]
branch: task/T014
---

## Goal
OSC get/set/subscribe for the full tree, matching real WING OSC semantics, for scripting and DAW control.

## Acceptance criteria
- [ ] oscsend/python test can get and set e.g. /ch/1/fdr
- [ ] Matches protocol.md semantics

## Log
- 2026-10-25 coordinator: created.
- 2026-10-25 T014-agent: implemented `crates/wing-proto/src/osc/{codec,service,server}.rs`, example `examples/osc_server.rs`, tests `tests/osc_loopback.rs` + unit tests. `make check` passes. Run: `cargo run -p wing-proto --example osc_server -- --bind 127.0.0.1:2223`.
  - Codec hand-rolled (no rosc): tiny subset, permissive (missing type tags, no bundles), zero deps. wing-proto now depends on wing-model.
  - Semantics (also in service.rs docs): `,f` set = engineering value, `,i` = int/enum index, `,s` = text/enum name; get leaf = `,sff`/`,sfi` (text, norm, value), enum/string `,s` (enum token); node get lists child names; node set `/ch/1 ,s "fdr=3"` and root `/ ,s "/ch.1.fdr=-1,mute=0,.2.fdr=0"` with `OK` ack at `<node>*` and documented error strings; `*`/`?`/`#` dumps (`?`/`#` text format unverified); `/#hash` addressing; `/%port` reply redirect; `/?` info.
  - Subscriptions `/*s` (triplets), `/*S` (single native arg), `/*b` (blob `D7 hash value` at `/`): one subscriber, last wins, configurable expiry (default 10 s), renew by resending; subscriber's own sets not echoed. Hook installed via `OscAttachment::attach(&mut store)` before sharing the store.
  - Not done / follow-ups: `,b` native passthrough (`/ ,b d7...`); `$`-shadow node events; unsolicited `/*` error events; exact console text for `?`/`#`; hash-in-middle addresses; interop test with companion-module-behringer-wing.
