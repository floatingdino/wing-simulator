---
id: T002
title: Research: WING discovery, native binary protocol, OSC, metering
stage: S0
status: todo
owner: -
depends: []
branch: -
---

## Goal
Produce docs/research/protocol.md: a complete implementable spec of how WING-Edit 3.3.4 finds and talks to a console. Sources: Behringer 'WING Remote Protocols' doc, Patrick-Gilles Maillot's WING docs/tools, open-source WING libraries (search GitHub/crates/PyPI), and observation of the running WING-Edit (e.g. bind a UDP listener on 2222 to capture its discovery probe; non-root only). Cover: UDP 2222 discovery request/response format and fields WING-Edit validates (model, fw version, serial, name), TCP 2222 native protocol framing/escaping/channels, node addressing (hash ids vs paths), get/set/subscribe semantics, node definition queries, meter subscription transport (UDP?) and formats, keepalive/timeouts, OSC 2223 address space, and firmware-version compatibility expectations of WING-Edit 3.3.4. List every reusable library with license.

## Acceptance criteria
- [ ] protocol.md has byte-level message formats with examples
- [ ] Includes a captured WING-Edit discovery probe hexdump (or documented reason it could not be captured)
- [ ] Lists open questions that need T003 (binary analysis) to resolve
- [ ] Reusable libraries table (name, lang, license, coverage)

## Log
- 2026-10-25 coordinator: created.
