---
id: T002
title: Research: WING discovery, native binary protocol, OSC, metering
stage: S0
status: done
owner: protocol-research-agent
depends: []
branch: -
---

## Goal
Produce docs/research/protocol.md: a complete implementable spec of how WING-Edit 3.3.4 finds and talks to a console. Sources: Behringer 'WING Remote Protocols' doc, Patrick-Gilles Maillot's WING docs/tools, open-source WING libraries (search GitHub/crates/PyPI), and observation of the running WING-Edit (e.g. bind a UDP listener on 2222 to capture its discovery probe; non-root only). Cover: UDP 2222 discovery request/response format and fields WING-Edit validates (model, fw version, serial, name), TCP 2222 native protocol framing/escaping/channels, node addressing (hash ids vs paths), get/set/subscribe semantics, node definition queries, meter subscription transport (UDP?) and formats, keepalive/timeouts, OSC 2223 address space, and firmware-version compatibility expectations of WING-Edit 3.3.4. List every reusable library with license.

## Acceptance criteria
- [x] protocol.md has byte-level message formats with examples
- [x] Includes a captured WING-Edit discovery probe hexdump (or documented reason it could not be captured)
- [x] Lists open questions that need T003 (binary analysis) to resolve
- [x] Reusable libraries table (name, lang, license, coverage)

## Log
- 2026-10-25 coordinator: created.
- 2026-10-25 protocol-research-agent: started; plan = live discovery capture (tools/protocol-probe/) + web/library source research -> docs/research/protocol.md.
- 2026-10-25 protocol-research-agent: DONE -> review.
  Files: docs/research/protocol.md (new); tools/protocol-probe/capture_discovery.py (passive UDP 2222 capture w/ dst addr),
  tools/protocol-probe/fake_wing.py (discovery/OSC responder + TCP 2222 logger, modes silent|end),
  tools/protocol-probe/captures/2026-10-25-wing-edit-3.3.4-discovery.txt.
  Live: WING-Edit 3.3.4 sends 'WING?' (57 49 4e 47 3f) every 2 s to subnet bcast + 255.255.255.255; accepts our CSV reply
  (lists name/ip/fw; uses the reply's ip field; ngc-full shown as "WING"; no fw check at discovery).
  Not captured: first TCP bytes (WING-Edit does not auto-connect; clicking CONNECT on the user's live session was avoided —
  manual procedure in protocol.md section 7). Side effect: stale "ALTNAME 10.9.9.9" entry in WING-Edit console list (RESCAN clears).
  How to test: `python3 tools/protocol-probe/capture_discovery.py --seconds 10` while WING-Edit is open.
  Proposed follow-ups: (1) human-assisted TCP capture per protocol.md section 7; (2) import libwing src/propmap.rs raw node defs
  (Apache-2.0, 60,748 records, fw 3.0.5) as seed schema for T010 after T003 reconciliation; (3) optional Windows VM running
  Maillot's wingemul as a differential-test oracle.
