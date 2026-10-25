---
id: T004
title: Research: WING signal flow + DSP model catalog
stage: S0
status: done
owner: research-agent-T004
depends: []
branch: -
---

## Goal
Produce docs/research/signal-flow.md (exact processing order and tap points for channel, aux, bus, main, matrix strips; input routing incl. sources, alt sources/USB, altsw; send taps; insert points; DCA/mute groups; solo/monitor; FX rack insert vs send; sample rate/latency) and docs/research/models.md: for EVERY block (filter/filter utility, gain, gate/expander, EQ, dynamics, inserts, FX rack) every alternate model: name in UI and in snapshot, the hardware it emulates, parameter list + ranges + units, and behavioural notes needed to emulate it (topology, ratio/knee/time constants, curves). Sources: WING manual(s), Behringer effects guides, firmware release notes, snapshot fixtures/PUBLIC.snap, and data/schema output of T003 if present.

## Acceptance criteria
- [x] signal-flow.md has a diagram per strip type with every tap point
- [x] models.md lists every model per block with params (cross-referenced to snapshot keys)
- [x] Each model has an 'emulation approach' line and difficulty estimate (S/M/L)

## Log
- 2026-10-25 coordinator: created.
- 2026-10-25 research-agent: started; inspecting fixtures/PUBLIC.snap.
- 2026-10-25 research-agent: extracted model tables + per-model parameter definitions from WING-Edit 3.3.4 (x86_64
  slice, otool disassembly of `ae::*_pardef_*` functions; scripts in session scratchpad, not committed - T003 owns
  the committed extractor). Cross-checked against fixtures/PUBLIC.snap (fw 1.13.9). Web: fw 3.1 review (Drew
  Brashler); official manual/remote-protocol PDFs unreachable (image-only / HTTP 500).
- 2026-10-25 research-agent: DONE -> review. Files: docs/research/signal-flow.md, docs/research/models.md (generated
  tables + hand notes). Counts: filter tool 4, gate+dyn 33 (13 gate-family, 20 comp-family), EQ 9 distinct (ch 7,
  aux 6, bus/main/mtx 7), FX 64 + NONE (26 restricted to FX1-8). T003 outputs appeared before finish and were
  cross-checked programmatically: model IDs identical, gate/dyn/EQ keys+ranges identical; T003 adds EQ params of
  EQ-type FX/strips and GEQ bands (merged into models.md, marked "from T003"). Details: models.md section 9.
  How to check: open both docs; every model in models.md has emulation approach + difficulty; tables list keys
  identical to snapshot keys.
  Proposed follow-ups: (1) measure/obtain knob->Hz/dB maps for vintage EQ/comp models; (2) verify open questions in
  signal-flow.md section 13 on a console or the manual; (3) T011 snapshot loader aliases (WARM mix->wmix, aux dyn
  default mdl CMB, out-delay m->dly, default lcs/hcs); (4) drive WING-Edit's `*_respf_*`/`*_gain_*`/`*_parchg_*` functions (T003 emulator) to get knob->unit maps and
  static curves for vintage models; (5) T003 numeric-key collisions (`31`, `40`) in variants.
