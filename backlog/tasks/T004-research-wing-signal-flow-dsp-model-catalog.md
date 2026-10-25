---
id: T004
title: Research: WING signal flow + DSP model catalog
stage: S0
status: todo
owner: -
depends: []
branch: -
---

## Goal
Produce docs/research/signal-flow.md (exact processing order and tap points for channel, aux, bus, main, matrix strips; input routing incl. sources, alt sources/USB, altsw; send taps; insert points; DCA/mute groups; solo/monitor; FX rack insert vs send; sample rate/latency) and docs/research/models.md: for EVERY block (filter/filter utility, gain, gate/expander, EQ, dynamics, inserts, FX rack) every alternate model: name in UI and in snapshot, the hardware it emulates, parameter list + ranges + units, and behavioural notes needed to emulate it (topology, ratio/knee/time constants, curves). Sources: WING manual(s), Behringer effects guides, firmware release notes, snapshot fixtures/PUBLIC.snap, and data/schema output of T003 if present.

## Acceptance criteria
- [ ] signal-flow.md has a diagram per strip type with every tap point
- [ ] models.md lists every model per block with params (cross-referenced to snapshot keys)
- [ ] Each model has an 'emulation approach' line and difficulty estimate (S/M/L)

## Log
- 2026-10-25 coordinator: created.
