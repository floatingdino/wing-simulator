---
id: T04J
title: EQ mix law for all EQ models + 44.1 kHz coefficient verification
stage: S4
status: todo
owner: -
depends: [T04G, T04H, T03C]
branch: -
---

## Goal
WING's EQ `mix` scales band gains (T03C finding), but the model slot host applies dry/wet for alternate EQ models. Apply the gain-scale mix law uniformly to STD and every alternate EQ (incl. FX-rack EQ variants later), and verify all EQ/filter coefficient code at 44.1 kHz by driving the WING-Edit emulation (tools/curve-extract) at 44.1k and comparing.

## Acceptance criteria
- [ ] Mix law identical to WING for all EQ models (tests vs emulated WING-Edit at several mix values)
- [ ] 44.1 kHz reference curves generated and all EQ/filter models within 0.01 dB

## Log
- 2026-10-25 coordinator: created from T04G/T04H follow-ups.
