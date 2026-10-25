---
id: T080
title: BUG: dynamics/gate sidechain key source selection has no effect
stage: S4
status: todo
owner: -
depends: []
branch: -
---

## Goal
User: "dynamics sidechain/key source handling doesn't seem to affect anything". Implement WING key-source routing end to end for gate and dyn slots on channels/aux/bus/main/mtx: key source selection (self / other channel / bus / any source per schema keys, e.g. ch/N/gate/... and ch/N/dyn/... sc source, `dynsc`, key filter, key listen), cross-strip keys (needs a graph dependency or a one-block-late tap — choose; document latency), key filters applied per model, external keys reaching every model (incl. alternate models that currently ignore the key, e.g. 3CMP). Regression tests: a ducker on ch1 keyed from ch2 ducks; compressor keyed from a bus reacts; key listen solos the key.

## Acceptance criteria
- [ ] Key source selection works for every source type in the schema, all models
- [ ] Tests; zero RT alloc

## Log
- 2026-10-26 coordinator: created from user report.
