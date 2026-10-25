---
id: T03E
title: Latency compensation (model-slot dry path + strip/bus alignment)
stage: S4
status: todo
owner: -
depends: [T04D, T037, T038]
branch: -
---

## Goal
Oversampled models report latency (76LA/NSTR: 57 samples; others 0). The engine's ModelSlot doesn't delay its dry path, so `mix` < 100 % comb-filters (PUBLIC.snap uses NSTR at mix 80), and channels with different model latencies arrive misaligned at busses/mains. Implement: (1) dry-path delay in ModelSlot matching the active model's latency (incl. during crossfades between models of different latency); (2) decide WING's behaviour — WING's DSP runs fixed-latency blocks, so emulate constant per-strip latency: pad every strip to the max possible model latency of its slots so summing stays phase-coherent regardless of model choice; document the total added latency; (3) report total I/O latency to the driver/REAPER where appropriate.

## Acceptance criteria
- [ ] mix < 100 % on NSTR shows no comb filtering (null test vs ideal parallel)
- [ ] Two channels with different dyn models summed to a bus stay sample-aligned (null test)
- [ ] Added latency documented; zero RT alloc

## Log
- 2026-10-25 coordinator: created from T04D follow-up.
