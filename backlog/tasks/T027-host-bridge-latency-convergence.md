---
id: T027
title: Host bridge latency convergence + startup underruns
stage: S2
status: todo
owner: -
depends: [T023, T026]
branch: -
---

## Goal
Live v0.2 run (docs/milestones/v0.2.md, log scratch/wing-sim-v0.2.log): with REAPER at 512-frame buffers the Scarlett bridge ring fill drifted 23 -> 30 ms (target ~5 ms extra) and the PI ratio correction swung ±500 ppm instead of settling near the true drift; 1 bridge underrun and 2 driver underruns at startup. Fix: fill estimation robust to large/bursty producer blocks (producer block 512 vs device block e.g. 128/256), target latency derived from max(producer block, consumer block) + margin, PI tuning (anti-windup, slower integrator, deadband), startup priming so no underruns. Also report end-to-end latency (REAPER out -> Scarlett) in the status line.

## Acceptance criteria
- [ ] Simulated test with 512-frame producer / 128-frame consumer: fill settles within 1 ms of target, correction within ±20 ppm of true drift after 30 s
- [ ] No startup underruns in driver or bridge (test + live log)
- [ ] Live: fill stable for 10 min with REAPER at 512 and 128

## Log
- 2026-10-25 coordinator: created from v0.2 live run.
