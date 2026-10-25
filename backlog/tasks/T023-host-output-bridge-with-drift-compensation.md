---
id: T023
title: Host output bridge with drift compensation
stage: S2
status: in_progress
owner: agent-T023
depends: [T022]
branch: task/T023
---

## Goal
CoreAudio client to any host device; mapping config 'USB out 45/46 -> Scarlett 2i4 1/2' (also sim internal outputs/monitor). Adaptive resampling for clock drift; configurable latency.

## Acceptance criteria
- [ ] Hear REAPER audio on Scarlett via the sim
- [ ] Drift test: 1h no buffer under/overflow

## Log
- 2026-10-25 coordinator: created.
