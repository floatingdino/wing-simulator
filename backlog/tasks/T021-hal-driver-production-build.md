---
id: T021
title: HAL driver production build
stage: S2
status: in_progress
owner: eng-agent
depends: [T006, T020]
branch: task/T021
---

## Goal
Driver uses wing-shm, reports correct latency/safety offsets, sample rates 44.1/48k, channel names 'USB 1..48', falls back to silence/loopback when daemon absent.

## Acceptance criteria
- [ ] REAPER sees 48x48 'Wing Simulator'
- [ ] Glitch-free 1h loopback at 64/128/256 buffers

## Log
- 2026-10-25 coordinator: created.
