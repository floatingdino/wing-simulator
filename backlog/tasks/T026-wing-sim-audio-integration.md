---
id: T026
title: wing-sim audio integration (shm -> engine -> USB outs + host bridge)
stage: S2
status: todo
owner: -
depends: [T020, T022, T023, T024, T03A, T035]
branch: -
---

## Goal
Wire the realtime audio path into apps/wing-sim: DaemonLink (wing-shm) drives the Engine (LiveClock, workers joined to the driver/device workgroup, time-constraint policy); per cycle: read 48 USB inputs → routing (T024) → channel/aux strips (T035) bound to the ParamStore via the T03A pipeline → (busses/mains come in T036; until then a provisional post-fader stereo sum to main 1 is acceptable, clearly marked) → output patching → write USB outs back to the driver and feed host bridges (wing-hostio, config file e.g. `--hostio hostio.toml`, default monitor A/main 1 -> Scarlett 1/2 when present). CPU load/xrun stats in the status line and via OSC. Fallback behaviour when the driver isn't present (offline clock for testing).

## Acceptance criteria
- [ ] With the driver installed: REAPER plays the session, channel processing audible on the host device, edits in WING-Edit take effect live
- [ ] Load %, xruns reported; zero RT allocations
- [ ] Integration test with an offline clock and synthetic input

## Log
- 2026-10-25 coordinator: created — gap found: no task owned daemon audio integration. T025 milestone depends on it.
