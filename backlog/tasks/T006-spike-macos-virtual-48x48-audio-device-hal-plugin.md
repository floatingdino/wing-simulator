---
id: T006
title: Spike: macOS virtual 48x48 audio device (HAL plugin)
stage: S0
status: todo
owner: -
depends: []
branch: -
---

## Goal
Build driver/ — an AudioServerPlugIn using libASPL (MIT; fetch via git submodule or CMake FetchContent; install cmake via brew if needed) named 'Wing Simulator' with 48 output channels (DAW->sim, 'USB in') and 48 input channels (sim->DAW, 'USB out'), 48 kHz (+44.1). Audio is exchanged with a userland process via a POSIX shared-memory SPSC ring (define a minimal header layout; T020 will productionize it). Without a daemon running, behave as loopback (out->in) so it can be tested alone. Provide build script, install/uninstall scripts (they need sudo: copy to /Library/Audio/Plug-Ins/HAL and restart coreaudiod) — DO NOT run sudo; mark task blocked with exact commands for the human. Also inspect REAPER config (~/Library/Application Support/REAPER/reaper.ini, current project) read-only and document how REAPER's hardware outs map to WING USB channels. Write docs/research/driver.md.

## Acceptance criteria
- [ ] driver builds with a single script to a .driver bundle
- [ ] Loopback behaviour unit/integration test plan documented; test runnable after human install
- [ ] Install commands documented for human; reaper routing documented

## Log
- 2026-10-25 coordinator: created.
