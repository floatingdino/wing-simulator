---
id: T061
title: Record/bounce outputs from the daemon
stage: S6
status: review
owner: engineering-agent
depends: [T026, T037, T038]
branch: task/T060-T061
---

## Goal
Record any output pair/mains to WAV from CLI/OSC while running.

## Acceptance criteria
- [x] Recording test (`apps/wing-sim/tests/record.rs`, plus the daemon-path check in `apps/wing-render/tests/determinism.rs`)

## Log
- 2026-10-25 coordinator: created.
- 2026-10-25 engineering-agent: implemented on `task/T060-T061` (with T060).
  - `apps/wing-sim/src/record.rs`: `Recorder` (control side) / `RecordSlot` (lock-free hand-over: atomic pointers
    `incoming`/`retired` + stop flag) / `RecordTap` (RT side: tap lanes resolved to engine ports on install, then
    `wing_hostio::ring` SPSC push per block, wait-free; overflow counted, never blocks). Non-RT writer thread drains
    every 20 ms into one 32-bit float `WavWriter` per tap group (headers patched every 2 s and on stop). Taps are
    allocated and dropped only on control threads (the RT thread swaps pointers). Tap syntax shared with wing-render
    (`main/1,bus/3,mon/1,usb_out/1-48`; stereo strips one file, flat taps one mono file per lane).
  - `Console::attach_recorder(slot)` + per-cycle poll/push in `Console::cycle`; `AudioService::spawn_with_recorder`
    (`spawn` keeps its signature and creates one); every console the service builds polls the slot.
    The status line ends with the recorder state (`rec off`, `rec armed (waiting for audio)`, `rec 12.3 s, 3
    file(s) -> DIR, dropped 0`).
  - OSC: generic `wing_proto::osc::SimExtension` hook (`OscConfig::extension`, tried before the T065 scene commands):
    `/wingsim/record/start ,ss <taps> <dir>`, `/wingsim/record/stop`, `/wingsim/record/status` (and `/wingsim/record`),
    each replying `/wingsim/record ,s <status|ERROR ...>`; the daemon logs start/stop. CLI client:
    `wing-sim record start <taps> <dir> | stop | status [--host H] [--osc-port N]` (sends OSC to the running
    daemon, prints the reply). A recording still running at daemon shutdown is finalized.
  - Limits: a console rebuild (sample-rate change / reattach) ends the recording (status says so; stop finalizes);
    WAV ≤ 4 GiB; files are at the console's rate (48 kHz if armed with no session).
  - Tests: `tests/record.rs`: bit-exact files vs console ports with a counting allocator on the RT thread (0
    allocations/frees across install + 200 cycles); stop of a never-installed recording, restart over a running
    one; OSC start/status/stop end-to-end on an ephemeral port. `wing-proto` unit test for the extension
    dispatch. `wing-render/tests/determinism.rs` records from the real AudioService and checks the status line.
    None touch ports 2222/2223 or `/wingsim.audio.v1`.
  - Files: apps/wing-sim/src/{record.rs,wav.rs,lib.rs,main.rs,audio/console.rs,audio/driver.rs},
    apps/wing-sim/tests/record.rs, crates/wing-proto/src/osc/{mod.rs,server.rs,service.rs}.
  - Test: `make check` (passes). Manual: `wing-sim record start main/1,usb_out/1-2 ~/rec` … `wing-sim record stop`.
