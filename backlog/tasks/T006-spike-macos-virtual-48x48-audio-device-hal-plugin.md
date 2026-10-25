---
id: T006
title: Spike: macOS virtual 48x48 audio device (HAL plugin)
stage: S0
status: done
owner: agent-T006
depends: []
branch: task/T006
---

## Goal
Build driver/ — an AudioServerPlugIn using libASPL (MIT; fetch via git submodule or CMake FetchContent; install cmake via brew if needed) named 'Wing Simulator' with 48 output channels (DAW->sim, 'USB in') and 48 input channels (sim->DAW, 'USB out'), 48 kHz (+44.1). Audio is exchanged with a userland process via a POSIX shared-memory SPSC ring (define a minimal header layout; T020 will productionize it). Without a daemon running, behave as loopback (out->in) so it can be tested alone. Provide build script, install/uninstall scripts (they need sudo: copy to /Library/Audio/Plug-Ins/HAL and restart coreaudiod) — DO NOT run sudo; mark task blocked with exact commands for the human. Also inspect REAPER config (~/Library/Application Support/REAPER/reaper.ini, current project) read-only and document how REAPER's hardware outs map to WING USB channels. Write docs/research/driver.md.

## Acceptance criteria
- [x] driver builds with a single script to a .driver bundle (`driver/build.sh` -> `driver/build/WingSimulator.driver`)
- [x] Loopback behaviour unit/integration test plan documented; test runnable after human install
      (pre-install: ctest `shm_selftest` + `driver_smoketest` pass; post-install: `wing_hal_probe`, docs/research/driver.md §6)
- [x] Install commands documented for human; reaper routing documented (docs/research/driver.md §5, §7)
- [ ] Human install + post-install probe (BLOCKED: needs sudo)

## Log
- 2026-10-25 coordinator: created.
- 2026-10-25 agent-T006: started. `brew install cmake` (4.4.4). libASPL v3.1.2 via FetchContent.
- 2026-10-25 agent-T006: IPC research. Third-party plug-ins run in the sandboxed
  `Core Audio Driver (X.driver)` helper (uid _coreaudiod) under the coreaudiod.sb profile. That profile has
  `(allow ipc-posix-shm)` unconditionally, denies file writes outside /Library/Preferences, and has no ipc-posix-sem.
  Verified with sandbox-exec + that profile: opening/mapping/writing a user-created shm segment works; writing
  to /tmp or /Users/Shared gets EPERM. macOS pshm quirks: mode is applied verbatim (no umask); fchmod gives
  EINVAL; ftruncate only works once; name max 31 chars. Chose POSIX shm `/wingsim.audio.v1`, mode 0666,
  driver creates or attaches, with a private-memory fallback (loopback only).
- 2026-10-25 agent-T006: implemented and tested. Files:
  driver/{CMakeLists.txt,Info.plist.in,build.sh,install.sh,uninstall.sh},
  driver/include/wing_shm.h, driver/src/{Driver.cpp,ShmLink.hpp,ShmLink.cpp},
  driver/tools/{wing_shm_client,wing_shm_selftest,wing_driver_smoketest,wing_hal_probe}.cpp,
  docs/research/driver.md.
  `driver/build.sh --test`: bundle valid (arm64+x86_64, ad-hoc signed); ctest 2/2 pass.
  - shm_selftest: exact loopback; daemon gain path exact with 0 underruns; epoch restart; detach goes back to loopback.
  - driver_smoketest: real bundle via the AudioServerPlugIn C interface with a fake host. Checks 48/48 float32,
    44.1k/48k, channel names, rate changes, StartIO epoch, exact loopback through DoIOOperation.
- 2026-10-25 agent-T006: BLOCKED awaiting human install. Quit REAPER, then from the repo (or this worktree):
    driver/build.sh --test
    sudo driver/install.sh
  which is equivalent to:
    sudo rm -rf /Library/Audio/Plug-Ins/HAL/WingSimulator.driver
    sudo cp -R driver/build/WingSimulator.driver /Library/Audio/Plug-Ins/HAL/
    sudo chown -R root:wheel /Library/Audio/Plug-Ins/HAL/WingSimulator.driver
    sudo launchctl kickstart -k system/com.apple.audio.coreaudiod   # or: sudo killall coreaudiod
  Then, without sudo:
    driver/build/wing_hal_probe --info
    driver/build/wing_hal_probe                # expect gain 1 (loopback)
    driver/build/wing_shm_client & driver/build/wing_hal_probe --expect-gain -1; kill %1
  The terminal needs Microphone permission. Uninstall: `sudo driver/uninstall.sh`.
  Once the probe passes, set status: review.
- Proposed follow-ups (not done here):
  - T020: replace daemon polling with os_sync_wait_on_address (shared) or clock-from-driver-timestamps; handle
    input-only clients (to_daemon not advancing).
  - T021: reconsider shm mode 0666 (daemon uid handshake). Measure real driver-host shm access via os_log.
  - REAPER project 2025-06-29_AM.RPP: master hw out 1/2 collides with USB In 1/2. Document or adjust in the
    virtual-soundcheck how-to.
