---
id: T021
title: HAL driver production build
stage: S2
status: done
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
- 2026-10-25 eng-agent: implementation done on `task/T021`; **blocked on a human reinstall** (needs sudo).
  - What changed (details: docs/research/driver.md section 9):
    - shm v2 throughout. No layout change. The default input delay is now 128 (was 256), in both C and Rust.
      `wing_shm_init_header_delay()` was added.
    - Latency: input stream latency = input delay D; output stream latency, device latency and safety
      offsets are 0. D is configurable with the `InputDelayFrames` key in
      `/Library/Preferences/Audio/com.wingsimulator.driver.plist`. That is the only prefs path the coreaudiod
      sandbox lets a plug-in read, and the driver reads the file directly. A segment with a different
      delay is recreated, exactly like a version mismatch.
    - Default D = 128. A sweep (`wing_shm_selftest --sweep`, RT driver thread) measured that a
      time-constraint daemon needs 0 extra frames at 64/128/256, over 60 s each, at load average 5. A
      normal-priority daemon stalls 5 to more than 20 ms under load and needs about 1024 or more. So 128 is
      headroom for real DSP, and the daemon must be RT (T022). Added `gain_client --rt` for the meantime.
    - Fallback: loopback (or silence where nothing was written) when the daemon is dead, on another epoch,
      or late.
      - Switching is a 128-frame crossfade, and a stalled daemon's missing frames are held at its last
        sample.
      - The daemon must be on time for 1024 frames before the driver fades back to it.
      - A stale-data guard (pid change, write_time going back or jumping) means stale ring content is never
        played after a daemon restart.
      - `to_daemon` gaps are zero-filled. This fixes a T006 bug: the previous IO session's audio leaked at
        StartIO.
      - A per-cycle cache gives a second HAL client the same block without advancing the fade twice.
      - `Revalidate` at StartIO re-attaches if the shm name was removed or recreated.
    - Rates 44.1k/48k only. A rate change gives a new epoch plus a `cycle_seq` wake, so the daemon is
      notified immediately; 96k is rejected. Channel names are "USB In n" / "USB Out n".
    - `os_log` only at load, StartIO, StopIO and rate changes. StopIO logs totals and the session's
      minimum daemon slack.
    - Copies: WriteMix goes straight into the ring and ReadInput straight into the HAL buffer
      (channel-outer). The only extra copy is the multi-client cache.
  - Files:
    - `driver/src/{Driver.cpp,ShmLink.cpp,ShmLink.hpp}` changed; `driver/src/Config.{hpp,cpp}` new
    - `driver/include/wing_shm.h`
    - `driver/tools/{wing_shm_selftest,wing_driver_smoketest,wing_hal_probe}.cpp`
    - `driver/CMakeLists.txt`; `driver/verify.sh` new
    - `crates/wing-shm`: layout default, `gain_client --rt`, `wake_latency --attach`, a doc line
    - `docs/research/driver.md` (status, sections 1/4/6/8, new section 9)
  - Tests (all pass):
    - `driver/build.sh --test`: `shm_selftest` with new phases for delay recreate, new-epoch no stale
      audio, SIGSTOP stall, SIGKILL crash plus scribbled ring plus restart (click-free) and Revalidate;
      `driver_smoketest`; and `driver_smoketest_delay` (`--delay 96` via plist). The smoketests cover
      latency/safety, all 96 names, rate change epoch + wake, a synchronous fake daemon with
      attach/stall/resume/1-cycle restart with a garbage gap/crash (0 clicks, 0 garbage), and two clients
      getting identical data.
    - `make check` passes, including `cpp_selftest` against the Rust daemon.
    - Mutation checks by hand: with no crossfade the tests report clicks; with no stale guard the
      smoketest plays the garbage.
    - Against the currently installed v1 driver, the read-only `wing_hal_probe --info --expect-latency
      256` passes. Against a test driver, `wake_latency --attach` shows p99 20 µs.
  - Human steps (in order):
    1. Quit REAPER.
    2. `cd /Users/sam/code/wing-simulator` (after merge; or this worktree before merge)
    3. `driver/build.sh && sudo driver/install.sh`
    4. `driver/verify.sh`. No sudo. It takes about 4 min and the terminal needs Microphone permission. It
       exits 0 with "ALL PASSED". It checks:
       - the installed binary
       - 48x48, names, input latency 128
       - loopback +1
       - 44.1k/48k with the shm rate
       - `gain_client --rt` gain −1
       - `wake_latency --attach` p99 ≤ 1 ms from inside the coreaudiod sandbox
       - 60 s at buffers 64/128/256 with 0 overloads, timestamp gaps, glitch frames or shm underruns
       - a click-free stall/crash/restart

       To tune the delay: `sudo defaults write /Library/Preferences/Audio/com.wingsimulator.driver
       InputDelayFrames -int N && sudo launchctl kickstart -k system/com.apple.audio.coreaudiod`.
  - Next: after verify passes, run a 1 h REAPER loopback at 64/128/256 (acceptance criterion 2), then
    set `status: review`.
  - Proposed follow-ups:
    - T022: the daemon RT thread + `os_workgroup`. Then rerun `wing_shm_selftest --sweep --client-arg
      --rt` with the real engine and retune the default D.
    - Restrict the shm (it is still 0666, with no daemon uid handshake).
    - `gain_client`/`DaemonLink` should re-attach automatically when the driver recreates the segment
      (version or delay change). Today they must be restarted.
- 2026-10-25 coordinator: v2 driver installed by user; driver/verify.sh ALL PASSED (scratch/verify-v2.log).
