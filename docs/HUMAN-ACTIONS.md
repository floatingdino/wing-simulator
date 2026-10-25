# Actions needed from the human (coordinator-maintained)

Agents never run sudo or touch the user's live sessions destructively. Items here unblock tasks.

| # | Action | Unblocks | Status |
|---|---|---|---|
| 1 | Grant project permissions (allow cargo, cmake, brew install, git, python3, xcodebuild, WebSearch, WebFetch in `.claude/settings.local.json`) or keep auto mode on | unattended running | open |
| 2 | (done 2026-10-25) Save the open WING-Edit scene to a file, then follow docs/research/protocol.md §7 (run `tools/protocol-probe/fake_wing.py`, click CONNECT on the fake console in WING-Edit) to capture WING-Edit's first TCP bytes | T013 fidelity | done 2026-10-25 |
| 3 | Quit REAPER, then run `driver/build.sh && sudo driver/install.sh`; verify with `driver/build/wing_hal_probe` (see docs/research/driver.md) | T021, T025 | done 2026-10-25 (wing_hal_probe PASS 48x48, unity loopback) |
| 4 | After the driver is installed: switch REAPER's audio device to "Wing Simulator" | T025 | done 2026-10-25 |
| 5 | In REAPER disable the master track's hardware output to outs 1/2 (it would sum the whole mix into USB In 1/2 = Kick/Snare) | T025 | done 2026-10-25 |
| 6 | Reinstall the v2 driver: quit REAPER, then `driver/build.sh && sudo driver/install.sh`, then `driver/verify.sh` (no sudo; terminal needs Microphone permission) and paste the result | T021 sign-off, T025 | done 2026-10-25 (verify ALL PASSED) |
