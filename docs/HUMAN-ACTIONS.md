# Actions needed from the human (coordinator-maintained)

Agents never run sudo or touch the user's live sessions destructively. Items here unblock tasks.

| # | Action | Unblocks | Status |
|---|---|---|---|
| 1 | Grant project permissions (allow cargo, cmake, brew install, git, python3, xcodebuild, WebSearch, WebFetch in `.claude/settings.local.json`) or keep auto mode on | unattended running | open |
| 2 | Save the open WING-Edit scene to a file, then follow docs/research/protocol.md §7 (run `tools/protocol-probe/fake_wing.py`, click CONNECT on the fake console in WING-Edit) to capture WING-Edit's first TCP bytes | T013 fidelity | open |
| 3 | Install the HAL driver with sudo (commands in docs/research/driver.md, after T006) | T021, T025 | waiting on T006 |
| 4 | After the driver is installed: switch REAPER's audio device to "Wing Simulator" | T025 | waiting |
