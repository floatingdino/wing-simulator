---
id: T064
title: Control app (menu bar) for config/status
stage: S6
status: review
owner: claude-T064
depends: [T023, T025]
branch: task/T064
---

## Goal
SwiftUI menu bar app: device routing (USB out N/M -> host device ch), scene file, CPU/xrun, start/stop daemon.

## Acceptance criteria
- [ ] Manual checklist (below; not run by the agent: it must not touch the user's running wing-sim or ports 2222/2223)
  1. `make app`, open `apps/WingSimMenu/build/WingSimMenu.app` (stop any other wing-sim first).
  2. Start wing-sim: icon changes, status shows load %, cycle p50/p99, xruns, WING-Edit clients, bridges (fill/target, e2e).
  3. Connect WING-Edit: clients count goes to 1.
  4. Host Output Routing…: pick tap (e.g. Main 1 L+R), device, first output; Apply & Reload: the bridge restarts
     (short gap) and the status lists it; `<state-dir>/hostio.toml` holds the route.
  5. Save As… a .snap; change a fader in WING-Edit; Copy Diff vs Scene File: the pasteboard has the wing-diff text.
  6. Open… the .snap: the fader returns. Stop wing-sim: status goes to stopped.

## Design
- wing-sim (Rust, `apps/wing-sim/src/control.rs`, lib module): off the RT path.
  - `<state-dir>/status.json`: `DaemonStatus` JSON, rewritten atomically every second by the main loop
    (`running:false` on exit). Schema fixture: `fixtures/control/status.json` (Rust and Swift tests both use it).
  - `<state-dir>/control.sock`: Unix socket, line protocol `ping | status | save P | export P | load P |
    reload-hostio [P] | hostio-off | quit` -> `ok ...`/`err ...`. The socket thread forwards requests over a
    channel; the main loop executes them. `--no-control` disables both.
  - Live host-output reload: `AudioService::set_hostio` restarts the session with the new bridges (pipeline kept);
    structured `AudioReport`/`BridgeReport` next to the status line.
- apps/WingSimMenu (Swift 6.2 package): `WingSimCore` (status decode/liveness, control client, hostio TOML
  read/write + validation, CoreAudio output devices, daemon launch, wing-diff) and the `WingSimMenu` executable
  (MenuBarExtra window + "Host Output Routing" window). Daemon started as
  `wing-sim --state-dir D [--hostio D/hostio.toml] [--scene F]`, log appended to `D/wing-sim.log`; binaries from
  `$WINGSIM_BIN_DIR`, the bundle's `Resources/bin`, or `<repo>/target/release`. `$WINGSIM_STATE_DIR` overrides D.
  A daemon not started by the app is shown as "(external)" and can still be controlled through the socket.

## Log
- 2026-10-25 coordinator: created.
- 2026-10-25 claude-T064: done, status review.
  - Files: apps/wing-sim/src/{control.rs (new), lib.rs, main.rs, audio/driver.rs, audio/hostio.rs},
    apps/wing-sim/Cargo.toml (serde, serde_json), apps/wing-sim/tests/audio.rs
    (`service_reloads_host_outputs_live`, private shm name), crates/wing-hostio/src/config.rs
    (`parses_wingsimmenu_output`), fixtures/{control/status.json, hostio/wingsimmenu.toml},
    apps/WingSimMenu/** (Package.swift, Sources, Tests, build.sh, smoke.sh, Info.plist.in), Makefile (`make app`),
    .gitignore, CLAUDE.md.
  - Test: `make check` (passes); `make app` = release wing-sim/wing-diff, build.sh, `swift test` (19 tests in 4
    suites, incl. a fake control-socket server), smoke.sh (app launches with a temp state dir and quits by itself,
    exit 0, no daemon started) - passes. The daemon itself was not launched (ports 2222/2223 are fixed).
  - Gotchas: discovery always binds UDP 2222, so a second wing-sim cannot run beside the user's; the app's Start
    then fails and shows the log tail. A daemon started before this change writes no status.json (shows stopped).
  - Proposed follow-ups: `--discovery-port` flag so the daemon can be integration-tested end to end;
    launch-at-login (SMAppService); per-route latency field in the routing editor; app icon.
