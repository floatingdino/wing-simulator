---
id: T087
title: Prevent the menu-bar app from running a stale bundled wing-sim
stage: S7
status: todo
owner: -
depends: []
branch: -
---

## Goal
The user started wing-sim from WingSimMenu.app, which ran a bundled binary built 2026-10-25 21:36 — missing every fix since. Make staleness impossible to miss: (1) embed git commit + build time in wing-sim/wing-render/wing-diff (`--version`, startup log line, status.json); (2) the menu app shows the daemon's version and warns when the repo's target/release binary is newer than the bundled one (dev builds), offering to use it; (3) `make app` always rebuilds the release binaries before bundling; `make dist` likewise.

## Acceptance criteria
- [ ] Version/commit visible in log, status.json and the app
- [ ] Stale-binary warning in the app (dev mode)

## Log
- 2026-10-26 coordinator: created after the user ran an outdated bundled daemon.
