---
id: T087
title: Prevent the menu-bar app from running a stale bundled wing-sim
stage: S7
status: done
owner: -
depends: []
branch: task/T087
---

## Goal
The user started wing-sim from WingSimMenu.app, which ran a bundled binary built 2026-10-25 21:36 — missing every fix since. Make staleness impossible to miss: (1) embed git commit + build time in wing-sim/wing-render/wing-diff (`--version`, startup log line, status.json); (2) the menu app shows the daemon's version and warns when the repo's target/release binary is newer than the bundled one (dev builds), offering to use it; (3) `make app` always rebuilds the release binaries before bundling; `make dist` likewise.

## Acceptance criteria
- [x] Version/commit visible in log, status.json and the app
- [x] Stale-binary warning in the app (dev mode)

## Log
- 2026-10-26 coordinator: created after the user ran an outdated bundled daemon.
- 2026-10-26 agent: done. Shared apps/build_info.rs (include!d by each app's build.rs) exports WING_GIT_HASH (short commit, `-dirty`, `unknown` when git is absent), WING_BUILD_UNIX/TIME (honours SOURCE_DATE_EPOCH). `--version`/`-V` on wing-sim, wing-render, wing-diff; wing-sim startup log line and status.json (`commit`, `built_unix`) carry it. Menu app: header shows version+commit; `BinaryLocator.newerBuild` (a later search dir's binary with newer mtime than the one found) drives an orange stale warning with "Use Newer Build" (applies on next start). `make app` now also builds wing-render (cargo always rebuilds before bundling; `make dist` already did); build.sh prints each bundled binary's --version. Files: apps/build_info.rs, apps/*/build.rs, apps/wing-{sim,render,diff}/src/main.rs, apps/wing-sim/src/control.rs, fixtures/control/status.json, WingSimMenu (DaemonStatus, DaemonPaths, AppModel, MenuContentView, tests, build.sh), Makefile. `make check` and `swift test` pass; --version verified on release binaries. Not run: `make app` smoke test, UI not visually checked.
