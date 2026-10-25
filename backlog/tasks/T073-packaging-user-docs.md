---
id: T073
title: Packaging + user docs
stage: S7
status: review
owner: T073 agent
depends: [T064]
branch: task/T073
---

## Goal
Installer (driver+daemon+app), README, troubleshooting.

## Acceptance criteria
- [x] Fresh-machine install checklist (docs/user/install-checklist.md; not yet run on a clean machine)

## Log
- 2026-10-25 coordinator: created.
- 2026-10-25 T073 agent: done, status review. `make dist` (tools/dist/dist.sh) -> dist/WingSimulator-0.1.0/ (20 MB): bin/{wing-sim,wing-render,wing-diff}, WingSimulator.driver (universal), WingSimMenu.app (bundles all three tools in Resources/bin; WingSimRepoRoot empty via WINGSIM_REPO_ROOT="", DaemonPaths ignores an empty root), install.sh/uninstall.sh, reaper integration, docs, SHA256SUMS. All ad-hoc signed and `codesign --verify`ed; notarisation documented as future work (install-checklist.md). install.sh: user-level parts need no privileges; the driver step prints the exact sudo commands, asks (default no; no TTY = no), then runs sudo itself; options `--dry-run`, `--no-driver`, `--driver-only`. Files: Makefile, .gitignore (/dist/), tools/dist/{dist,install,uninstall}.sh, apps/WingSimMenu/build.sh (+wing-render, WINGSIM_REPO_ROOT), DaemonPaths.swift, README.md, docs/user/install-checklist.md. Tested: make dist, install.sh --dry-run, make check passes, swift test 19 pass. Not tested: real install/uninstall on a clean machine (needs sudo; human step: follow docs/user/install-checklist.md); uninstall.sh only syntax-checked.
  Proposed follow-ups: Developer ID signing + notarisation + .pkg; confirm README claims on the WING-Edit monitor knob and solo behaviour in WING-Edit; zip/dmg make target.
