---
id: T001
title: Repo scaffold: Rust workspace + build/test harness
stage: S0
status: todo
owner: -
depends: []
branch: -
---

## Goal
Create the Cargo workspace and empty crates per docs/ARCHITECTURE.md (wing-proto, wing-model, wing-dsp, wing-blocks, wing-engine, wing-shm, apps/wing-sim, apps/wing-render), a top-level Makefile (`make check` = fmt check + clippy -D warnings + tests), rust-toolchain.toml (stable), release profile (lto=thin, codegen-units=1, target-cpu=apple-m1 via .cargo/config.toml), criterion bench skeleton in wing-dsp, and CLAUDE.md with conventions (RT rules from backlog/README.md).

## Acceptance criteria
- [ ] `make check` passes on a clean clone
- [ ] Each crate has a lib.rs with module doc + one trivial test
- [ ] CLAUDE.md documents build/test commands and RT rules

## Log
- 2026-10-25 coordinator: created.
