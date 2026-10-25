---
id: T001
title: Repo scaffold: Rust workspace + build/test harness
stage: S0
status: done
owner: eng-agent-T001
depends: []
branch: task/T001
---

## Goal
Create the Cargo workspace and empty crates per docs/ARCHITECTURE.md (wing-proto, wing-model, wing-dsp, wing-blocks, wing-engine, wing-shm, apps/wing-sim, apps/wing-render), a top-level Makefile (`make check` = fmt check + clippy -D warnings + tests), rust-toolchain.toml (stable), release profile (lto=thin, codegen-units=1, target-cpu=apple-m1 via .cargo/config.toml), criterion bench skeleton in wing-dsp, and CLAUDE.md with conventions (RT rules from backlog/README.md).

## Acceptance criteria
- [ ] `make check` passes on a clean clone
- [ ] Each crate has a lib.rs with module doc + one trivial test
- [ ] CLAUDE.md documents build/test commands and RT rules

## Log
- 2026-10-25 coordinator: created.
- 2026-10-25 eng-agent-T001: Done, status review. Files: Cargo.toml (workspace; release lto=thin, cgu=1, panic=abort), Cargo.lock, rust-toolchain.toml, .cargo/config.toml (apple-m1), Makefile (check/fmt/build/bench), CLAUDE.md, 6 crates, 2 apps, crates/wing-dsp/benches/dsp.rs (criterion dev-dep only). Test: make check (rc=0); make bench.
