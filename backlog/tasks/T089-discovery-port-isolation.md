---
id: T089
title: wing-sim test isolation: discovery always binds UDP 2222
stage: S7
status: review
owner: agent
depends: []
branch: task/T089
---

## Goal
A test/smoke run with `--port`/`--osc-port` set to private ports still bound discovery on UDP 2222 and answered the user's WING-Edit (T088 incident). Add `--discovery-port N` and `--no-discovery`; default discovery port follows `--port` when it's non-default; all tests/examples/harnesses (soak, audio_only, wing-verify) use `--no-discovery` or a private port. Add a test that a non-default `--port` never binds 2222.

## Acceptance criteria
- [x] No test/example binds 2222/2223
- [x] Flags documented in README

## Log
- 2026-10-26 coordinator: created from T088 incident report.
- 2026-10-26 agent: added `--discovery-port N` and `--no-discovery` to wing-sim (apps/wing-sim/src/main.rs); discovery port follows `--port` when it is non-default (`Args::discovery_bind_port`), mutually exclusive flags rejected; `--no-discovery` uses an ephemeral meter socket. The startup "listening:" line prints `discovery off` or `udp <addr>`. Audit: no example/test/harness (soak, audio_only, wing-render, verify-hardware, menu smoke.sh) starts discovery; they use in-process ephemeral loopback sockets. Tests: unit `discovery_port_never_defaults_to_2222_with_private_port`; integration `apps/wing-sim/tests/port_isolation.rs` spawns the binary with `--port 0` and checks the log never shows 2222/2223. README section "Ports, and running a second copy". Note: default `--osc-port` is still 2223 (pass `--osc-port N`/`--no-osc`).
