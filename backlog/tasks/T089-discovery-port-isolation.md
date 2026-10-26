---
id: T089
title: wing-sim test isolation: discovery always binds UDP 2222
stage: S7
status: todo
owner: -
depends: []
branch: -
---

## Goal
A test/smoke run with `--port`/`--osc-port` set to private ports still bound discovery on UDP 2222 and answered the user's WING-Edit (T088 incident). Add `--discovery-port N` and `--no-discovery`; default discovery port follows `--port` when it's non-default; all tests/examples/harnesses (soak, audio_only, wing-verify) use `--no-discovery` or a private port. Add a test that a non-default `--port` never binds 2222.

## Acceptance criteria
- [ ] No test/example binds 2222/2223
- [ ] Flags documented in README

## Log
- 2026-10-26 coordinator: created from T088 incident report.
