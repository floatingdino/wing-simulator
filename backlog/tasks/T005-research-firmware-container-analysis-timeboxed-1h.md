---
id: T005
title: Research: firmware container analysis (timeboxed 1h)
stage: S0
status: todo
owner: -
depends: []
branch: -
---

## Goal
Determine what can be learned from wing-fullsize-release-3.1.2-20260729.wingfw (it appears encrypted/compressed). Entropy profile, any headers/containers, known WING firmware structure from public sources. Do NOT attempt to defeat encryption via cracking keys; only document structure, public info, and anything recoverable in clear (e.g. release notes, model lists, version info). Output docs/research/firmware.md.

## Acceptance criteria
- [ ] firmware.md states clearly whether DSP code/coeffs are recoverable and why
- [ ] Lists any useful clear-text metadata found

## Log
- 2026-10-25 coordinator: created.
