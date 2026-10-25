---
id: T005
title: Research: firmware container analysis (timeboxed 1h)
stage: S0
status: done
owner: research-agent
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
- 2026-10-25 research agent: analysed .wingfw (entropy map, signature scan, strings, brief web search). Result: no container header; 0-0x234000 random-looking (encrypted/compressed), 0x234000 Efinix T85 FPGA bitstream with clear header, 0x450000-0x49C000 clear Cortex-M MCU code (Dante/WING-LIVE/host SPI), bulk 0x520000+ compressed with unidentified LZ scheme (not encrypted) containing readable snapshot JSON. DSP code/coeffs not recoverable. No key cracking attempted. Output docs/research/firmware.md. Files changed: docs/research/firmware.md. Scripts in scratchpad t005/. Proposed follow-ups: optionally identify LZ scheme to harvest embedded factory scene + node names (clear assets only).
