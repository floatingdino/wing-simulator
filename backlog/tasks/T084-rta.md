---
id: T084
title: RTA support for WING-Edit ("RTA in use! Check mixer settings.")
stage: S6
status: todo
owner: -
depends: []
branch: -
---

## Goal
WING-Edit's EQ pages show "RTA in use! Check mixer settings." Find what WING-Edit expects (cfg/rta/* nodes: rtasrc, rtatap, decay, detector, range, gain, auto; the RTA meter report type in the meter protocol; any $-nodes signalling RTA availability) from the binary (strings/symbols, Unicorn emulation if needed — never run WING-Edit natively), protocol docs and libwing; then implement an RTA in the daemon: FFT analyser on the selected source/tap (off the RT thread: RT side copies the tapped block into a lock-free ring), band layout and dB scaling as WING sends, decay/peak detector, streamed via the meter transport at WING's rate. The EQ page RTA overlay in WING-Edit should then work.

## Acceptance criteria
- [ ] Root cause of the message documented
- [ ] RTA data packets match WING's format (tests); live: overlay visible in WING-Edit (manual check by user)
- [ ] Zero RT alloc, analyser cost reported

## Log
- 2026-10-26 coordinator: created from user request.
