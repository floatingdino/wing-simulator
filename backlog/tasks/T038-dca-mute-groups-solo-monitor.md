---
id: T038
title: DCA, mute groups, solo/monitor
stage: S3
status: todo
owner: -
depends: [T036]
branch: -
---

## Goal
DCAs, mute groups, solo (PFL/AFL modes), monitor section output to host device.

## Acceptance criteria
- [ ] Monitor section per scene cfg/mon + cfg/solo: monitor A/B source (e.g. MAIN.1), srclvl/srcmix, dim, mono, delay, limiter, monitor EQ; solo modes (LIVE/etc.), ch/bus/main/mtx taps (PFL/AFL), exclusive vs additive, solo-in-place where WING supports it
- [ ] Solo priority: while any solo is active the monitor outputs carry the solo bus (mixed per srcmix), returning to the monitor source when cleared, click-free
- [ ] Monitor A/B are selectable as host-bridge sources (T023 mapping, e.g. `mon/1 -> "Scarlett 2i4 USB" 1/2`) so the host output behaves like console monitors with solo priority; this is the recommended default host mapping
- [ ] Solo state round-trips with WING-Edit (solo buttons, clear solo, solo indicators)
- [ ] Behaviour tests

## Log
- 2026-10-25 coordinator: created.
- 2026-10-25 coordinator: user requested monitor-output-with-solo-priority workflow; acceptance expanded.
