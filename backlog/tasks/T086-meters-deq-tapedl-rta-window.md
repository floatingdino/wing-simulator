---
id: T086
title: BUG: DEQ and TAPE-DL meters static; "RTA in use on another window!"; meter report churn
stage: S4
status: todo
owner: -
depends: [T082, T084]
branch: -
---

## Goal
User (v0.5 build): 76LA GR display works, but dynamic-EQ (DEQ) meters don't move and TAPE-DL VU meters don't move. RTA overlay now says "RTA in use on another window!". Evidence in scratch/wing-sim-v0.5.log: WING-Edit re-defines meter reports continuously (`report 809..820 defined` within ~150 ms, alternating `1 meters: ChannelV2 1` and `22 meters: ChannelV2 1-16, MainV2 1, Fx 7, Source 1,4-5,8, Rta`), and `cfg/rta/rtasrc` sets. Find out (WING-Edit code via strings/symbols/Unicorn emulation only — never run it natively; protocol docs; libwing) (a) why WING-Edit keeps re-defining reports (missing ack/reply? report id handling? renew semantics?) and fix the server; (b) the exact condition for "RTA in use on another window!" (likely RTA ownership tied to a report id / client / window id, or multiple reports containing Rta) and satisfy it; (c) the DEQ meter words (band gain/activity) and TAPE-DL's VU/state words WING-Edit reads for those pages, and populate them from the models. Tests for each.

## Acceptance criteria
- [ ] No report-definition churn in a replay of the log's request sequence
- [ ] RTA condition documented and satisfied (test)
- [ ] DEQ and TAPE-DL meter words populated (tests)

## Log
- 2026-10-26 coordinator: created from user v0.5 test.
