---
id: T03G
title: BUG: channel solo from WING-Edit has no effect on monitor output
stage: S3
status: todo
owner: -
depends: [T038, T03F]
branch: -
---

## Goal
User (live, v0.4 build): monitor A on the Scarlett works, DCA mute/solo works, but channel solo does nothing. Evidence (scratch/wing-sim-v0.4.log): WING-Edit sends `ch/1/$solo = 1/0`, `bus/N/$solo = 1/0`, `$stat/solo = 0/1`, and `cfg/solo/mon = PH`. Find why channel `$solo` doesn't reach the solo bus/monitor (binding of `ch/N/$solo` and `aux/N/$solo`; solo-to-monitor routing when cfg/solo/mon = PH vs SPK and which of mon/1 / mon/2 is PH (phones) vs SPK (speakers) — the host bridge plays mon/1); fix so soloing a channel in WING-Edit switches the monitor that the host bridge plays (if solo targets the phones monitor and the bridge plays the other, make the default host mapping follow `cfg/solo/mon`, or document clearly and pick the WING-faithful mapping). Regression test reproducing the exact set sequence from the log first.

## Acceptance criteria
- [ ] Test fails before / passes after
- [ ] Channel, aux, bus, main, mtx and DCA solo all affect the monitor output per cfg/solo
- [ ] PH/SPK ↔ mon/1/mon/2 mapping documented with evidence

## Log
- 2026-10-25 coordinator: created from user report; DCA solo/mute confirmed working, monitors confirmed working.
