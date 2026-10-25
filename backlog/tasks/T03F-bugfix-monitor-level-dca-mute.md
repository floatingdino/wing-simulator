---
id: T03F
title: BUG: monitor level stuck at -inf; DCA mutes not respected
stage: S3
status: todo
owner: -
depends: [T038]
branch: -
---

## Goal
User report (live, v0.3 build, WING-Edit connected PC->Mixer, menu-bar app routing mon/1 -> Scarlett):
1. With the host tap on the monitor (mon/1) there is no audio: "the level fader doesn't move from -inf". Works with a main tap.
2. DCA mutes are not respected (muting a DCA doesn't mute its members).
Diagnose with evidence (logs in scratch/wing-sim-v0.3b.log or the current wing-sim log, the protocol fixtures, schema, docs). Hypotheses: (1) the monitor level node is a hardware-pot/$ node WING-Edit can't set, or the server rejects/ignores its sets, or the scene push sets it to -inf after wing-sim's startup override; decide WING-faithful behaviour (on a real WING the monitor level is the physical knob) — e.g. treat monitor A/B level as a simulator-side setting (default 0 dB, exposed in OSC/control socket/menu app) unless WING-Edit can legitimately set it, and make sure WING-Edit's monitor-level control (if it exists) works. (2) T038 applied DCA mutes only when cfg/dcamgrp is on — verify WING semantics (manual/WING-Edit behaviour/schema descriptions) — DCA mute most likely always mutes members; dcamgrp likely means something else. Fix both, with regression tests reproducing each bug first.

## Acceptance criteria
- [ ] Regression tests fail before / pass after
- [ ] mon/1 host tap audible after a WING-Edit PC->Mixer push of the user's scene
- [ ] DCA mute mutes members (and unmute restores) per WING semantics, documented

## Log
- 2026-10-25 coordinator: created from user report.
- 2026-10-25 user (real WING owner): DCA mute always mutes members' main sends; with cfg/dcamgrp ON it also mutes bus/pre-fader sends (acts as a mute group).
