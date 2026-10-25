---
id: T03F
title: BUG: monitor level stuck at -inf; DCA mutes not respected
stage: S3
status: done
owner: engineering-agent
depends: [T038]
branch: task/T03F
---

## Goal
User report (live, v0.3 build, WING-Edit connected PC->Mixer, menu-bar app routing mon/1 -> Scarlett):
1. With the host tap on the monitor (mon/1) there is no audio: "the level fader doesn't move from -inf". Works with a main tap.
2. DCA mutes are not respected (muting a DCA doesn't mute its members).
Diagnose with evidence (logs in scratch/wing-sim-v0.3b.log or the current wing-sim log, the protocol fixtures, schema, docs). Hypotheses: (1) the monitor level node is a hardware-pot/$ node WING-Edit can't set, or the server rejects/ignores its sets, or the scene push sets it to -inf after wing-sim's startup override; decide WING-faithful behaviour (on a real WING the monitor level is the physical knob) — e.g. treat monitor A/B level as a simulator-side setting (default 0 dB, exposed in OSC/control socket/menu app) unless WING-Edit can legitimately set it, and make sure WING-Edit's monitor-level control (if it exists) works. (2) T038 applied DCA mutes only when cfg/dcamgrp is on — verify WING semantics (manual/WING-Edit behaviour/schema descriptions) — DCA mute most likely always mutes members; dcamgrp likely means something else. Fix both, with regression tests reproducing each bug first.

## Acceptance criteria
- [x] Regression tests fail before / pass after
- [x] mon/1 host tap audible after a WING-Edit PC->Mixer push of the user's scene
- [x] DCA mute mutes members (and unmute restores) per WING semantics, documented

## Log
- 2026-10-25 coordinator: created from user report.
- 2026-10-25 engineering agent: diagnosed + fixed both, status review.
  **Bug 1 (monitor level -oo) root cause**: `cfg/mon/1|2/lvl` ("PHONES"/"SPEAKERS") are the console's physical
  monitor knobs; they are not in snapshots, so WING-Edit's model holds -oo and its PC->MIXER push sends
  `cfg/mon/1/lvl = -144` and `cfg/mon/2/lvl = -144` (evidence: fixtures/protocol/pc-to-mixer-scene.json lines
  11383/11421, decoded from the WING-Edit 3.3.4 capture). The server applies them like any set (not rejected; no
  $-node involved), and wing-sim's 0 dB override (`audio::default_monitor_levels`) only ran once at startup, so the
  push at 3424 s in scratch/wing-sim-v0.3b.log ("28718 sets (489 changed)") silenced monitor A for good. Main taps
  were unaffected, matching the report. Reproduced: apps/wing-sim/tests/audio.rs
  `monitor_level_survives_wing_edit_push` (applies the fixture's cfg/mon sets with a client origin: -277 dB before).
  **Fix**: the knob owns the level. `wing_engine::mix::MonitorLevels` (lock-free, 2 knobs, -144..+10 dB) via
  `MixConfig::monitor_levels`: the monitor node takes its level from the knob and ignores `lvl` sets (no dip during
  a push). `wing_sim::audio::MonitorKnobs` (default 0 dB) is installed by `prepare`; `ConsoleControl::refresh`
  writes the knob back over any client set of `lvl` (WING-Edit reads the knob position, like a pot). Controls:
  control socket `monitor-level [<1|2|a|b> <dB|-inf>]`, OSC `/wingsim/mon/<1|2>/lvl [,f dB]` (query without arg),
  status.json `monitor_levels_db: [A, B]` (null with --no-audio; fixture + WingSimMenu DaemonStatus decode it).
  WING semantics: dedicated knobs set monitor A/B (phones/speakers) level (WING manuals); WING-Edit has no working
  monitor-level control in this model (its sets snap back) - logged in docs/VERIFY-ON-HARDWARE.md (with $lvl/$lvlact).
  **Bug 2 (DCA mute) root cause**: T038 applied DCA mutes only while `cfg/dcamgrp` was on; the user's scene (and
  PUBLIC.snap, and the WING-Edit push: `cfg/dcamgrp: 0`) has it off, so DCA mutes did nothing (log: dca/5/mute
  sets at 3545-3548 s had no effect). Reproduced: crates/wing-engine/tests/monitor.rs
  `dca_mute_mutes_main_sends_and_dcamgrp_makes_it_a_full_mute` (main not silent with dcamgrp off).
  **Semantics (user-confirmed on a real WING, via coordinator)**: a DCA mute always mutes its members' main (LR/main)
  sends; `cfg/dcamgrp` on makes it a mute group: also the bus sends incl. PRE sends (full mute). wing-docs.com
  (global audio settings) agrees on the "on" case. **Fix**: `logic::flag::MAIN_MUTE` (dcamgrp off) for ch/aux/bus
  members, cut by the main nodes on the member's send slot (ramped; PRE and POST main sends; bus/matrix sends, the
  strip itself, solo unaffected); dcamgrp on = `flag::MUTE` as before. Main/matrix members (no main send) are muted
  outright [I]. Docs: logic.rs module docs, signal-flow.md §9/§10, VERIFY-ON-HARDWARE.md (DCA row marked VERIFIED).
  Side effect: wing-render determinism test unmutes PUBLIC.snap's muted DCAs 5/8 (their group busses now lose their
  main sends, as on the console).
  Files: crates/wing-engine/src/mix/{logic,nodes,monitor,mod}.rs, crates/wing-engine/tests/monitor.rs,
  apps/wing-sim/src/{audio/mod.rs,audio/console.rs,control.rs,main.rs}, apps/wing-sim/tests/audio.rs,
  apps/wing-render/tests/determinism.rs, fixtures/control/status.json, apps/WingSimMenu (DaemonStatus + test),
  docs. Tests: `make check` passes; `swift test` in apps/WingSimMenu passes.
  Proposed follow-ups: persist the knob levels across restarts (state dir) and a menu-bar slider; the user's
  running v0.3 daemon needs a restart on this build (until then: `monitor-level` does not exist; setting
  cfg/mon/1/lvl via OSC to 0 after each WING-Edit push is the workaround).
