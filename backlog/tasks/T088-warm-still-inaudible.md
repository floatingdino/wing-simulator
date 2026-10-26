---
id: T088
title: BUG: Soul Warmth Pre distortion still inaudible live (125% drive, 100% mix, gate slot ch21)
stage: S4
status: todo
owner: -
depends: [T085]
branch: -
---

## Goal
User (latest build via menu app, 2026-10-26 10:15): on VOX channel 21 with WARM in the gate slot, drive 125 % and mix 100 %, the distortion is inaudible; the WARM trim knob audibly changes level. T081/T085 tests pass, so either (a) the live path differs from the tested path (wrong model actually active in the slot, block on/bypass state, the gate slot's wrapper mix still applied, scene values from WING-Edit's push e.g. `gate/on`, `wmix`, legacy `mix`), or (b) the saturation law is too gentle at real vocal levels (~-30..-18 dBFS RMS). Do both: add a debug introspection command (control socket + OSC) that dumps, for a strip, the active model per slot, its effective parameter values as the DSP sees them, and the measured in/out RMS and THD of the live block; use it against a reproduction loaded from the user's latest scene (scratch/live-mix-v0.6.snap or the autosave in ~/Library/Application Support/WingSimulator/state.snap) with realistic vocal-level programme. Then make WARM's character match the referenced SSL VHD-style "Soul Warmth" behaviour: drive adds clearly audible harmonics at nominal levels (document target THD vs drive at -18 dBFS, cite sources), keep level roughly compensated. Regression test at -24 dBFS RMS vocal-like input asserting audible THD (> ~1 %) at max drive.

## Acceptance criteria
- [ ] Root cause identified with evidence (path vs law)
- [ ] Audible saturation at realistic levels (test); introspection command available
- [ ] User confirms live

## Log
- 2026-10-26 coordinator: created from user retest.
