---
id: T090
title: Calibrate WARM (Soul Warmth Pre) against the user's real WING
stage: S4
status: todo
owner: -
depends: [T088, T072]
branch: -
---

## Goal
User: WARM is now audible but "definitely a lot lower than the model on the real console". Add a WARM calibration set to tools/verify-hardware (wing-verify gen/compare): scenes with WARM in a channel's gate slot at mix 100 %, drive sweep (e.g. 10/33/66/100/125 %), harmonics/colour at default and extremes, trim 0; test signals: 1 kHz sines at -36/-24/-18/-12/-6 dBFS, a 100 Hz sine, two-tone IMD, and a short vocal-like excerpt; a quick tier under 5 minutes of capture. Add `wing-verify fit-warm <recordings>` that measures THD/harmonic profile, gain and compression vs level/drive from the recordings and fits WARM's law parameters (drive gain curve, bias/harmonic balance, make-up) to them, writing the fitted constants to a data file the model loads (data/calibration/warm.json) with a test that the fitted model reproduces the recordings' THD within ±2 dB per point. Until recordings exist, keep current constants. Document the procedure in tools/verify-hardware/README.md (WARM section, ~5 min).

## Acceptance criteria
- [ ] Calibration scenes + signals generated; README steps
- [ ] fit-warm works on synthetic recordings (test) and updates data/calibration/warm.json
- [ ] After the user's capture: fitted model within ±2 dB THD per point

## Log
- 2026-10-26 coordinator: created from user feedback.
