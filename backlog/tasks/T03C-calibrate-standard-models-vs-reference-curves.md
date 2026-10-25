---
id: T03C
title: Calibrate standard models vs WING-Edit reference curves + dynxo/key listen
stage: S3
status: todo
owner: -
depends: [T00A, T031, T032, T033, T034]
branch: -
---

## Goal
Validate and correct the standard models (filter, GATE/DUCK, STD EQ, COMP/EXP) against data/reference-curves (T00A). T034 guessed: knee width (2 dB/step), RMS window, LIN release rate, AUTO release constants, 90 dB expander floor, make-up applied to wet path only — confirm or fix each, citing evidence. Also implement the dynamics crossover/sidechain EQ `dynxo` and key listen (solo of the sidechain signal) per models.md.

## Acceptance criteria
- [ ] Every guessed constant confirmed or corrected with evidence
- [ ] Fidelity tests pass against reference curves within tolerance
- [ ] dynxo + key listen implemented and tested

## Log
- 2026-10-25 coordinator: created from T034 follow-ups.
- 2026-10-25 coordinator: also from T031/T032 — gate ACCENT (guessed 0–10 dB hysteresis), Butterworth cut shape, input balance law and gate detector timing are unverified assumptions.
