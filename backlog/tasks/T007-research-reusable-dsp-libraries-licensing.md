---
id: T007
title: Research: reusable DSP libraries + licensing
stage: S0
status: todo
owner: -
depends: []
branch: -
---

## Goal
Survey permissive open-source DSP to reuse for WING blocks/models (Rust: fundsp, biquad, dasp, rubato for resampling, coreaudio-rs/cpal; C/C++: Airwindows (MIT), Faust libraries (various), Freeverb/Dattorro reverbs, CHOW DSP (BSD/GPL check), Signalsmith DSP (MIT), DaisySP (MIT), Surge/Vital FX (GPL - note), r8brain (MIT), libsamplerate). Map each WING block/model family to a recommended implementation source and license risk. Output docs/research/dsp-reuse.md.

## Acceptance criteria
- [ ] Table: model family -> recommended lib/algorithm -> license -> integration effort
- [ ] Recommendation for resampler (host bridge) and oversampling

## Log
- 2026-10-25 coordinator: created.
