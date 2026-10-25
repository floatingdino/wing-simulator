---
id: T007
title: Research: reusable DSP libraries + licensing
stage: S0
status: done
owner: research-agent
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
- 2026-10-25 research agent: surveyed Rust/C++ DSP libs, licenses (GPL/LGPL flagged), mapped WING families to sources, recommended rubato + PI drift control and halfband/ADAA oversampling. Output docs/research/dsp-reuse.md. Some licenses marked (verify) (Signalsmith, Faust libs, chowdsp_utils, NAM) as not confirmed online. Files changed: docs/research/dsp-reuse.md. Proposed follow-ups: verify licenses; THIRD_PARTY_LICENSES + CI check.
