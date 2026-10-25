---
id: T046
title: Filter-tool models: TILT, MAX, AP1, AP2
stage: S4
status: review
owner: agent
depends: [T045, T031]
branch: task/T046
---

## Goal
Implement the 4 filter-tool models per docs/research/models.md §2 (MAX = BBE Sonic Maximizer-style phase/level processor).

## Acceptance criteria
- [ ] Response tests vs reference curves
- [ ] Bench per model

## Log
- 2026-10-25 coordinator: created by decomposing epics T040-T044/T051 after T004.
- 2026-10-25 agent: DONE, status review. Files: crates/wing-blocks/src/models/{mod,filter_tool}.rs (new), lib.rs (+`pub mod models`), model.rs (4 factory lines + import + test tweak), tests/filter_tool_reference.rs, benches/filter_tool.rs, Cargo.toml (+bench).
  Findings: WING-Edit computes each tool filter as a single biquad (4th section). TILT = 2 real pole/zero pairs in the K-plane (pivot ~656 Hz, K1/K2 fitted; positive tilt = bass boost, DC=+tilt dB, Nyquist=-tilt dB, opposite to the "brighter" assumption in eq.rs docs). MAX = fixed real-pole allpass (~161 Hz, ~3.19 kHz) + proc*kappa*(1-z^-1)^2 + low*c numerator terms, exact and linear in both knobs; WING's MAX is fully LTI (no level-dependent BBE VCA behaviour). AP1 = pre-warped 1st-order allpass; AP2 = RBJ allpass with alpha = tan(w0/(2Q)) (not sin). f64 section, 10 ms per-sample coefficient ramp, no alloc.
  Tests: `cargo test -p wing-blocks --test filter_tool_reference` vs all 56 tool curves (release): worst mag TILT 0.0023 dB, MAX 0.001 dB, AP1/AP2 ~0; phase <=0.18 deg (float32 reference coeffs at 100 Hz). `make check` passes. Bench (64-sample stereo block): ~177 ns static, ~240 ns with per-block automation.
