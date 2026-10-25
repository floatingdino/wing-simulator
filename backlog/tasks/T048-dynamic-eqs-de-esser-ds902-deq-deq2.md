---
id: T048
title: Dynamic EQs + de-esser: DS902, DEQ, DEQ2
stage: S4
status: review
owner: agent-T048
depends: [T045, T033]
branch: task/T048
---

## Goal
Per models.md §3.1.

## Acceptance criteria
- [x] Band/threshold behaviour tests

## Log
- 2026-10-25 coordinator: created by decomposing epics T040-T044/T051 after T004.
- 2026-10-25 agent-T048: implemented DS902, DEQ and DEQ2 in `crates/wing-blocks/src/models/dyn_eq.rs` and
  registered them in `model.rs`. Slot order, ranges and defaults come from `data/schema/wing-tree.json`
  (DEQ2 defaults are 1-f 200 Hz and 2-f 2.99 kHz; models.md lists 150 Hz). Design and constant sources are in the
  module doc.
  - **WING EQ band laws recovered** from the WING-Edit reference biquads (`eq-std-ch.json`) by fitting the
    pole/zero polynomials of the bilinear transform:
    - The peak boost is `(s²+sG/q+1)/(s²+s/q+1)` and the cut is its exact inverse. This is RBJ with
      Q_rbj = q·min(A, 1/A).
    - The shelf is a mirrored Butterworth shelf that ignores Q. The cut corner is scaled in the warped (tan)
      domain by √G.
    - The STD high shelf clamps its corner to about 50 Hz.
    - Above about 1 kHz, WING de-cramps low-Q peaks: 1 dB at 1 kHz with Q 0.44, and several dB at 3–10 kHz.
    - DEQ reproduces the WING biquads within 0.1 dB from 50 to 300 Hz, within 0.4 dB at 30 Hz (float32 limit of
      the reference coefficients) and within 1 dB up to 1 kHz.
  - The dynamic band is a TPT SVF or TPT one-pole with output mixing. The control path runs every 8 samples
    (gain computer, ballistics and coefficient targets). Coefficients are interpolated per sample, so the
    gain changes without zipper noise and the filter stays stable.
  - The DS902 de-esser compares HF energy with total energy, so its action does not depend on input level.
    HF mode is a dynamic WING-law high shelf; FULL mode applies wide-band gain.
  - The assumptions are listed in `docs/VERIFY-ON-HARDWARE.md` (6 rows, T048).
  - Files: `crates/wing-blocks/src/models/{dyn_eq.rs,mod.rs}`, `crates/wing-blocks/src/model.rs`,
    `crates/wing-blocks/tests/dyn_eq_no_alloc.rs`, `crates/wing-blocks/benches/dyn_eq.rs`,
    `crates/wing-blocks/Cargo.toml` (`[[bench]] dyn_eq`), `docs/VERIFY-ON-HARDWARE.md`.
  - Tests: `cargo test -p wing-blocks dyn_eq` runs 14 unit tests and the no-alloc integration test. They cover:
    - static band gain vs level (above and below the threshold);
    - shape laws (cut = 1/boost, 0 dB = identity);
    - a comparison against the WING reference biquads;
    - attack and release times (including att 0 and low mode);
    - stability under per-block f/Q/shape slamming;
    - zipper-free modulation (second-difference bound);
    - DEQ2 series and band independence;
    - listen, OFF as wide band, and params/defaults/clamps;
    - DS902: transparency, level independence, timing, HF vs FULL, and the external key.
  - Results: `make check` passes. Bench (`cargo bench -p wing-blocks --bench dyn_eq`, Apple M1, stereo 64-sample
    block, gain moving): DEQ 0.89 µs, DEQ2 1.82 µs, DS902 1.07 µs, DS902 FULL 0.80 µs.
  - Proposed follow-ups:
    - Drive `DynEqGraph::setDynDeq*` in WING-Edit (reference-curves.md §6) to confirm the DEQ shapes, the LP6/HP6
      laws and whether DEQ applies the 50 Hz clamp.
    - FX `DEQ3` (T050/T051) can reuse `DeqBand` directly (three bands in series).
- 2026-10-25 agent-T048 (coordinator request): merged main into task/T048. The T03C `wingcoef` is now on main
  and resolves the STD EQ follow-up above. I resolved the `model.rs` and `Cargo.toml` conflicts per the
  model-family convention: one `create_model` line per model, my own line in the implemented list, and a
  separate `[[bench]] dyn_eq`.
  - The DEQ, DEQ2 and DS902 sections now come from `crate::wingcoef`, called as `eq.rs::wing_band` calls it:
    - BP uses `peq`;
    - LP12 uses `ls12` at Q 0.707, with f truncated and capped at 8 kHz;
    - HP12 uses `hqhs12`, with f truncated and floored at 50 Hz;
    - LP6 and HP6 use `hqls6` and `hqhs6`;
    - the side-chain uses `bp`, `lp12`, `lp6`, `hp12` and `hp6`;
    - the DS902 detector uses `hp12` and the DS902 HF mode uses `hqhs12`.
  - Runtime: the TPT SVF is replaced by an f64 DF1 biquad, so the WING coefficients are used as they are. The
    8-sample recompute and the per-sample linear coefficient blend are kept. A band within 0.001 dB of 0 dB is
    the identity section.
  - Fidelity: against `eq-std-ch.json`, 476 cases (mid PEQ, low SHV, high SHV) over 20 Hz–20 kHz have a max
    error of 0.00605 dB. The gain ±1 ulp read-back convention follows T03C. The old fit had errors of 1 dB at
    1 kHz and several dB at 3–10 kHz.
  - New test `deq_runtime_settles_on_the_wing_section`: the running band's impulse response at full dynamic
    gain matches the section within 0.01 dB. The stability, zipper and timing tests still pass on DF1.
  - Results: `make check` passes. Bench: DEQ 1.56 µs, DEQ2 3.25 µs, DS902 0.68 µs, DS902 FULL 1.2 µs (noisy),
    per stereo 64-sample block.
