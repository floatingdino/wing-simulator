---
id: T04D
title: FET comps: 76LA, NSTR
stage: S4
status: done
owner: agent-T04D
depends: [T045, T034]
branch: task/T04D
---

## Goal
1176-style and Distressor-style incl. nonlinear coloration with oversampling; knob-position maps from WING-Edit curves.

## Acceptance criteria
- [x] Static/time tests; THD profile documented

## Log
- 2026-10-25 coordinator: created by decomposing epics T040-T044/T051 after T004.
- 2026-10-25 agent-T04D: started on branch task/T04D (deps T045, T034 done).
- 2026-10-25 agent-T04D: **review**. `76LA` and `NSTR` implemented as `FetComp` (`FetKind::{La76, Nstr}`) and
  registered in `create_model`.
  - Design: stereo-linked peak detector; audio 2x oversampled (`Oversampler2x`); control path (static curve +
    dB ballistics) at 16x on a half-band-interpolated key (`Oversampler2x` + `Oversampler4x`); GR decimated by an
    83-tap Kaiser FIR and aligned by a 36-sample (2x-rate) audio delay. Latency 57 samples (39 + 18). Ballistics
    in f64 (a 3.5 s release at 768 kHz is below f32 resolution). Why 16x: measured worst aliasing (10 kHz,
    76LA att/rel 7) was -68 dBc with the detector at 2x, -72 dBc at 8x with a linearly interpolated key, -86 dBc
    at 8x with a half-band key, and <= -100 dBc at 16x.
  - Colouration (at 2x, polynomials of order <= 3): 76LA FET H2 growing with GR (x3 in ALL), class-A output
    stage (H2 + soft cubic, +12 dBFS headroom); NSTR LF-weighted (300 Hz one-pole) H2 + H3 scaled per ratio,
    soft output stage (+18 dBFS). 2 Hz DC blocker after the even-order terms.
  - Sources per constant (also in the doc comments of `comp_fet.rs`): 1176LN attack 20..800 µs, release
    50..1100 ms, 7 = fastest, ratio set 4/8/12/20/ALL (UA 1176LN manual specs; models.md §3.2); "soft knee at
    4:1, harder at 20:1", ALL "over-compression with nasty knee and changed bias", "transient overshoot",
    programme-dependent release, distortion at fast settings (models.md §3.2); Distressor attack ~50 µs..30 ms,
    release ~50 ms..3.5 s, ratio set 1.5..20 + NUKE, soft "opto-like" low ratios, 10:1 = 1176, Dist modes not
    exposed (models.md §3.2, EL8 manual); slot order, ranges, defaults and enum tokens
    (data/schema/wing-tree.json); GR meter 76LA `-GR/20` capped at -1, NSTR `GR/20` capped at 1.3
    (data/reference-curves/dyn-models.json, `comp_gain_u1176` / `comp_gain_distress`). Everything else (knob dB
    laws, threshold, threshold offsets, knee widths, ALL dip/overshoot/bias, PD and opto release shapes, all
    colouration amounts) is an assumption, listed in docs/VERIFY-ON-HARDWARE.md.
  - Not modelled because WING does not expose them (models.md): Distressor 1:1, Dist 2/3, detector HP / band
    emphasis, release-mode switch. NSTR ratios follow the WING enum (1.5:1 first, no 1:1).
  - The ratio slot is a string enum, so every numeric `ParamValue` form is read as its index (the engine pipeline
    sends `Float(index)`; wing-engine `scene_recall_30k...` caught the first version).
  - THD profile (H2..H5, -6 dBFS peak; 100 Hz / 1 kHz / 5 kHz): 76LA no GR 0.013/0.013/0.013 %; 8:1 slow
    (8.7 dB GR) 0.39/0.38/0.38 %; 8:1 fast 1.5/0.41/0.38 %; ALL fast 2.3/0.86/0.83 %; NSTR 1.5:1
    0.30/0.024/0.014 %; 4:1 0.30/0.024/0.006 %; 10:1 0.40/0.033/0.006 %; NUKE 0.47/0.039/0.006 %.
  - Bench (`cargo bench -p wing-blocks --bench comp_fet`, M1, stereo 64-sample block): 76LA 15.5-16.7 µs,
    NSTR 15.9-17.6 µs (~1.3 % of a core per instance).
  - Files: crates/wing-blocks/src/models/comp_fet.rs (new), crates/wing-blocks/src/models/mod.rs,
    crates/wing-blocks/src/model.rs (registration; registry max latency is now 57), crates/wing-blocks/Cargo.toml
    ([[bench]] comp_fet), crates/wing-blocks/benches/comp_fet.rs (new), crates/wing-blocks/tests/comp_fet.rs
    (new), docs/VERIFY-ON-HARDWARE.md.
  - Tests: `cargo test -p wing-blocks comp_fet` covers knob maps vs published endpoints; static curves per ratio
    (DC, colouration off, both models, incl. ALL dip and NUKE) plus a tone through colouration within 0.3 dB;
    attack/release 10-90 % = tau ln 9 within 2 % (76LA knobs 1/4/7, NSTR 0/5/10); programme-dependent, opto and
    ALL overshoot behaviour; aliasing < -90 dBc at 10 kHz for every ratio x fastest/default/slowest timing; THD
    profile bounds; latency 57 and external key; GR meter vs dyn-models.json; zero allocation. `make check` passes
    (the comp_fet unit tests take ~30 s in the debug profile).
  - Proposed follow-ups: (1) wing-engine `ModelSlot` does not delay its dry path by the model latency, so
    `mix` < 100 % with 76LA/NSTR (57 samples) comb-filters (PUBLIC.snap uses NSTR at mix 80). Compensate the dry
    path in the slot. (2) Capture the 76LA/NSTR `*_parchg_*` coefficient blocks from WING-Edit (reference-curves
    follow-up "hook CdummyAudioEngine") to replace the knob-law and threshold assumptions. (3) Optional SIMD for
    the 16x control path if many instances are needed.
