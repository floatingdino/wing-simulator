---
id: T05R
title: Reverb core + HALL, ROOM, CHAMBER, CONCERT, AMBI
stage: S5
status: done
owner: claude-T05R
depends: [T050]
branch: task/T05R
---

## Goal
Shared high-quality FDN/algorithmic reverb core (RT-safe, SIMD) + these models per models.md §6.

## Acceptance criteria
- [x] Decay-time accuracy, stereo decorrelation, CPU per instance

## Log
- 2026-10-25 coordinator: created by decomposing epics T040-T044/T051 after T004.
- 2026-10-25 claude-T05R: done on branch task/T05R.
  - Core (crates/wing-blocks/src/models/fx_reverb/core.rs, `ReverbCore<N>`, N = 8/16): pre-delay ring -> 8-tap/side
    ER bank (window = size x SPREAD, gain envelope = SHAPE; also the tail's build-up) + echo stage (ROOM:
    recirculating lines with feedback; CHAMBER/CONCERT: plain taps with fader levels) -> 4 Schroeder input
    diffusers/side -> N-line FDN: Lagrange-3 modulated reads, per-line Jot gain x RBJ low shelf (bass mult @
    model crossover) x RBJ high shelf (HF T60 = hf_ratio x dcy above `damp`), in-loop allpass per line, fast
    Walsh-Hadamard feedback, orthogonal +-1 in/out vectors -> wet = ER + echo + tail (CONCERT: tail chorus) ->
    SVF LO/HI CUT. Frame-interleaved `Vec<[f32; N]>` delay memory + `[f32; N]` lane loops (vectorised); flush
    < 1e-18 in every recursive write, so silence decays to exact zero. Sources (algorithms only): Jot & Chaigne
    1991, Dattorro 1997, Signalsmith "Let's write a reverb" (MIT) - cited in the module doc.
  - Click-free params: one-pole block glides (80 ms) + per-sample ramps for times/levels/ER and line gains;
    echo time changes crossfade two taps (30 ms) instead of gliding (a 1.2 s jump would fast-forward the line).
  - Rejected on measurement: parallel `g_mid + dg*LP + dg*HP` decay filter (branch phases -> +10 % T60 at
    250 Hz with damping at 1.5 kHz); replaced by the shelf cascade.
  - Models (one file each, mapping table in each module doc, all [I]): HALL 16 lines, ROOM 8 (+recirculating
    echoes, fixed level 0.3, no level key), CHAMBER 8, CONCERT 16 (refl taps, depth, spin, chorus), AMBI 8
    (ER-dominant, `tail` = tail level). Registered in model.rs; `stub_is_passthrough` / fidelity stub test now
    use `V-REV`.
  - Files: crates/wing-blocks/src/models/fx_reverb/{mod,core,hall,room,chamber,concert,ambi,tests}.rs,
    models/mod.rs, model.rs, fidelity.rs (stub id), Cargo.toml ([[bench]] fx_reverb), benches/fx_reverb.rs,
    tests/fx_reverb_no_alloc.rs.
  - Test: `make check` passes. `cargo test -p wing-blocks --lib fx_reverb -- --nocapture` prints metrics:
    T60 (Schroeder, -5..-35 dB fit, octave bands) within +-4 % of `dcy` at 250/1k/2k Hz for all models and at
    sizes/decays 0.4..6 s, LF (`mult` 2, 100 Hz) within -5.5 %, HF (`damp` 1.5 kHz, 8 kHz) within +-3 %;
    normalised echo density 0.99..1.10 at 250 ms (AMBI at 80 ms); max |IACC| (+-1 ms) 0.14..0.28; output exactly
    zero 6..16 s after an impulse (no subnormals, state fully flushed); stable/finite at all-min/all-max params;
    every parameter swept min->max and jumped max->min on a 150 Hz sine: HF (>6 kHz) < -40 dB re level (worst
    -46 dB); zero allocations (tests/fx_reverb_no_alloc.rs + registry test). Wet/dry RMS at defaults -5..-12 dB.
  - Bench `cargo bench -p wing-blocks --bench fx_reverb` (64-sample stereo block, 48 kHz, defaults, M-series):
    HALL 5.9 us, ROOM 4.3, CHAMBER 4.1, CONCERT 6.6, AMBI 4.0 (target < 15 us).
  - Memory (48 kHz): ~0.8 MB (HALL) .. ~2 MB (CONCERT) per instance, x2 at 96 kHz; the engine preallocates one
    per model per full FX slot (~30 MB for 8 slots). `reset()` (model swap) memsets that on the RT thread
    (tens of us once).
  - Proposed follow-ups: tune characters/levels against console recordings (T072: T60 per band, ER patterns,
    output level, `shp`/`sprd`/`spin`/`depth`/`crs` tapers are all [I]); consider an idle detector that
    skips processing once input and tail are exactly zero; lazy/partial clear on model swap if the reset memset
    shows up in RT profiling.
