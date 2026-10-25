---
id: T05Q
title: EQ/dyn FX: GEQ, PIA(FX), LIMITER, DE-S2, DEQ3, C5-CMB, SPKMAN + FX-rack EQ models
stage: S5
status: done
owner: agent-T05Q
depends: [T050, T04H, T04F]
branch: task/T05Q
---

## Goal
FX-rack versions reuse channel EQ/dyn blocks where identical.

## Acceptance criteria
- [x] Tests per model

## Log
- 2026-10-25 coordinator: created by decomposing epics T040-T044/T051 after T004.
- 2026-10-25 agent-T05Q: done, ready for review.
  - **Oracle**: WING-Edit has `ae::fx_parchg_<model>` for every FX model (the DSP parameter block it posts to the
    engine). New `tools/curve-extract/fx_eqdyn.py` (Unicorn, never native) hooks the dummy engine's post method
    and records GEQ, the 7 FX EQs and SPKMAN's linear part -> `data/reference-curves/fx-eqdyn.json` (200 kB,
    deterministic). Write-up: `docs/research/reference-curves.md` §5b.
  - **Code**: `crates/wing-blocks/src/models/fx_eqdyn/` (`mod.rs` FxEq + shared DF1 `Section`, `geq.rs`,
    `limiter.rs`, `deesser.rs`, `deq3.rs`, `combinator.rs`, `spkman.rs`, `tests.rs`); `wingcoef::hqhs12q25` +
    its tables (port of `Cbiquad::hqhs12q25`); registration in `model.rs`, `pub mod` in `models/mod.rs`, bench
    `benches/fx_eqdyn.rs`, `tests/fx_eqdyn_no_alloc.rs`.
  - Exact: FX SOUL/E88/E84/F110/PULSAR/MACH4/PIA = channel `AltEq` with slot 1 `mix` as gain scale (FX parchg
    calls/inlines the channel `eq_parchg_*` with mix/100; all 84 recorded sets match); GEQ = port of
    `fx_parchg_geq` (STD/TRU/TRIM, coefficients < 1e-6); SPKMAN HP/LP (`Cxover48`), tilt, phase all-pass +
    invert and delays match all 36 recorded sets exactly. DEQ3 = 3 channel `DeqBand`s (same posted layout).
  - Designed [I] (detector code is SHARC firmware): LIMITER = look-ahead brickwall (running-min + A-tap moving
    average, Catmull-Rom inter-sample peak estimate, soft knee centred on the ceiling, squeeze = faster release
    below 3 dB GR, auto gain off = output compensated by -gin), constant latency 1 ms + 2 = 50 samples @48k;
    DE-S2 = SPL-style band/total energy ratio (female 6.5k/11k, male 5k/8.5k, 0.4 dB per amount unit, M/S mode);
    C5-CMB = fixed WING crossovers 79.6/317/1262/5024 Hz LR24/LR48 with all-pass compensation (sum flat),
    per-band RMS comp, auto release, SBC (50 % pull to the mean GR), width, mix via all-passed dry, band solo;
    SPKMAN limiter = RMS(x sqrt2)/peak, infinite ratio, `limrel`.
  - **Engine impact**: every FX slot's constant latency (max over hosted models, T03E) is now 50 samples
    (LIMITER). Updated `crates/wing-engine/tests/fx.rs` (expected delays + switch-off crossfade timing include
    `slot_latency`).
  - Bench (64-sample stereo blocks, M-series): GEQ 4.6 us, PIA 0.83, SOUL/PULSAR/MACH4 0.53, LIMITER 1.5,
    DE-S2 0.58, DEQ3 3.9, C5-CMB 7.3, SPKMAN 2.8.
  - Test: `make check` (fmt, clippy -D warnings, all workspace tests) passes; `cargo test -p wing-blocks
    fx_eqdyn`; regenerate oracle: `lipo -thin arm64 .../WING-Edit -output we-arm64; VENV/bin/python -I
    tools/curve-extract/fx_eqdyn.py we-arm64 data/reference-curves/fx-eqdyn.json 12`.
  - Assumptions for VERIFY-ON-HARDWARE (coordinator): LIMITER knee/squeeze/auto-gain semantics and 50-sample
    latency; DE-S2 band centres, onset -15 dB, slope 1.5, 20 dB max, 2 ms detector, 0.5/40 ms; C5-CMB detector
    (2 ms RMS, hard knee), auto-release law, SBC law, width = side x (1 + w/50), dry via all-passes; SPKMAN
    limiter (5 ms RMS, instant peak attack), DEQ3/SPKMAN DEQ key = own input; FX `mix` 0..125 % = gain scale.
  - Proposed follow-ups: (1) decide whether one FX model's look-ahead should cost every FX insert 50 samples
    (per-model FX slot latency or engine align target); (2) `fx_eqdyn.py`'s post hook works for every
    `fx_parchg_*` - other FX tasks can record exact parameter mappings the same way; (3) record the DE-S2 /
    C5-CMB / LIMITER / DEQ3 posted constants over parameter grids to pin down ranges/scales.
- 2026-10-25 agent-T05Q: merged main (T05D/T05M/T05E/T05R). Conflicts: additive lines in model.rs, models/mod.rs,
  Cargo.toml (union); `crates/wing-engine/tests/fx.rs` taken from main, whose expectations already derive every
  insert delay from `FxRack::slot_latency` (= max latency over the slot's registered models) - no hard-coded slot
  latency left (doc comment de-numbered). Slot latency is now max(T05E 46, LIMITER 50) = 50. `make check` passes.
