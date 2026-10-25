---
id: T05V
title: Vintage reverbs: V-ROOM, V-REV, VSS3
stage: S5
status: review
owner: claude-T05V
depends: [T05R]
branch: task/T05V
---

## Goal
Lexicon 224/480-/TC-style per models.md §6.

## Acceptance criteria
- [x] Decay tests (T60 vs `dcy` and every band multiplier within +-10 %)

## Log
- 2026-10-25 coordinator: created by decomposing epics T040-T044/T051 after T004.
- 2026-10-25 claude-T05V: implemented V-ROOM, V-REV, VSS3 on the T05R FDN core. Done, status review.
  - Emulated units per models.md: V-ROOM = Quantec QRS-style room, V-REV = EMT 250-style (both "vintage", Lexicon
    224-lineage topology: allpass loops, chorused lines, bass/mid/treble decay), VSS3 = TC VSS3-style ER generator
    + 4-band-decay tail. Every WING schema param mapped (slot order from data/schema; VSS3 `preset`/`load`/`view`
    are UI actions, documented as no DSP effect). Mappings marked [I] in the module docs.
  - Vintage character: input 4th-order Butterworth band-limit at the unit bandwidth (QRS 15 kHz, EMT 12 kHz [I])
    and output HI CUT clamped to it (aliasing-free "low internal SR"); V-ROOM FREEZE (T60 3000 s, input muted)
    + ADD (layer input into the frozen room), ER L/R first reflections, LEVEL; V-REV FRONT/REAR tap sets,
    XFORMER soft saturation, MOD SPD chorus.
  - Core extensions (crates/wing-blocks/src/models/fx_reverb/core.rs, all optional; T05R behaviour unchanged):
    `CoreParams::{xover_hz, hf_ratio, mid_split, rear}` (crossover / HF-ratio overrides, 4th decay band = one more
    high shelf per line, rear output tap set), shelf corners kept ordered (stability), `MAX_GAIN` 0.9995 ->
    0.99995 and decay clamp 200 -> 5000 s (freeze headroom). concert.rs map gained `..CoreParams::default()`.
  - Files: fx_reverb/{vintage.rs, vss3.rs, tests_vintage.rs} (new), fx_reverb/{core.rs, mod.rs, concert.rs,
    tests.rs (helpers pub(super))}, model.rs (registration; stub test uses `NONE`), fidelity.rs (stub test uses
    `NONE`), Cargo.toml ([[bench]] fx_reverb_vintage), benches/fx_reverb_vintage.rs,
    tests/fx_reverb_vintage_no_alloc.rs.
  - Test: `cargo test -p wing-blocks --lib fx_reverb` (32 tests incl. 16 vintage: T60 +-10 % vs dcy / size /
    rvtype / lmult / hmult / VSS3 4-band, freeze hold (-0.1 dB over 3 s) + mute + add, first reflections, level,
    rear outputs, transformer, band-limit, VSS3 ER onset/size/pos/type/balance, echo density, IACC, silence ->
    exact zero, extreme settings incl. reversed crossovers, click-free sweeps of every param, round trip, chunked
    long blocks); `cargo test -p wing-blocks --test fx_reverb_vintage_no_alloc`; `make check` passes;
    `cargo test --workspace --release -- --include-ignored` passes.
  - Bench (`cargo bench -p wing-blocks --bench fx_reverb_vintage`, M-series, 64-sample stereo block): V-ROOM
    6.4 us, V-REV 4.5 us, VSS3 9.9 us (HALL 5.9 us unchanged).
  - Known limits: an HF T60 longer than the mid one (V-ROOM `hmult` > 1, VSS3 `hdcy` > others) is capped in the top
    octaves by the core's Lagrange interpolation loss (within 10 % up to ~6 kHz in large rooms); slope-1 shelves
    need ~2 octaves between VSS3 crossovers for a band to reach its multiplier.
  - Proposed follow-ups: (1) compensate the interpolation HF loss in the per-line absorbent filter (core);
    (2) VSS3 preset library as data (109 presets) applied by the control layer on `load`; (3) verify ER tables /
    bandwidths / multiplier corners against a console recording.
