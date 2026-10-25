---
id: T04J
title: EQ mix law for all EQ models + 44.1 kHz coefficient verification
stage: S4
status: review
owner: T04J agent
depends: [T04G, T04H, T03C]
branch: task/T04J
---

## Goal
WING's EQ `mix` scales band gains (T03C finding), but the model slot host applies dry/wet for alternate EQ models. Apply the gain-scale mix law uniformly to STD and every alternate EQ (incl. FX-rack EQ variants later), and verify all EQ/filter coefficient code at 44.1 kHz by driving the WING-Edit emulation (tools/curve-extract) at 44.1k and comparing.

## Acceptance criteria
- [x] Mix law identical to WING for all EQ models (tests vs emulated WING-Edit at several mix values)
- [x] 44.1 kHz reference curves generated and all EQ/filter models within 0.01 dB

## Log
- 2026-10-25 coordinator: created from T04G/T04H follow-ups.
- 2026-10-25 T04J agent: **done, ready for review.**
  - Mix law (disassembly of `ae::Cequalizer::update`): for ch/bus/aux it calls `f_table[mdl].parchg(idx, slots, mask,
    usage, dsp)` with `s0 = mix/100.0f`, one call site per usage shared by STD and all alternate models; each parchg
    scales its knob gains (ported in T03C/T04G/T04H). No dry/wet anywhere.
  - `ModelSlot` (crates/wing-engine/src/params/strip.rs): every `*/eq` group now defaults to the gain-scale law
    (mix 0..125 %, wet = on, `set_gain_scale` to all instances); other block kinds keep dry/wet. Fixed `settle()`
    resetting the wet ramp to mix% even for EQ slots. channel.rs no longer opts in by hand.
  - `AltEq` (eq_alt/mod.rs): implements `Model::set_gain_scale(mix %)` (was an inherent method the host never
    called, so alternate EQs ignored mix); the scale glides like a knob and survives `load_defaults`.
  - Filter tool (models/filter_tool.rs): 44.1 kHz showed T046's 48 kHz fit was off (0.54 dB); replaced with a port of
    the tool part of `ae::Cchfilter::update` (TILT = 10^(t/20)*hs12(fc, -2t, 0.2); MAX/AP1/AP2 exact). Now exact at
    48 and 44.1 kHz (0.0005 dB, 0.0000 deg).
  - 44.1 kHz references: tools/curve-extract/curves44k1.py -> data/reference-curves/44k1/ (392 kB, 215 curves,
    deterministic). docs/research/reference-curves.md §5a.
  - Tests: eq_alt `fidelity_44k1_with_mix` (coefficients bit-exact, 0.00005 dB), `gain_scale_glides_and_settles`;
    t03c_fidelity `*_44k1*` (filter LC/HC, STD ch/aux/bus incl. mix: max 0.00005 dB); filter_tool_reference
    `tool_filters_vs_wing_edit_44k1`; wing-engine tests/eq_mix_law.rs (every EQ slot is gain-scale with 125 % max;
    processed slot impulse responses at 44.1 kHz vs WING-Edit at mix 0..125 %: alt 0.0002 dB, STD 0.002 dB).
  - Files: crates/wing-blocks/src/models/{eq_alt/mod.rs,eq_alt/tests.rs,filter_tool.rs},
    crates/wing-blocks/tests/{t03c_fidelity.rs,filter_tool_reference.rs}, crates/wing-engine/src/{channel.rs,
    params/strip.rs}, crates/wing-engine/tests/eq_mix_law.rs, tools/curve-extract/curves44k1.py,
    data/reference-curves/44k1/*, docs/research/reference-curves.md.
  - How to test: `make check` (passes). Regenerate data: `VENV/bin/python -I tools/curve-extract/curves44k1.py
    --app WING-Edit.app --out data/reference-curves/44k1`.
  - Proposed follow-ups: (1) FX-rack EQ variants (SOUL..MACH4 as FX, `f_table_all`) should reuse `AltEq` with the same
    `set_gain_scale` once T050/T051 host them (verify their parchg call passes mix the same way). (2) 44.1 kHz for
    dynsc/dynxo key filters. (3) bus/main/mtx strips are not yet built in the engine; their `*/eq` slots will get the
    law automatically via `ModelSlot::from_group`.
