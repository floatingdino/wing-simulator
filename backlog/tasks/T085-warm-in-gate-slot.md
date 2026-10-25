---
id: T085
title: BUG: Soul Warmth Pre drive inaudible in the GATE slot (VOX channels)
stage: S4
status: review
owner: claude-T085
depends: [T081]
branch: task/T085
---

## Goal
User (v0.5 build): WARM drive still inaudible when WARM is loaded in the gate slot on VOX channels (e.g. ch 21). Live log scratch/wing-sim-v0.5.log shows `ch/21/gate/1 = 125 %` etc. T081 only tested the dyn slot (`ch/7/dyn/1`). Reproduce exactly via `ch/21/gate/mdl = WARM` + `ch/21/gate/1..n` through the full pipeline, fix the gate-slot binding/processing (and check every model is processed correctly when hosted in the gate slot vs dyn slot — both slots offer the same 32 models), extend the T081 param audit to cover both slots for every dynamics model.

## Acceptance criteria
- [x] Regression test fails before / passes after
- [x] Param audit covers gate AND dyn slot for all 32 models

## Log
- 2026-10-26 coordinator: created from user v0.5 test.
- 2026-10-26 claude-T085: done, status review.
  - **Reproduced** through the exact path (ParamStore -> pipeline -> ChannelStrip bound as `ch/21` -> gate
    ModelSlot -> `WarmthPre`) with the user's ch 21 state: `gate {on, mdl WARM, mix 5, drv 100, hrm 100,
    col 0.1, wmix 5}`, then the log's sets `gate/5 = 100`, `gate/1` 10 -> 125: output changed by -49.5 dB on a
    -18 dBFS 220 Hz tone (inaudible); with `gate/mix = 100` -23 dB. The gate slot's binding is correct (model
    index 12, slot 1 = drv, readback 125).
  - **Root cause**: double dry/wet. The gate block's own `mix` was 5 %, so the slot blended WARM at 5 % on
    top of WARM's `wmix`; drive/wmix changes were scaled by 0.05. Where the 5 % comes from: the v0.5 scene was
    loaded with `gate/mix = 100` (T019 `warm-mix`), then WING-Edit's connect push (28718 sets) wrote its own
    state with `gate/mix = gate/wmix = 5` (ch 20-22; 30/30 on ch 23-24). WING-Edit 3.3.4's legacy-scene upgrade
    does exactly that: x86_64 code at 0x10026b026 `if strcasecmp(gate.mdl, "WARM") == 0 && gate.mix: gate.wmix
    .copyFrom(gate.mix)` and keeps the block `mix`. So every Behringer-upgraded WARM carries mix == wmix, and
    applying both squares it (5 % -> 0.25 %). T081 only tested `ch/7/dyn` at block mix 100 %.
  - **Fix**: WARM's MIX (`wmix`) is the block MIX. New `Model::owns_mix()` / `RtModel::owns_mix()` (default
    false; `WarmthPre` true); `ModelSlot` uses `w = on` (no block-mix blend) while such a model is selected, in
    every slot that hosts it (ch gate/dyn, bus/main/mtx dyn). Block `gain`/`on` unchanged. Logged in
    VERIFY-ON-HARDWARE (does a console blend WARM once or twice with block mix 5 %?).
  - **Audit extension** (crates/wing-engine/tests/param_audit.rs): `warm_in_gate_slot_drive_is_audible_with_
    upgraded_block_mix` (user's ch 21 state, gate and dyn; before -35 dB on the programme, after -9 dB; and
    block mix 5 vs 100 bit-identical for WARM); `gate_and_dyn_slot_parity_sampled` / `full_gate_and_dyn_slot_
    parity` (all 32 models in `ch/1/gate` vs `ch/1/dyn`, every param written explicitly, defaults + range ends:
    bit-identical; full sweep in check-full); `slot_wrapper_audit_gate_and_dyn` (both slots, all 32 models:
    `gain` live, block `mix` live except WARM where it must be inert). The existing per-param audit already ran
    `ch/*/gate` and `ch/*/dyn` (186 params each) but with the block wrapper at defaults, so it could not see this.
  - Parity findings (not bugs): (1) the factory scene's dyn values differ from pardef defaults a freshly
    selected gate model gets (COMP `rel` 150.33 vs 150 ms): store state, test writes every param; (2) a slot
    behind the other (idle) slot gets its input delayed by that slot's constant latency (57 samples), which
    shifts block-rate control updates (PSE/CMB/DS902 detectors -39..-56 dB, LA/L100 ~-170 dB); with the hosting
    slot first in `proc` (GEDI vs DGEI) both slots are bit-identical for all 32 models.
  - Files: crates/wing-blocks/src/model.rs, crates/wing-blocks/src/models/dyn_special.rs,
    crates/wing-engine/src/params/strip.rs, crates/wing-engine/src/params/std_models.rs,
    crates/wing-engine/tests/param_audit.rs, docs/VERIFY-ON-HARDWARE.md.
  - Test: `make check` (pass); `cargo test -p wing-engine --release --test param_audit -- --include-ignored
    full_gate_and_dyn_slot_parity full_ch_gate full_ch_dyn` (pass).
  - Proposed follow-ups: the user's running session still holds `ch/20-24/gate/mix` = 5/30 % from WING-Edit's
    push; with this build it no longer matters for WARM, but a non-WARM model loaded there would inherit it (as
    on a console). Verify the single-blend assumption on hardware (VERIFY-ON-HARDWARE row).
