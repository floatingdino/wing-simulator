---
id: T081
title: BUG: Soul Warmth Pre drive ineffective + audit every model's parameter binding
stage: S4
status: review
owner: claude-T081
depends: []
branch: task/T081
---

## Goal
User: WARM (Soul Warmth Pre) drive knob appears ineffective. Fix it, then add a systematic audit test so this class of bug can't recur: for EVERY model in every block kind (filter-tool, gate, dyn, EQ, FX), drive the parameter path exactly as WING-Edit does (schema path set via ParamStore → params pipeline → ModelSlot → model.set_param slot index), sweep each parameter across its range, and assert the output changes measurably (or the param is documented as having no audible effect, e.g. a display-only key). Report any params that don't reach the DSP (slot-index mismatches, wrong scaling, enum mapping) and fix them.

## Acceptance criteria
- [x] WARM drive audibly changes output (test)
- [x] Audit test covers all models/params; all dead params fixed or justified
- [x] Report in task Log

## Log
- 2026-10-26 coordinator: created from user report.
- 2026-10-26 claude-T081: done, status review.
  - **WARM root cause** (reproduced through ParamStore `ch/7/dyn/1` -> pipeline -> ChannelStrip -> ModelSlot ->
    `WarmthPre::set_param(1)`): the binding was correct (slot 1 = `drv`, value arrives); the *drive law* was the bug.
    `g = 1.5 * drv/100` topped out at 1.9 with unity small-signal gain, so drive 10 -> 125 changed a -30 dBFS tone by
    -44 dB (inaudible) and a -12 dBFS tone by -24 dB. New law `g = 0.5 * 12^((drv - 33)/92)` (0.27 / 0.5 / 6 at
    10 / 33 / 125 %): default calibration unchanged (1.4 % THD at -12 dBFS), 125 % = 14 % THD at -12 dBFS, > 1 % at
    -30 dBFS; 10 -> 125 now changes -12 dBFS by -10 dB. Also reaches `*BUS*` `w_drv`. Alias bound at the extreme
    corner (125 %, 0.9 amplitude, 10 kHz) is now -48 dBc (< -100 dBc up to -20 dBFS); test bounds per case.
    VERIFY-ON-HARDWARE rows updated.
  - **Audit** `crates/wing-engine/tests/param_audit.rs`: every group with models (ch flt/gate/eq/dyn, aux eq/dyn,
    bus/main/mtx eq/dyn, fx/1, fx/9), every model, every parameter, set by slot path through ParamStore -> pipeline ->
    real host (ChannelStrip / MixStrip / FxSlot) -> ModelSlot -> model. Checks binding, readback (model reports the
    pipeline's engineering value at every sweep point), audibility (sweep min..max / every enum value on a stereo
    programme with a pitched voice burst; params dead at defaults retried in enabling contexts: switches flipped,
    one gain/threshold at an extreme, random, selector + amount pairs), repeatability after `reset` (bit-exact) and
    finiteness. `JUSTIFIED` table for the legitimately inaudible. `make check`: binding+readback of all 2876 params
    + sampled audio (ch groups, fx/1 in 2 shards; each test < 6 s dev). `make check-full`: `full_*` (all groups,
    fx sharded; ~70 s release). `WING_AUDIT_REPORT=1` prints the tables.
  - **Bugs found and fixed**:
    1. WARM drive law (above).
    2. `C5-CMB` `slope` (fx): the pipeline sends enum indices as `Float(index)`; the model read Float as the dB
       value (`> 36`), so 48 dB/oct (index 1) was unreachable from WING-Edit (READBACK). Float < 2 = index now.
    3. `CONCERT` (and every ReverbCore reverb with chorus): `reset` did not restart the chorus LFO phase, so output
       after a reset/model swap depended on history (NonDet, -21 dB). Fixed.
    4. `DOUBLE`: `reset` kept the voice shifters and wander RNG (NonDet, -6 dB). Fixed.
    5. `MOOD`: `reset` kept the random-LFO RNG (NonDet). Fixed.
    No UNBOUND (every layout covers its group's slots), no NonFinite.
  - **Audit results (full sweep, final)**: Ok = live at defaults, Ctx = live once an enabling parameter is set
    (e.g. EQ Q needs gain, attack needs `auto` off, DEQ3 band 3 needs its threshold).

    | group | params | Ok | Ctx | Justified | Dead / Readback / Unbound / NonDet |
    |---|---|---|---|---|---|
    | ch/*/flt | 6 | 6 | 0 | 0 | 0 |
    | ch/*/gate | 186 | 144 | 41 | 1 | 0 |
    | ch/*/eq | 93 | 63 | 30 | 0 | 0 |
    | ch/*/dyn | 186 | 144 | 41 | 1 | 0 |
    | aux/*/eq | 85 | 58 | 27 | 0 | 0 |
    | aux/*/dyn | 150 | 135 | 15 | 0 | 0 |
    | bus/*/eq | 103 | 72 | 31 | 0 | 0 |
    | bus/*/dyn | 186 | 144 | 41 | 1 | 0 |
    | main/*/eq | 103 | 72 | 31 | 0 | 0 |
    | main/*/dyn | 186 | 144 | 41 | 1 | 0 |
    | mtx/*/eq | 103 | 72 | 31 | 0 | 0 |
    | mtx/*/dyn | 186 | 144 | 41 | 1 | 0 |
    | fx/1 | 784 | 550 | 225 | 9 | 0 |
    | fx/9 | 515 | 289 | 220 | 6 | 0 |

    Before the fixes: WARM drive weak (not caught as dead, -44 dB at -30 dBFS: the dedicated test
    `warm_drive_is_audible_through_the_pipeline` covers it), C5-CMB slope READBACK, CONCERT 16 params + DOUBLE 2 +
    MOOD 3 NonDet.
  - **Justified** (no DSP effect by design): DEQ2 `$page` (UI page); EXT `ein` (routing, resolved by the input
    router, tests/fx_ext.rs) and `lat` (stored, deliberately not a processing latency); ROTARY `dac`/`hac` (rotor
    acceleration, only during SPEED changes; covered by `rotary_rotors_ramp_with_inertia`); VSS3 `preset`/`load`/
    `view` (UI); MACH4/PULSAR FX `mix` (schema range is the single value 100).
  - Files: crates/wing-blocks/src/models/dyn_special.rs (+ dyn_special/tests.rs), fx_eqdyn/combinator.rs
    (+ fx_eqdyn/tests.rs), fx_reverb/core.rs, fx_pitch.rs, fx_mod.rs; crates/wing-engine/tests/param_audit.rs (new);
    docs/VERIFY-ON-HARDWARE.md.
  - Test: `make check` (pass); `make check-full`; full audit report:
    `WING_AUDIT_REPORT=1 cargo test -p wing-engine --release --test param_audit -- --include-ignored --nocapture`.
  - Proposed follow-ups: VSS3 preset library is not applied on `load` (console behaviour, not modelled); verify the
    WARM drive law / THD curve on hardware (VERIFY-ON-HARDWARE); the audit holds parameters static, so
    transition-only behaviour (rotary acceleration, smoothing) stays with the model unit tests.
