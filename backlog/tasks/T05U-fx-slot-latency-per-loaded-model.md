---
id: T05U
title: FX slot latency follows the loaded model (not max over all models)
stage: S5
status: review
owner: agent-T05U
depends: [T05T]
branch: task/T05U
---

## Goal
Since T03E, an FX slot's latency is fixed at the max latency of ANY model it can host. With PCORR (360 samples = 7.5 ms) that makes every insert-using strip 7.5 ms late even when the slot holds a zero-latency EQ. Change the policy: a slot's latency = the latency of its currently loaded model; when the model (or insert assignment) changes, the owning strip's alignment pad is recomputed and re-aligned with a short click-free crossfade (model changes already crossfade). Channel/bus alignment targets become dynamic: recompute the global strip alignment when any inserted slot's latency changes (off the RT thread, applied atomically, crossfaded). Same idea may apply to dynamics slots (WARM/76LA/NSTR) — evaluate and apply if cheap. Document resulting typical latency for PUBLIC.snap.

## Acceptance criteria
- [x] Slot with STD-latency-0 model adds 0 latency; switching to PCORR adds 360 and strips stay aligned (null tests)
- [x] No clicks on model switch / realignment; zero RT alloc
- [x] Tests derive expected latency from the engine, nothing hard-coded

## Log
- 2026-10-25 coordinator: created after T05T made every FX slot 360 samples.
- 2026-10-25 agent-T05U: done, status review.
  - FX slots: `ModelSlot::with_dynamic_latency()` (used by `FxSlot`): latency = loaded model's; each model
    gets the live input, dry path at the model latency; a swap between latencies crossfades the dry path
    between the two delays in step with the model fade (also while the slot is off/idle; a swap requested
    during a dry-only fade is deferred). Dynamic slots use a raised-cosine fade (`latency::smooth_fade`): a
    linear fade between time-shifted copies leaves a `|b-a|/len` slope kink at its ends.
  - Alignment board (fx.rs `Shared`): strips publish own slot latency (`InsertHost::set_own_latency`,
    after Commit); on own/slot-latency/owner change the rack recomputes demand = own + owned insert latency
    per strip and publishes the max per family (input ch+aux, bus, main, mtx) in atomics. Runs in the
    parameter phase between cycles (the only place models/claims change; O(76+16), alloc-free) rather
    than a separate control thread — the inputs live on the RT side, and this way the next cycle is
    already consistent (no one-block lag). Strips read `InsertHost::align_target()` at block start;
    `StripAlign::sync(own, inserts, target)`; `FxRack::align_target(strip)` exposes it.
  - `LatencyPad`: TapLine-based, history always written, delay changes crossfade (raised cosine, 5 ms =
    `MODEL_XFADE_S`; retarget mid-fade deferred, reverse supported); `set_delay_now` for construction.
  - Dynamics slots evaluated and NOT made dynamic: only the strip output is padded, so a dynamic dyn/gate
    slot would shift every pre-fader tap after it (ptap T3..T5 = PRE sends / monitor mixes) by the model
    latency vs other channels (confirmed: the PRE-send bus test broke by 57 samples). Saving would be at
    most 114/57 samples. Infrastructure supports it (`with_dynamic_latency`) if per-tap pads are added.
  - PUBLIC.snap (`cargo test -p wing-engine --test fx_latency -- --nocapture`): before (T03E slot max,
    every FX slot 360, unaligned): input strips 114..834 samples, busses 57..777, mains 57..777, mtx
    57..417. After: all aligned per family — inputs 474 (9.88 ms; PCORR on ch 21/22), busses 57 (1.19 ms),
    mains 199 (4.15 ms; *MASTER* 142 on main 4), mtx 57. Documented in crates/wing-engine/src/latency.rs.
  - Files: crates/wing-engine/src/{latency.rs, params/strip.rs, fx.rs, channel.rs, mix/strip.rs,
    mix/nodes.rs}, tests/{fx_latency.rs (new), fx.rs}, wing-blocks fx_pitch.rs/fx_strip.rs (docs).
  - Tests: tests/fx_latency.rs — GEQ adds 0, PCORR adds `slot_latency` (from engine) to every input strip,
    ch1 (insert) vs ch2 (polarity-inverted) bus nulls exactly before/with/after PCORR; switches smooth
    (2nd difference < 2x steady; hard switch > 20x); switching strip below the family max (pad and slot
    fade together) also clean; PUBLIC.snap targets = family max demand. latency.rs pad crossfade unit
    test. Zero RT alloc: existing fx_alloc.rs swaps PCORR/GEQ/... and moves owners under the counting
    allocator (board + pad fades covered). `make check` and `make check-full` pass.
  - Proposed follow-ups: (1) per-tap alignment pads (would allow dynamic dyn/gate slots and align taps after
    FX inserts); (2) a 474-sample input latency whenever any channel inserts PCORR is the price of
    alignment — consider an option to exclude strips from alignment; (3) report engine latency to HAL
    (T03E follow-up 1) now varies with the scene.
