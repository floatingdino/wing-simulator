---
id: T05U
title: FX slot latency follows the loaded model (not max over all models)
stage: S5
status: todo
owner: -
depends: [T05T]
branch: -
---

## Goal
Since T03E, an FX slot's latency is fixed at the max latency of ANY model it can host. With PCORR (360 samples = 7.5 ms) that makes every insert-using strip 7.5 ms late even when the slot holds a zero-latency EQ. Change the policy: a slot's latency = the latency of its currently loaded model; when the model (or insert assignment) changes, the owning strip's alignment pad is recomputed and re-aligned with a short click-free crossfade (model changes already crossfade). Channel/bus alignment targets become dynamic: recompute the global strip alignment when any inserted slot's latency changes (off the RT thread, applied atomically, crossfaded). Same idea may apply to dynamics slots (WARM/76LA/NSTR) — evaluate and apply if cheap. Document resulting typical latency for PUBLIC.snap.

## Acceptance criteria
- [ ] Slot with STD-latency-0 model adds 0 latency; switching to PCORR adds 360 and strips stay aligned (null tests)
- [ ] No clicks on model switch / realignment; zero RT alloc
- [ ] Tests derive expected latency from the engine, nothing hard-coded

## Log
- 2026-10-25 coordinator: created after T05T made every FX slot 360 samples.
