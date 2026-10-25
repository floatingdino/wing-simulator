---
id: T03E
title: Latency compensation (model-slot dry path + strip/bus alignment)
stage: S4
status: review
owner: agent-T03E
depends: [T04D, T037, T038]
branch: task/T03E
---

## Goal
Oversampled models report latency (76LA/NSTR: 57 samples; others 0). The engine's ModelSlot doesn't delay its dry path, so `mix` < 100 % comb-filters (PUBLIC.snap uses NSTR at mix 80), and channels with different model latencies arrive misaligned at busses/mains. Implement: (1) dry-path delay in ModelSlot matching the active model's latency (incl. during crossfades between models of different latency); (2) decide WING's behaviour — WING's DSP runs fixed-latency blocks, so emulate constant per-strip latency: pad every strip to the max possible model latency of its slots so summing stays phase-coherent regardless of model choice; document the total added latency; (3) report total I/O latency to the driver/REAPER where appropriate.

## Acceptance criteria
- [x] mix < 100 % on NSTR shows no comb filtering (null test vs ideal parallel)
- [x] Two channels with different dyn models summed to a bus stay sample-aligned (null test)
- [x] Added latency documented; zero RT alloc

## Log
- 2026-10-25 coordinator: created from T04D follow-up.
- 2026-10-25 agent-T03E: done, status review.
  - Decision: WING-style constant latency. Each `ModelSlot` runs at the max latency of its hostable models
    (gate/dyn slots: 57 = 76LA/NSTR; EQ/filter tool: 0). One input `TapLine` per side feeds the dry path at
    the slot latency and each model (and its side-chain key) at `slot - model latency`, so mix blends,
    crossfades and on/off are aligned and the incoming model of a swap is primed from the input history
    (no 57-sample hole in the crossfade). Zero-latency slots are bit-exact/zero-cost as before.
  - Strips: `StripAlign` pad at the strip output; channel and aux share the input-strip latency (aux pads the
    gate + tool slots it does not run), bus/main/mtx their own. API for FX (T050): `latency()`,
    `set_insert_latency(n)` (pad absorbs insert latency), `set_align_latency(Some(n))` (engine target), on
    `ChannelStrip` and `MixStrip`; reusable `latency::{LatencyPad, TapLine, StripAlign}`.
  - Added latency @48k: channel/aux 114 samples (2.375 ms), bus/main/mtx 57 (1.19 ms) before the output delay;
    ch -> bus -> main = 228, -> mtx 285 (5.94 ms). Documented in crates/wing-engine/src/latency.rs.
  - Files: crates/wing-engine/src/{latency.rs (new), lib.rs, params/strip.rs, channel.rs, mix/strip.rs},
    tests/{latency.rs (new), channel_strip.rs, mix.rs, mix_strip.rs, monitor.rs, channel_no_alloc.rs},
    benches/strip.rs (`slot_nstr_mix80_64`: 18.7 us/64 samples, almost all NSTR itself).
  - Tests: `cargo test -p wing-engine --test latency` (NSTR mix 80 null vs ideal parallel -144 dB; uncompensated
    -5.5 dB; NSTR vs COMP vs aux strips exact dry null + xcorr lag 0; busses fed by NSTR/COMP channels aligned;
    COMP->NSTR swap click-free). Existing tests shifted by the strip latency where they compare across a strip;
    mix-law tests (mix.rs, monitor.rs) use `no_processing()` hooks. channel_no_alloc covers align calls.
    `make check` passes.
  - Proposed follow-ups: (1) report engine latency to the HAL/REAPER: needs a wing_shm header field
    (layout version bump) + driver `kAudioStreamPropertyLatency` for input = window + DSP latency (not done:
    C++/layout change); (2) T050 FX rack: call `set_insert_latency` on insert engage; (3) PRE/IN taps and
    sends are not compensated (tap sees latency accumulated so far) — verify against WING;
    (4) channel->main vs channel->bus->main paths differ by one bus latency (57), as on hardware w/o PDC.
