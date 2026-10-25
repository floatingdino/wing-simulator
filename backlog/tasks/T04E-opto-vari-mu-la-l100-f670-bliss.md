---
id: T04E
title: Opto/vari-mu: LA, L100, F670, BLISS
stage: S4
status: done
owner: agent-T04E
depends: [T045, T034]
branch: task/T04E
---

## Goal
LA-2A, LTA100, Fairchild 670, Eternal Bliss per models.md §3.2; program-dependent release.

## Acceptance criteria
- [x] Behaviour tests

## Log
- 2026-10-25 coordinator: created by decomposing epics T040-T044/T051 after T004.
- 2026-10-25 agent-T04E: implemented all four models in `crates/wing-blocks/src/models/comp_opto.rs` (`OptoComp`,
  stereo-linked; slot order/defaults/enum tables from `data/schema/wing-tree.json`), registered one line per model
  in `model.rs::create_model`. Merged main (T04C comp_console, T047 gate_vintage) per the family merge convention.
  Status -> review.
  - Hardware identified: LA = Teletronix LA-2A (`comp_pardef_la2a`), L100 = Summit Audio TLA-100A (`tla100a`),
    F670 = Fairchild 670 (`f670`), BLISS = elysia mpressor (`comp_pardef_mpress`; class-A discrete feed-forward,
    negative ratios as in the Eventide Omnipressor; SOS / Barry Rudolph / bonedo reviews).
  - Constants and sources (all others are assumptions, listed in docs/VERIFY-ON-HARDWARE.md):
    - LA-2A: attack 10 ms average, release ~0.06 s to 50 %, 0.5..5 s complete release (Universal Audio LA-2A
      Classic spec, help.uaudio.com/hc/en-us/articles/206356233; same figures on oxbowaudio.com / zzounds; Wikipedia's
      1..15 s not used). COMP ~3:1 (models.md / UA docs); LIM 20:1 assumed (UA: "higher ratio").
      Model: feedback leveler (detector on the attenuated signal, pre GAIN pot, as on the LA-2A), EL-panel mean
      square 5 ms, 10 dB quadratic knee, two-part T4 cell GR = 0.6 fast (42 ms) + 0.4 slow, slow tau = 0.24..2.4 s
      (= 0.5..5 s / ln 8, "complete" = GR < 5 %) interpolated by a cell memory (3 s charge while lit, 30 s decay).
      Cell attack tau = 10 ms * (1+k) so the closed-loop attack is the published 10 ms in both modes.
    - TLA-100A: attack 180 us..100 ms, release 32 ms..1.5 s (Zen Pro Audio spec listing; Summit manual gives only
      FAST/MED/SLOW) -> ends = FAST/SLOW, MED = geometric middle. "The slower the release, the more the programme
      determines it" and "GAIN RED works like a threshold and increases ratio" (Softube TLA-100A manual) -> same
      leveler engine, slow-part spread x1/x3/x6, thr 6-3.6*gr dBFS, ratio 2+0.4*gr, 12 dB knee.
    - Fairchild 670 time constants (670 manual table as repeated in thomann.de "fairchild_670" listings):
      1: 0.2 ms/0.3 s, 2: 0.2 ms/0.8 s, 3: 0.4 ms/2 s, 4: 0.4 ms/5 s, 5: 0.4 ms/2 s single peaks, 10 s multiple,
      6: 0.2 ms/0.3 s single, 10 s multiple, 25 s consistently high level (Thomann prints 0.2 s for pos 5 singles; the
      manual's 2 s used). Model: feedback, full-wave peak rectifier vs DC threshold, vari-mu law GR = 10 dB *
      knee(|y|/v_thr - 1, 0.5) (GR in dB linear in grid bias of a remote-cutoff tube -> ratio rises with GR: ~2.8:1
      at 6 dB, ~4.5:1 at 20 dB), parallel RC stages (charge 0.1 s / 2 s, assumptions) for pos 5/6; INPUT
      attenuator before the cell + fixed +10 dB amp; BIAS < CAL halves gm slope and unbalances the push-pull stage.
    - mpressor: WING ranges used (attack 0.4..150 ms, release 5..1200 ms). Anti-Log = linear-gain-domain ballistics
      (vs dB one-poles); Auto Fast = attack rate x (1 + overshoot/6 dB); GR limiter = GR clamp at |glim|. Peak
      detector, hard knee, no colouration.
  - Tube colouration (LA, L100, F670; WING's is undocumented -> mild, ~-45 dBc H2/H3 at 0 dBFS): cubic soft clip
    with bias, applied as y = x + HP5Hz(ADAA1(residual)): linear path untouched (no ADAA half-sample droop), residual
    first-order antiderivative anti-aliased, zero latency, DC passes unchanged. Chose ADAA over the wing-dsp
    oversampler to avoid 39 samples of dynamics latency. Measured: 15 kHz at +6 dBFS, folded H3 at 3 kHz -47 dBc
    (naive -33), H2 at 18 kHz -60 (naive -37.5); +16 dBFS clipping: 21 kHz -58 (naive -26).
  - GR telemetry = applied cell GR; `gr_meter` = WING-Edit `comp_gain_*`: GR/20 cap 1 (LA/F670/BLISS), L100
    negative (-GR/20 cap -1); checked against dyn-models.json.
  - Files: `crates/wing-blocks/src/models/comp_opto.rs` (new, incl. unit tests), `src/models/mod.rs`,
    `src/model.rs` (4 registrations, implemented list, stub tests now use 3CMP), `tests/comp_opto.rs` (reference
    GR meter, factory, zero alloc), `benches/comp_opto.rs` + `[[bench]]`, `docs/VERIFY-ON-HARDWARE.md`.
  - Tests (`cargo test -p wing-blocks comp_opto`): LA static COMP/LIM/knee/knob maps vs solved feedback law, attack
    10 ms both modes, 50 % release 40-80 ms and complete release 0.4-1.0 s (0.1 s burst) vs 3.5-5.5 s (12 s
    programme); L100 static/knob, attack ordering, per-switch release + programme dependence; F670 static vs solved
    vari-mu law, ratio rising with GR, INPUT/BIAS/THR; pos 1-4 release within 3 % of published, pos 6 single 0.3 s
    vs multiple vs sustained (>10 s), pos 5 2 s; BLISS static incl. negative ratios + GR limit, log/anti-log
    release shapes, auto-fast; tube DC transparency/harmonic level/aliasing; stereo link, sidechain, params.
    `make check` passes (after merging main).
  - Bench (M1, stereo 64-sample block): LA 1.39 us, L100 1.38 us, F670 2.37 us (pos 6: 2.63 us), BLISS 0.64 us.
  - Proposed follow-ups: measure a WING (or capture `comp_parchg_la2a/f670/tla100a/mpress` constants via the T00A
    hook) for knob laws, ratios and whether colouration exists; optional 2x-oversampled tube path if hardware shows
    strong HF saturation; L100 FAST attack with instantaneous-power detection ripples on LF content (by design,
    verify against hardware).
- 2026-10-25 coordinator: follow-up — aliasing at +6 dBFS/15 kHz is -47 dBc (target -60); consider 2x oversampling around the tube stage in T070 perf/quality pass.
