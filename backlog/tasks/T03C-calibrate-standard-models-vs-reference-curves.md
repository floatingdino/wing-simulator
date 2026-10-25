---
id: T03C
title: Calibrate standard models vs WING-Edit reference curves + dynxo/key listen
stage: S3
status: done
owner: claude-T03C
depends: [T00A, T031, T032, T033, T034]
branch: task/T03C
---

## Goal
Validate and correct the standard models (filter, GATE/DUCK, STD EQ, COMP/EXP) against data/reference-curves (T00A). T034 guessed: knee width (2 dB/step), RMS window, LIN release rate, AUTO release constants, 90 dB expander floor, make-up applied to wet path only — confirm or fix each, citing evidence. Also implement the dynamics crossover/sidechain EQ `dynxo` and key listen (solo of the sidechain signal) per models.md.

## Acceptance criteria
- [x] Every guessed constant confirmed or corrected with evidence (4 remain unresolvable from WING-Edit, see table)
- [x] Fidelity tests pass against reference curves within tolerance
- [x] dynxo + key listen implemented and tested

## Log
- 2026-10-25 coordinator: created from T034 follow-ups.
- 2026-10-25 coordinator: also from T031/T032 — gate ACCENT (guessed 0–10 dB hysteresis), Butterworth cut shape, input balance law and gate detector timing are unverified assumptions.
- 2026-10-25 coordinator: T033 guessed conventions to verify: Q and shelf-Q definitions, CUT behaviour, Bessel normalisation, tilt pivot, mix law.
- 2026-10-25 claude-T03C: done, status review. Method: disassembled the coefficient/law code of WING-Edit 3.3.4
  (`Cbiquad::hqpeq/hqbp/ls12/hs12/hqls6/hqhs6/hqls12/hqhs12/hqlp12/hqhp12`, `Cxover48::Cxover48`,
  `eq_parchg_xseries[_bus]`, `Cequalizer::update`, `Cchfilter::update`, `Cdynsidechain::update`,
  `Ccompxover::update`, `gate|comp_parchg/respf_xseries_*`, `Cdynamics::update`) with `objdump` and ported it line
  by line in f32 (fused ops, double exp/tan, original constants, 48k/44.1k tables read from `__TEXT,__const`) into
  `crates/wing-blocks/src/wingcoef/` (+ generated `tables.rs`); the blocks now design with it.
  Results vs `data/reference-curves` (`tests/t03c_fidelity.rs`, full grids, 20 Hz..20 kHz; coefficients rel. 2e-5,
  magnitude 0.01 dB): filter LC/HC 0.0005 dB (coef 1.2e-7); STD EQ ch/aux/bus 0.0065 dB (coef 1.5e-6);
  dynsc + dynxo 0.0006 dB (coef 3.8e-7); GATE/DUCK static 0.000 dB; COMP 0.0005 dB; EXP 0.0006 dB.
  Gotchas: exported params are printed read-backs ("6.0" dB was the Cpar float 6.0000005: 1 ulp = 0.015 dB at 20 Hz
  for a 50 Hz shelf), so the EQ magnitude test also tries +-1 ulp gains; below 20 Hz 1-ulp noise of near-identity
  sections is worth ~0.02 dB (rounding, not law). The dynxo export leaves `depth` at -30 dB (not in params).

  | constant / convention | guess (T031-T034) | verified value | evidence |
  |---|---|---|---|
  | COMP/EXP knee width | 2 dB/step | 1.2 bits = 7.2247 dB/step, quadratic, centred on thr | `comp_respf_xseries_comp` knee terms -0.3K, 0.6K, -1/(1.2K) |
  | COMP/EXP threshold | thr | DSP gets trunc(thr); law `x = thr/6 - in/6.0206` (eff. thr x1.00343) | respf; parchg `trunc`, `*(-1/6)` |
  | RMS window | 10 ms | COMP 20 ms, EXP 2 ms (one-pole 0.99895897 / 0.98955512 @48k) | comp/exp parchg (44.1k pair consistent) |
  | PEAK detector | instant abs | one-pole on abs, tau 4.55 ms (0.995427) | parchg |
  | LIN release rate | 10 dB per rel | 16.64 bits = 100.18 dB per att/rel time, attack too | parchg `B*16.64/att`, `B*16.64/rel` |
  | LOG attack/release | one-pole tau | confirmed (tau = att/rel), att >= 0.5 ms | parchg `B*1000/t`, `max(att,0.5)` |
  | AUTO constants | 250 ms charge, rel x4 | RMS: att instant, rel 1/21.24 s, stages 22.5/60.3 ms; PEAK: 1/1.197 s, 5.4 ms, 6.05 s | parchg; DSP topology not in WING-Edit, mapped onto fast/slow dual release [I] |
  | EXP floor | 90 dB | none in WING; numeric floor 144 dB (WING silence sentinel) | no range term in exp parchg/respf |
  | make-up on wet only | assumed | confirmed: dry = 1-mix, wet = mix*10^(gain/20) | `Cdynamics::update` |
  | COMP hold | hld | posted B*500/hld (gate 1000): maybe 2*hld; kept hld | unresolved |
  | GATE ACCENT | 0-10 dB hysteresis | WING posts acc/50 (0..2) only; meaning not recoverable; guess kept | `gate_parchg` out[18] |
  | GATE detector | 10 ms release | 40 Hz bilinear one-pole, tau 3.98 ms (used as peak-follower release [I]) | gate parchg out[17] |
  | GATE/DUCK timing | linear-dB slews over the range | one-pole in linear gain, 10-90 % = att/rel; GATE-ratio release = linear ramp of linear gain 1->0 in rel; DUCK falls with att, recovers with rel | gate/ducker parchg |
  | GATE range | range dB, 60 = oo | floor 10^(-range/20), 0 at last step; respf uses trunc(range) | parchg/respf |
  | Ch LC/HC shape | bilinear Butterworth | LC bilinear Butterworth (6 hqhp6, 12 hqhp12, 18 1st+Q1, 24 Q .541/1.307); HC Massberg analog-matched 6/12 | `Cchfilter::update` |
  | PEQ Q convention | RBJ Q | Orfanidis prescribed-Nyquist PEQ; BW of Q/sqrt(G) (log10/0.301), edge gain = dB midpoint, Q x1.4^(w0-1.822) above 13.9 kHz | `hqpeq` |
  | Shelf Q | RBJ shelf Q | ignored: low = ls12(min(trunc f, 8000), g, .707); high = hqhs12(max(trunc f, 50), g), table-corrected > 2.5 kHz; f = unity-side corner | `eq_parchg_xseries` |
  | CUT | resonant 12 dB with Q | Q ignored: aux low bilinear BW12; bus low 2x hqhp12 (= LR24); high hqlp12(.707) | `eq_parchg_xseries[_bus]` |
  | Bessel normalisation | -3 dB | none: BS12 = Q .577 at f; BS24 = f*1.0588 (Q .806) + f*0.9444 (Q .522) | `Cxover48` |
  | Tilt pivot / sign | 1 kHz, +tilt brighter | hs12(pivot, -2t, Q 0.2)*10^(t/20): +tilt = bass up; pivot 650/780/925 Hz at 0/3/6 dB | `eq_parchg_xseries_bus` (agrees with T046 note) |
  | EQ mix law | dry/wet blend | band gains (dB) x mix/100 (PEQ/SHV only; not tilt/cuts/PRE EQ) | `Cequalizer::update` passes mix/100 to parchg |
  | Bus section budget | - | 10 biquads: a low cut > 2 sections takes bands 1..; a high cut > 1 section takes bands 6, 5, 4 | `eq_parchg_xseries_bus` copy loops |
  | Input balance law (T031) | opposite side down | unresolved: DSP-side, not in WING-Edit/T00A data | - |

  dynxo (new `sidechain.rs`): `Ccompxover` biquad H (LO = hqlp12, HI = hqhp12, S6/S12 = high shelf of -depth or, for
  depth < 0, low shelf of depth, PEAK = hqpeq(f, -depth, q) x10^(depth/20) for depth < 0), applied as
  `x + H*(model(x) - x)` [I] (WING-Edit only posts H). Key filter (dynsc/gatesc): LP12 hqlp12(.707), HP12 hqhp12,
  BP hqbp(q) unity peak, NOTCH hqpeq(-30 dB). Key listen: new `Dynamics::process_listen`; `Gate::key_signal` and
  engine `ModelSlot::key_signal` now carry the exact-filtered key.
  Coordinator note (T046): positive TILT = bass boost holds for the bus EQ tilt too; STD EQ has no allpass, so
  the AP2 alpha convention does not apply.
  Files: wing-blocks `wingcoef/{mod,tables}.rs`, `sidechain.rs` (new), `eq.rs`, `filter.rs`, `gate.rs`,
  `dynamics.rs`, `model.rs` (+ default `Model::set_gain_scale`), `std_models.rs`, `lib.rs`, `Cargo.toml`
  (serde_json dev-dep), `tests/t03c_fidelity.rs` (new), `tests/fidelity_reference.rs` (tightened); wing-engine
  `params/strip.rs` (exact key filter, `set_xover`, `set_mix_in_model`), `params/std_models.rs`, `channel.rs`
  (EQ mix law, `ch/*/dynxo` binding). Additive APIs only: `StdEq::wing_sections`, `FilterBlock::wing_sections`,
  `GateParams::{static_gain_db, floor_lin}`, `Gate::gain_lin`, `ScFilter::kind`, `DynParams::xover`,
  `eq::tilt_pivot_hz`. `BandType::uses_q` now true only for PEQ (WING ignores shelf/cut Q).
  Test: `make check` green; `cargo test -p wing-blocks --test t03c_fidelity -- --nocapture` prints worst cases.
  Proposed follow-ups: (1) recover DSP-side AUTO topology, COMP hold factor, ACCENT and balance law (console
  measurement / firmware DSP code); (2) 44.1 kHz reference sweep to verify the 44.1k tables; (3) map fw 1.13 dynxo
  tokens (HI6/LO6/LO12 + depth) to 3.x; (4) aux/bus/main dynxo binding (schema has only `ch/*/dynxo`); (5) f32
  one-pole stall: the gate now keeps f64 gain state, other smoothers may need the same.
