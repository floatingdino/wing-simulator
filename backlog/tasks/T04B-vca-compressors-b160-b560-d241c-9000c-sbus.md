---
id: T04B
title: VCA compressors: B160, B560, D241C, 9000C, SBUS
stage: S4
status: review
owner: agent-T04B
depends: [T045, T034]
branch: task/T04B
---

## Goal
dbx 160/560, Drawmer, SSL channel/bus-style per models.md §3.2.

## Acceptance criteria
- [x] Static curve + time-constant tests vs reference curves (no WING static curves exist for these models; tests check the cited hardware specs and the WING-Edit GR-meter mapping)

## Log
- 2026-10-25 coordinator: created by decomposing epics T040-T044/T051 after T004.
- 2026-10-25 agent-T04B: implemented all five models in `crates/wing-blocks/src/models/comp_vca.rs` (one shared
  feed-forward VCA core `VcaComp`, stereo-linked; slots/enum tables/defaults from `data/schema/wing-tree.json`),
  registered with one line per model in `model.rs::create_model`. Status -> review.
  - Constants and sources:
    - B160/B560 timing: dbx 160A spec sheet (https://c3.zzounds.com/media/160ACutSheet-160ae9a0b6f5c925275ab71466c59d56.pdf):
      attack "15 ms for 10 dB, 5 ms for 20 dB, 3 ms for 30 dB", release "125 dB/sec rate" (8 ms/1 dB, 80 ms/10 dB).
      Modelled physically as the dbx true-RMS log-domain detector = linear mean-square one-pole with
      tau = 10/(ln10*125) = 34.7 ms: releases at exactly 125 dB/s and gives 15.7/6.6/2.8 ms (63 % of the dB
      change) for 10/20/30 dB rises. No further GR smoothing (detector sets all ballistics). dbx 160 = hard knee.
    - B160 threshold: volts RMS (dbx 160 calibration, 10 mV..; WING 0.01..5) -> dBu (0.7746 V) -> dBFS.
    - B560: dbx 160X threshold -40..+20 dBm/dBu (vintagedigital.com.au dbx 160X listing), ratios beyond infinity
      (160X "INFINITY+" negative compression) via slope 1-1/R (R<0 => GR > input excess). OverEasy (dbx 165A /
      160A docs: ratio increases gradually from 1:1 through the threshold region) as log-sum knee
      `(1-1/R)*w*log10(1+10^(d/w))`, w = 6 dB (10-90 % ratio transition ~11.5 dB). **w is an assumption**: dbx
      publishes no width (models.md: 10-15 dB).
    - dBu thresholds (B160, B560, 9000C) use 0 dBu = -18 dBFS (EBU R68). **Assumption**: WING's alignment for these
      models is unpublished; WING max I/O level is +21 dBu (retail spec listings). One constant `DBU_AT_0DBFS`.
    - D241C: Drawmer DL241 docs (drawmer.com DL241 page, manual at barryrudolph.com/recall/manuals/drawmerdl241.pdf,
      retailer copies): attack 0.5-100 ms, release 0.05-5 s, Auto mode "continually optimises" both; soft knee where
      the ratio ramps in over a 10 dB input range above threshold (quadratic knee from thr to thr+10);
      zero-response-time, zero-overshoot peak limiter (instant GR = overshoot, exponential `lrel` release; no
      look-ahead, output never exceeds `lim`). Detector PEAK (assumption, not documented). AUTO attack 10 ms
      (assumption) + two-stage auto release (below).
    - 9000C: attack 30 ms normal / 3 ms fast "at 20 dB gain reduction" (Mix review of SSL E-series dynamics,
      mixonline.com; Waves E-Channel guide gives 1 ms fast: not used), release 0.1-4 s, threshold +10..-20 dBu on
      hardware (WING -30..+18 dBu), PK switch = peak detection (SSL Super 9000 docs: "Fast Attack mode and Peak
      Detect", THAT 2181 VCA). Hard knee. RMS window 10 ms (**assumption**).
    - SBUS: stepped attack 0.1/0.3/1/3/10/30 ms and release 0.1..1.6 s + AUTO (SSL G-Comp 500 / UA SSL 4000 G docs),
      ratios 2/4/10 original (+1.5/3/5 on the 500 module); AUTO = "program-dependent, 2-stage release" (SSL Bus
      Comp marketing text): fast stage 0.1 s, slow stage charge 0.5 s / release 1.6 s, GR = max (constants are
      **assumptions** bounded by the manual steps). Feed-forward (SSL Bus+ describes the classic mode as the
      "conventional feed-forward" topology), full-wave/peak detector, hard knee, threshold dBFS.
    - Time-constant semantics: att/rel knob values are one-pole time constants in the dB domain (63 %).
    - No VCA colouration modelled (none documented for these units); no make-up (WING wrapper `gain`).
  - GR telemetry: `ModelTelemetry.gain_reduction_db` = actual applied GR (comp + D241C limiter). `gr_meter(kind, gr)`
    reproduces WING-Edit `comp_gain_<m>` exactly: B160/SBUS `-0.3*10^(-GR/20)`, B560 GR/20 cap 2, D241C cap 1.5,
    9000C cap 1 (checked against dyn-models.json).
  - Files: `crates/wing-blocks/src/models/{mod.rs,comp_vca.rs}` (new), `src/lib.rs` (+`pub mod models`),
    `src/model.rs` (5 registration lines + import + test list), `tests/comp_vca.rs` (reference GR meter, factory,
    zero-alloc), `benches/comp_vca.rs` + `[[bench]]` in Cargo.toml.
  - Tests: unit tests in comp_vca.rs (static curves for all five vs the laws above, dbx attack 10/20/30 dB and
    125 dB/s release, B560 OverEasy/hard/negative/inf ratios, D241C knee + 10-90 % att/rel + limiter overshoot +
    AUTO two-stage, 9000C normal/fast attack at 20 dB GR + release + RMS-vs-peak, SBUS every attack/release step +
    AUTO, stereo link, sidechain, param clamping/snapping, telemetry = applied gain). `make check` passes.
  - Bench (M1, stereo 64-sample block): B160 424 ns, B560 322 ns, D241C 485 ns, 9000C 425 ns, SBUS 448 ns.
  - Proposed follow-ups: measure WING hardware (or capture `comp_parchg_*` posted constants via the T00A
    follow-up hook) to confirm the dBu alignment, OverEasy width, SSL RMS window and AUTO constants; check
    whether WING's SBUS uses a feedback-style side-chain (the original G comp's side-chain contains a VCA).
