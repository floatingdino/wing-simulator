---
id: T090
title: Calibrate WARM (Soul Warmth Pre) against the user's real WING
stage: S4
status: blocked
owner: claude-T090
depends: [T088, T072]
branch: task/T090
---

## Goal
User: WARM is now audible but "definitely a lot lower than the model on the real console". Add a WARM calibration set to tools/verify-hardware (wing-verify gen/compare): scenes with WARM in a channel's gate slot at mix 100 %, drive sweep (e.g. 10/33/66/100/125 %), harmonics/colour at default and extremes, trim 0; test signals: 1 kHz sines at -36/-24/-18/-12/-6 dBFS, a 100 Hz sine, two-tone IMD, and a short vocal-like excerpt; a quick tier under 5 minutes of capture. Add `wing-verify fit-warm <recordings>` that measures THD/harmonic profile, gain and compression vs level/drive from the recordings and fits WARM's law parameters (drive gain curve, bias/harmonic balance, make-up) to them, writing the fitted constants to a data file the model loads (data/calibration/warm.json) with a test that the fitted model reproduces the recordings' THD within ±2 dB per point. Until recordings exist, keep current constants. Document the procedure in tools/verify-hardware/README.md (WARM section, ~5 min).

## Acceptance criteria
- [x] Calibration scenes + signals generated; README steps
- [x] fit-warm works on synthetic recordings (test) and updates data/calibration/warm.json
- [ ] After the user's capture: fitted model within ±2 dB THD per point

## Log
- 2026-10-26 coordinator: created from user feedback.
- 2026-10-26 claude-T090: done, status review (the third criterion waits for the user's capture).
  - **Law made calibratable** (crates/wing-blocks/src/models/dyn_special.rs): `WarmLaw` = drive points
    `(drv %, shaper gain)` interpolated log-linearly (extrapolated along the end segments), `t_max` (bias at
    hrm -100) and `makeup_ref_dbfs_rms`. Fixed-size and `Copy`. `WarmthPre::new()` uses `WarmLaw::builtin()`
    (parsed once from `data/calibration/warm.json`, embedded with `include_str!`), `WarmthPre::with_law()` for
    fitting. The shipped file is the T088 law exactly (3 points 10/33/125 %, t_max 0.46, ref -18.0103 dBFS RMS
    = WARM_REF_PEAK), so behaviour is unchanged until recordings exist. wing-blocks now depends on serde_json
    (was a dev-dependency).
  - **Kit** (apps/wing-render/src/verify/warm.rs): `wing-verify warm-gen --out warm-kit [--tier all]` /
    `verify.sh warm-gen warm-kit`. suite.wav 15 s: pilot (-33 dBFS so WARM barely changes it), 1 kHz at
    -36/-24/-18/-12/-6 dBFS RMS, 100 Hz -18, SMPTE IMD 60 Hz + 7 kHz 4:1 at -18, vocal-like 3 s at -24, end
    pilot. Quick tier 8 scenes: `warm-thru` (WARM off, level reference), drive 10/33/66/100/125 at hrm 0,
    125 % at hrm -100 / +100; full adds hrm +-100 at 66 % and colour +-1 at 100 %. Same base routing as T072
    (USB 1 -> ch 1 -> MAIN -> USB out 1/2), WARM in ch 1's gate slot, `gate/mix` and `wmix` 100, trim 0.
    Capture ~20 s per scene (manifest `capture_pad_seconds` 4; capture.sh reads it, default 14 for the T072
    kit): ~4 min with recalls.
  - **fit-warm** (`wing-verify fit-warm <recordings> [--out warm-report] [--write data/calibration/warm.json]`,
    `verify.sh fit-warm warm-rec` writes the repo file). It needs only the recordings: the suite and scenes are
    rebuilt deterministically. Measures each recording after pilot alignment (THD H2..H10, H2, H3, level per 1 kHz
    point, 100 Hz THD, IMD, vocal residual via `wing_engine::probe::analyse`). Fit on the 1 kHz points with a
    static one-period shaper evaluation: for each t_max (grid + golden), the best gain per drive setting
    (shape error THD + 0.25 (H2, H3), floor -80 dB), then the make-up reference from the level change vs
    `warm-thru`. Drives below the THD floor are dropped from the curve. Checks the fitted law with the real
    `WarmthPre` over the suite: PASS = every 1 kHz / 100 Hz THD within +-2 dB. Writes `warm-fit.md` (recording
    / fitted / previous law per point, levels) and the JSON (law + per-point fit table).
  - Tests: wing-blocks `builtin_calibration_parses`, `uncalibrated_law_is_the_t088_law`,
    `warm_law_interpolates_and_validates`, `warm_with_law_uses_the_law`; apps/wing-render/tests/verify_warm.rs:
    `warm_kit_generates` (scene contents, < 5 min capture), `fit_warm_recovers_a_synthetic_console_sampled`
    (3 scenes, ~6 s dev) and, ignored as slow, the all-scenes variant + `kit_scene_renders_as_the_model`
    (a kit scene through the whole console = WARM alone, THD within 0.5 dB, level within 0.3 dB). Synthetic
    console (law 0.35/0.9/2.6/9/28, t_max 0.32, ref -22, -3 dB chain, noise, delays) fitted back: worst THD
    delta 0.06 dB, t_max 0.318, ref -21.8, gains within 0.5 %. The previous law misses the same points by > 2 dB.
  - Files: crates/wing-blocks/{Cargo.toml, src/models/dyn_special.rs, src/models/dyn_special/tests.rs};
    data/calibration/warm.json (new); apps/wing-render/src/verify/{warm.rs (new), mod.rs},
    apps/wing-render/src/bin/wing-verify.rs, apps/wing-render/tests/verify_warm.rs (new);
    tools/verify-hardware/{README.md (WARM section), verify.sh, capture.sh}; docs/VERIFY-ON-HARDWARE.md; .gitignore.
  - Test: `make check` pass, `make check-full` pass. No daemon was started.
  - Gotchas: after a fit the T088 target tests (`warm_drive_targets_at_nominal_level`, `tests/warm_live.rs`)
    run on the calibrated law and may need their targets updated to the hardware's. The fitted model is
    only as flexible as the tanh law: if the console's curve has another shape, the report shows which points
    stay off.
  - Proposed follow-ups: fit COLOR's tilt from the cm1/cp1 scenes; a `--law <json>` override for
    wing-render/wing-sim so a fit can be auditioned without a rebuild.
- 2026-10-26 coordinator: user has no console access now; kit + fit-warm merged; blocked on capture (HUMAN-ACTIONS #7).
