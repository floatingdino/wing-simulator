---
id: T03G
title: BUG: channel solo from WING-Edit has no effect on monitor output
stage: S3
status: done
owner: agent-T03G
depends: [T038, T03F]
branch: task/T03G
---

## Goal
User (live, v0.4 build): monitor A on the Scarlett works, DCA mute/solo works, but channel solo does nothing. Evidence (scratch/wing-sim-v0.4.log): WING-Edit sends `ch/1/$solo = 1/0`, `bus/N/$solo = 1/0`, `$stat/solo = 0/1`, and `cfg/solo/mon = PH`. Find why channel `$solo` doesn't reach the solo bus/monitor (binding of `ch/N/$solo` and `aux/N/$solo`; solo-to-monitor routing when cfg/solo/mon = PH vs SPK and which of mon/1 / mon/2 is PH (phones) vs SPK (speakers) — the host bridge plays mon/1); fix so soloing a channel in WING-Edit switches the monitor that the host bridge plays (if solo targets the phones monitor and the bridge plays the other, make the default host mapping follow `cfg/solo/mon`, or document clearly and pick the WING-faithful mapping). Regression test reproducing the exact set sequence from the log first.

## Acceptance criteria
- [x] Test fails before / passes after
- [x] Channel, aux, bus, main, mtx and DCA solo all affect the monitor output per cfg/solo
- [x] PH/SPK ↔ mon/1/mon/2 mapping documented with evidence

## Log
- 2026-10-25 coordinator: created from user report; DCA solo/mute confirmed working, monitors confirmed working.
- 2026-10-25 agent-T03G: root cause found and fixed; status review.
  - **Root cause (two parts):** (1) `MixLogic` routed a solo to monitor m only if the strip's `mon` (MON BUS
    A/B/A+B) AND `cfg/solo/mon` both included m (a T038 [I] inference). Factory strips (and every strip/DCA in the
    user's scene) are `A`, so with `cfg/solo/mon` = SPK every solo went *nowhere* (flags had AFL but neither
    SOLO_MON bit; reproduced: ch/1/$solo=1 left both monitors at exact silence). (2) Even routed correctly, SPK =
    monitor 2 and the host bridge plays `mon/1`, so the solo would still be inaudible on the Scarlett.
  - **Evidence:** `scratch/live-mix-v0.4.snap` has `cfg/solo/mon: "SPK"` (all strips `mon: A`); WING-Edit's push at
    12.5 s did not change it to PH (the 94.4 s `cfg/solo/mon = PH` set is logged as "1 changed"). Both channel solos
    (49.2 s, 267.4 s) happened under SPK (SPK again from 233.9 s to 320.1 s); the DCA solos that "worked"
    (145.9-162.6 s) happened under PH. The `ch/N/$solo` binding, exclusive/clear logic and monitor DSP were fine.
  - **PH/SPK mapping (decided, documented in signal-flow.md §10 and logic.rs docs):** PH = `cfg/mon/1` (phones,
    monitor A), SPK = `cfg/mon/2` (speakers, B), PH+SPK = both. Evidence: schema long names `cfg/mon/1/lvl` "PHONES",
    `cfg/mon/2/lvl` "SPEAKERS"; enum order PH/SPK/PH+SPK = A/B/A+B; direct-input options MON.PH/MON.SPK.
  - **Fix:** solo destination = `cfg/solo/mon` only (strip `mon` no longer gates it; its function stays unknown
    [I]). Plus a simulator-only "listened monitors" mask (`MixLogic::set_listen`, set by `Console::build` from the
    host-bridge taps via `console::listen_mask`): a solo whose WING destination misses every monitor the host plays
    is also sent to the played one(s). PH / PH+SPK with the default `mon/1` route are unchanged (WING-faithful);
    SPK now also reaches mon/1. Engine default (no mask) is plain WING routing. Click-free (existing monitor ramps),
    no RT changes (control-side flags only).
  - Files: crates/wing-engine/src/mix/logic.rs (solo_dest, set_listen/listen, strip mon dropped from solo routing,
    docs, unit test), apps/wing-sim/src/audio/console.rs (listen_mask, set in Console::build),
    crates/wing-hostio/src/config.rs (doc), docs/research/signal-flow.md §10, tests below.
  - Tests: apps/wing-sim/tests/audio.rs `t03g_wing_edit_solo_sequence_reaches_host_monitor` replays the log's set
    sequence (client origin) against the scene's solo config with a `mon/1` bridge: fails before (49 s ch solo:
    -277 dB on both monitors), passes after. crates/wing-engine/tests/monitor.rs
    `every_solo_kind_reaches_the_cfg_solo_monitor_and_the_listened_one`: ch/aux/bus/main/mtx/DCA solo x
    (PH, SPK, PH+SPK) x listen masks, all strips on MON BUS B; `srcmix_mixes_source_under_solo` updated.
    `make check` passes.
  - Proposed follow-ups: verify on a real WING what strip `mon` (MON BUS A/B) does and whether SPK solos are heard
    on phones; optional daemon/menu option to turn solo-follows-host off.
