---
id: T083
title: BUG: mute toggles don't visually update in WING-Edit; solos exclusive instead of additive
stage: S3
status: review
owner: agent-T083
depends: []
branch: task/T083
---

## Goal
User: (1) mute toggles on channels/buses don't visually update in WING-Edit (audio mutes, UI button doesn't toggle) — likely the server doesn't echo/confirm the new value to the originating client (we fan out only to other clients), or WING-Edit sends a toggle value / expects an echo for some node types. Determine from protocol docs/captures what a real console replies (fixtures/protocol, docs/research/wing-edit-session.md, libwing), and implement (echo to sender for all sets, or for those node types). Check the same for other buttons (solo, on/off switches, enums). (2) Solos behave exclusive; on a WING they're additive by default — check cfg/solo exclusive/"excl" key semantics and our default; make additive the behaviour unless the scene's exclusive setting is on. Regression tests with the native protocol client.

## Acceptance criteria
- [x] WING-Edit-style client sees its own mute/solo toggles confirmed (test)
- [x] Solo additive by default, exclusive when the scene says so (test; see the Log for what "the scene says so" means)

## Log
- 2026-10-26 coordinator: created from user report.
- 2026-10-26 agent-T083: diagnosis and fix (branch task/T083).
  Evidence:
  - No capture of a real console exists (fixtures/protocol are WING-Edit pushes against fakes; libwing is a client
    and says nothing about echo). wing-edit-session.md: plain value sets need no echo (push, DCA buttons).
  - scratch/wing-sim-v0.4.log: while the user clicked channel mutes (audio muted), **no `ch/N/mute` set is ever
    logged**; `bus/1|2/mute = 1` appear once each and never `= 0`; DCA mutes (`dca/5|8/mute`) alternate 1/0 and work.
    The log's "sets" count only value tokens, not `D8`/`D9` actions → WING-Edit toggles strip mutes with `D8`
    (click) and needs the console to tell it the result. The server applied the click but fan-out skips the setter,
    so WING-Edit never got the new value.
  - Schema: ch/aux/bus/main/mtx have `$mute` (int 0..2, like `$sololed`), DCAs have none (and DCA buttons work);
    protocol.md §3.2 [DOC] shows the console emitting `mute` + `$mute` together. The sim never wrote `$mute`.
  - Solo: `$ctl/cfg/soloexcl` (EXCL SOLO) is `true` in the factory scene, which is the user's scene
    (scratch/live-mix-v0.4.snap = PUBLICWING factory snapshot) and in PUBLIC.snap; it is a `ce_data` control-surface
    pref (signal-flow.md §10). The user's real WING solos additively from WING-Edit with it on, so it cannot apply to
    remote writes of `$solo`. The sim applied it to every origin → exclusive from WING-Edit.
  Changes:
  - `crates/wing-proto/src/server/session.rs`: echo to the setter (`D7 h <value>`, no `DE`) after every `D8`/`D9`
    and after value sets the console adjusted (number off by > 1 % / 0.01, string differs, form change such as enum
    index → name, every `D6`). Sets stored as sent are not echoed (A1 push: 0 echoes, asserted). New stat `echoes`.
  - `crates/wing-engine/src/mix/logic.rs`: `*/$mute` indicator written by `MixLogic::refresh` (1 own mute,
    2 only via mute group / DCA mute [I, by analogy with `$sololed`], 0); strip `mute` is now watched. Exclusive
    solo only for solos switched on with `ORIGIN_LOCAL` (console itself) when EXCL SOLO is on; native/OSC client solos
    are always additive. `$stat/solo = 0` clear-solo unchanged.
  - Tests: `crates/wing-proto/tests/server_replay.rs::tcp_click_and_adjusted_sets_are_confirmed_to_the_setter`
    (native TCP client, ephemeral port: D8 on ch/bus mute and `$solo` confirmed to setter + observer, D9 step, no echo
    for a plain set, clamp and enum-index echoes) + `echoes == 0` on the A1 push replay;
    `crates/wing-engine/tests/monitor.rs::{client_solos_are_additive_with_excl_solo_on,
    mute_indicator_follows_own_and_group_mutes}`; new `apps/wing-sim/tests/native_buttons.rs` (native server +
    `audio::prepare` control logic with a refresh thread, ephemeral port: mute clicks confirmed with `mute` and
    `$mute`, WING-Edit solo sequence from the log stays additive with factory EXCL SOLO). Both native_buttons tests
    fail with the fixes disabled.
  - docs/research/protocol.md: §3.2 table + §9 Q10 updated.
  Test: `make check` passes. Not verified against the live WING-Edit (user's running wing-sim untouched).
  Note: `$mute` (like `$sololed`) is written by the audio control thread's refresh, so it only updates when the
  daemon runs with audio (`--audio`, the user's setup); the `mute` echo itself works without audio.
  Proposed follow-ups:
  - Verify with WING-Edit live that ch/bus mute buttons toggle; if not, capture WING-Edit's click bytes (D8 vs set)
    and whether it draws `mute` or `$mute`.
  - `$fdr` mirror (emitted with `fdr` by the console) is still not modelled.
  - Capture a real console's reply to `D8`/sets to settle protocol.md §9 Q10.
