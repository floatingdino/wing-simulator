---
id: T065
title: DAW sync: scene recall via OSC/MIDI from REAPER
stage: S6
status: review
owner: agent-T065
depends: [T014, T017]
branch: task/T065
---

## Goal
Timeline-driven snapshot/scene recall and parameter automation via OSC from REAPER.

## Acceptance criteria
- [x] Demo project (`integrations/reaper/demo/WingSim-Demo.RPP`) + tests (OSC loopback store/recall, pattern/demo/Lua checks)

## Log
- 2026-10-25 coordinator: created.
- 2026-10-25 agent-T065: done, status review.
  - Daemon: OSC now routes `/$ctl/lib/$action` (`,s`/`,i`) to the scene library like the native protocol
    (other `$ctl/lib/$*` sets dropped) and adds simulator extensions `/wingsim/scene/{recall,store,next,prev}`,
    `/wingsim/scenes` (list reply) and `/wingsim/marker`/`/wingsim/region` (REAPER LAST_MARKER/REGION_NAME
    feedback; `WING:<scene|#N>` recalls, repeats ignored). Library action `GO:name` (recall by name/#N).
    `OscConfig.library`, `OscService::with_library`; wing-sim wires the library into OSC.
  - REAPER: `integrations/reaper/` — `wing_osc.lua` (OSC encoder, sends via background `nc -u`, host/port
    from ExtState "WingSim"), `WING - Recall scenes at markers.lua` (deferred follower, crossings + chase on
    play/loop/seek), `WING - Store scene.lua` (+ optional marker), `WING - Recall scene.lua`, Next/Previous,
    `WingSim-Markers.ReaperOSC` / `WingSim-Faders.ReaperOSC` (control-surface patterns; faders map track
    volume dB/mute 1-40 to /ch/N/fdr,/ch/N/mute), demo RPP. Docs: `docs/user/reaper-integration.md`.
  - Files: crates/wing-proto/src/osc/{service,server}.rs, apps/wing-sim/src/{main,library}.rs,
    apps/wing-sim/tests/reaper_integration.rs, integrations/reaper/*, docs/user/reaper-integration.md.
  - Test: `make check` passes. `cargo test -p wing-sim osc_store_and_recall_loopback` (UDP loopback, ephemeral
    ports), `cargo test -p wing-proto osc::service`, `cargo test -p wing-sim --test reaper_integration`.
  - Gotchas: no luac on this machine, so the Lua syntax check in reaper_integration skips (runs `luac -p` when
    present); scripts were only balance-checked by a heuristic. Not tested in a live REAPER; pattern names
    (LAST_MARKER_NAME, LAST_REGION_NAME, TRACK_VOLUME_DB, TRACK_MUTE) are from REAPER's Default.ReaperOSC
    from memory. `#N` is 0-based (existing T017 convention).
  - Proposed follow-ups: verify in a live REAPER (marker feedback timing, `b` mute type, demo RPP load); add
    a `wing-sim` CLI OSC sender for Windows; pan/send mapping with WING units via a dedicated endpoint.
