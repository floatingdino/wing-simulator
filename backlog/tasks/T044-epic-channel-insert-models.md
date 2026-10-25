---
id: T044
title: Insert points hosting FX slots + automix X/Y
stage: S4
status: review
owner: claude-T050-T044
depends: [T050, T035]
branch: task/T050-T044
---

## Goal
Coordinator decomposes into one task per model.

## Acceptance criteria
- [x] Insert points Ins1/Ins2 host FX slots on ch (pre+post), aux (pre), bus/main/mtx (pre+post); no child tasks needed (inserts have no models)
- [x] Channel post-insert automix AUTO_X/AUTO_Y with weight `w` and `cfg/amix/x|y` (in the schema, implemented)

## Log
- 2026-10-25 coordinator: created.
- 2026-10-25 coordinator: inserts have no models of their own (host an FX slot); retitled.
- 2026-10-25 claude-T050-T044: done with T050 on branch task/T050-T044 (see T050 log for files/tests).
  - Every strip's insert points call the FX rack through a per-strip `InsertHost` handle
    (`FxRack::host(strip)`); strip nodes forward claim changes on `Commit` (`channel::sync_inserts`). A slot
    serves one insert; others pass through with `$stat` N/A (`FxRack::insert_state`). Latency of engaged,
    owned inserts is included in `ChannelStrip::latency_with`, `MixStrip::latency`,
    `InputStripNode::latency`, `MixStripNode::latency`.
  - Automix (channel `postins.mode` AUTO_X/AUTO_Y, `w`, `cfg/amix`): Dugan gain sharing
    `g_i = sqrt(Pw_i / sum Pw_j)` on smoothed power (5 ms attack / 100 ms release), others' levels from the
    previous block (keeps channels parallel, deterministic); disabled group = pass-through. Test:
    `automix_shares_gain` (2 equal mics -3 dB each, +6 dB weight split, groups X/Y, disable).
  - Test: `make check` passes.
