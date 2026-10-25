---
id: T011
title: Snapshot (.snap) load/save round-trip
stage: S1
status: review
owner: eng-agent
depends: [T010]
branch: task/T011-T019
---

## Goal
Load fixtures/PUBLIC.snap into the store and save it back. Semantic round-trip equality; preserve unknown keys verbatim; write format accepted by a real WING (same type/version fields).

## Acceptance criteria
- [x] Round-trip test on PUBLIC.snap: JSON semantically equal
- [x] Unknown keys preserved
- [x] Loader reports unmapped keys

## Log
- 2026-10-25 coordinator: created.
- 2026-10-25 eng-agent: done (with T019, branch task/T011-T019). `Snapshot::load(&store, text)` resets the
  store to the factory scene, applies model selectors first (repeat until stable), then all keys; returns a
  `LoadReport` (applied/migrated/preserved/unmapped/rejected/adjusted). `Snapshot::save` writes the 3.x layout:
  the 25,078 params the fw 3.0.6 factory scene persists (no flag predicts this, so the factory scene is
  embedded and used as the mask) + active model slots + any extra tree params the file had; schema order;
  compact JSON; header verbatim for 3.x files. Unknown keys (and empty objects like `ce_data/lib`) are kept
  as raw JSON in a side table and re-inserted at their path. Own tiny order-preserving JSON codec (no new
  deps; numbers keep source text for verbatim preservation).
  Results: factory scene -> store -> JSON equal key-for-key, every number the same f32 (values on the step
  grid are stored bit-exact; off-grid ones are quantised and reported as `adjusted`), 0 unmapped/rejected,
  3 preserved GUI prefs (`$ctl/$globals/usewheel|timefmt|datefmt`, not in the WING-Edit tree). Edited 3.x
  scene with an unknown nested key + header field round-trips; PUBLIC.snap loads (0 unmapped after T019),
  saves as 3.x and reloads idempotently (byte-identical second save). `diff()` + `entries()` give the T063
  basis. Header choices for migrated files documented in snapshot-migration.md §1 (factory `type`/`creator_fw`,
  `creator_model` ngc-full -> wing-fullsize); acceptance by real hardware is unverified.
  Files: crates/wing-model/src/{snapshot.rs,migrate.rs,json.rs} (new), src/lib.rs (exports, docs),
  src/store.rs (`ParamStore::set_raw`), tests/snapshot.rs, examples/snap.rs,
  docs/research/snapshot-migration.md. Test: `make check` (passes);
  `cargo run -p wing-model --example snap -- load fixtures/PUBLIC.snap --dump ch/1 [--report] [--save out.snap]`,
  `cargo run -p wing-model --example snap -- diff a.snap b.snap`.
  Proposed follow-ups: verify header acceptance and a migrated PUBLIC.snap on a 3.x console / WING-Edit
  (T062); consider a `persist` bit in schema.bin instead of the embedded factory-scene mask.
