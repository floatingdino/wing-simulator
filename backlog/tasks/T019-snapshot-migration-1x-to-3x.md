---
id: T019
title: Snapshot migration: firmware 1.x keys -> 3.x schema
stage: S1
status: done
owner: eng-agent
depends: [T011]
branch: task/T011-T019
---

## Goal
fixtures/PUBLIC.snap (fw 1.13.9) has 1,709 keys absent from the 3.x schema (largest group: per-send ind/wid, 1,536 keys). Build a migration map so 1.x scenes load with correct mix intent, and document lossy cases. Exporting must write the 3.x format.

## Acceptance criteria
- [x] PUBLIC.snap loads with 0 unexplained unmapped keys
- [x] Mapping table documented with rationale per key family

## Log
- 2026-10-25 coordinator: created from T003 coverage report.
- 2026-10-25 coordinator: T004 alias list: WARM mix->wmix, output delay m->dly, dynxo HI6->type, solo mon A->PH, aux dyn has no mdl, missing filter slopes; plus send ind/wid.
- 2026-10-25 eng-agent: done (branch task/T011-T019). `migrate.rs` rewrites the flattened 1.x leaves before
  they are applied (legacy = creator_fw major < 3). 15 rules, table with rationale/lossiness per key family in
  docs/research/snapshot-migration.md §3. PUBLIC.snap: 0 unmapped, 0 rejected, 1,755 notes, 8 lossy (2 sends
  with 70 % width, 6 dynxo band+slope tokens). Finding: send `ind` = "independent of busmode" (the 4 ind=true
  sends are exactly the 4 whose mode differs from their bus's busmode), so ind/busmode migrate losslessly into
  3.x per-send `mode`. WARM `mix` -> `wmix` (else ch 20-24 would get a 5-30 % block mix); `dly/m` -> `mode=M` +
  `dly`; dynxo LO6/LO12/HI6/HI12/PC -> LO/HI/PEAK; solo mon A -> PH; aux dyn gets `mdl=CMB` (1.x combo keys
  map 1:1); missing `lcs/hcs` -> 12 (assumed, verify); mtx dir/1 -> dir; talk indiv -> A/B; safes L/C/R ->
  LEFT/CENTER/RIGHT; seldblclick/mondim bool conversions. fx `40` keys resolve natively. 142 values are
  re-quantised to 3.x grids (rel/hld law changes, <= 2.2 %), reported as `adjusted`. Saving writes 3.x
  (snapshot.11 header, no legacy keys).
  Files: crates/wing-model/src/{snapshot.rs,migrate.rs,json.rs} (new), src/lib.rs (exports, docs),
  src/store.rs (`ParamStore::set_raw`), tests/snapshot.rs, examples/snap.rs,
  docs/research/snapshot-migration.md. Test: `make check` (passes);
  `cargo run -p wing-model --example snap -- load fixtures/PUBLIC.snap --dump ch/1 [--report] [--save out.snap]`,
  `cargo run -p wing-model --example snap -- diff a.snap b.snap`.
  Proposed follow-ups: confirm on hardware the dynxo slope home in 3.x (S6/S12?), 1.x LC/HC slopes,
  seldblclick=true and non-zero safes formats (snapshot-migration.md §5).
