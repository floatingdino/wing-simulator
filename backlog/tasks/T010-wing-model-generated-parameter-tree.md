---
id: T010
title: wing-model: generated parameter tree
stage: S1
status: review
owner: agent-T010
depends: [T001, T003]
branch: task/T010
---

## Goal
Codegen (build.rs or tool) from data/schema/wing-tree.json into a flat, index-addressed parameter store (Vec of atomics/f32 + metadata table) with path<->id<->hash lookup, defaults, clamping/quantisation, enum mapping and change notification queue.

## Acceptance criteria
- [x] All schema nodes addressable by path/hash
- [x] Unit tests for range/enum/taper conversions
- [x] Lookup O(1) and allocation-free get/set

## Log
- 2026-10-25 coordinator: created.
- 2026-10-25 agent-T010: implemented; status review.
  - **Codegen choice**: checked-in generator `tools/schema-extract/gen_model.py` -> binary blob
    `crates/wing-model/src/schema.bin` (1.1 MB, `include_bytes!`, decoded once via `Schema::get()`).
    Chosen over build.rs (needs serde_json build-dep, re-parses 5 MB JSON per clean build) and over a
    generated .rs (~43k struct literals compile slowly). wing-model clean build: 0.3 s debug / 0.6 s
    release; `make check` ~6 s warm. Regenerate: `python3 -I tools/schema-extract/gen_model.py`;
    `--check` fails if the blob is stale. Blob format documented in the generator docstring.
  - **Schema** (`schema.rs`): 43,008 concrete nodes (preorder; parent/first-child/next-sibling),
    1,396 deduped descriptors (type, class, flags, unit, min/max, step/logstep, steps, fmt, describe,
    enum tokens + display names, maxlen), per-node factory defaults (firmware 3.0.6 scene, baked in
    by the generator = the "factory scene loader"), 54 model groups / 1,174 models / 2,605 block
    instances. Hash computed from path at init; O(1) open-addressing hash->id; path->id = hash +
    path verification (alloc-free); `children()`, `write_path()`.
  - **Store** (`store.rs`): `AtomicU32` value + `AtomicU16` active descriptor per node, string table
    (fixed buffers, per-slot mutex). `set*` returns `Ok(changed)`, validates against the active model
    (`SetError::Inactive` for unused slots). Setting a selector switches the model and applies the
    model's `pardef` defaults. `lookup()` accepts model names (`ch/1/dyn/thr`); active-model names
    win over numeric slot names for text paths (`fx/1/40` under GEQ = 40 Hz band in slot 5).
    Change notification: lock-free MPSC ring (Vyukov, `queue.rs`) + per-param dirty flags, so changes
    coalesce and the queue cannot overflow (`RtConsumer::pop` returns the latest value), plus
    synchronous broadcast hooks with an `Origin` tag for protocol servers.
  - **Conversions** (`value.rs`): lin/log grid quantisation (incl. reversed ranges like
    `cfg/mon/*/dim` 40..0), fader law both ways (no dB quantisation: factory values like -6 dB are
    off-grid), normalised positions, enum by index/token/display name, fenum nearest, text parsing
    (`2k00`, `-oo`, `ON`, `m:ss`), WING-Edit display formats (`%+.1f`, `%k.1f`, `%kf`, `%T`).
  - **Coordinator note (T004 numeric keys)**: EQ-slot variants already had `40` (ch MACH4, slot 2) and
    `31` (bus/main/mtx PIA, slot 2); bus/main/mtx EQ selectors offer PIA (not MACH4), aux neither
    (tested). The real gap was in **FX** variants: T003 dropped params whose name equals an existing
    slot index. Filled (descriptor flag `from_models_md`, copied from a same-range sibling per
    models.md): fx GEQ `40` (slot 5), PIA `31` (slot 3), MACH4 `40` (slot 3), *MASTER* `40` (slot 7),
    for fx 1-8 and 9-16. Generator warns on any remaining slot gap (none). Also re-parsed 41 int
    ranges with units (`int [0 .. 100 %]`) that T003 left as 0..0, and matched the truncated VSS3
    preset default (`Medium Vocal Ha`).
  - **Tests** (`cargo test -p wing-model`: 24 unit + 3 integration): hash parity for all 43,007
    JSON `hashes` entries (wing_hash, by_path, by_hash, path round-trip); display formatting
    reproduces WING-Edit's `describe` bounds for >1,000 descriptors; conversion edge cases (NaN,
    out-of-range, -0, -oo); model-switch defaults (GATE/DUCK/LA, dly mode units, fx GEQ/PIA, bus
    PIA); defaults lie on their grids; RT coalescing + hooks; zero-alloc get/set/lookup/model
    switch/string set/format (counting allocator; one warm-up iteration because macOS std Mutex
    boxes on first lock). `make check` passes.
  - **Bench** (`cargo bench -p wing-model`, M-series): by_hash 0.8 ns, lookup path 20 ns, lookup
    model name 34 ns, get 1.4 ns, set (changed, notifies) 6 ns, wing_hash 6.5 ns, `ParamStore::new` 0.19 ms.
  - Files: `crates/wing-model/{Cargo.toml,src/{lib,hash,schema,store,value,queue}.rs,src/schema.bin,
    tests/{hash_parity,zero_alloc}.rs,benches/lookup.rs}`, `tools/schema-extract/gen_model.py`, Cargo.lock.
  - Proposed follow-ups: (1) `%t` format (wlive card times) unverified: describe shows `0:00:00` for
    both bounds; treated as ms -> h:mm:ss. (2) Centre-detent pan variant (flag byte +0x100) is not in
    the JSON; add to the T003 extractor if needed. (3) Delay-mode switches apply the new mode's
    default rather than converting the distance; check console behaviour. (4) Fix the T003 extractor
    for numeric-name model params and unit-suffixed int ranges so the generator fills become no-ops.
    (5) `make bench` only runs wing-dsp; consider adding `-p wing-model`.
