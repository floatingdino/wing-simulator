---
id: T045
title: Model framework + fidelity test harness
stage: S4
status: review
owner: agent-T045-T035
depends: [T030, T010]
branch: task/T045-T035
---

## Goal
In crates/wing-blocks: Model trait (prepare(sr,max_block), process(stereo/mono, sidechain), latency(), set_param by schema slot, reset), per-block model registries (filter/gate-dyn/eq/fx) with preallocated instances so model switches never allocate on the RT thread; stereo-link; a fidelity harness that measures magnitude response / static gain curves / time constants and compares to data/reference-curves (T00A) with tolerances.

## Acceptance criteria
- [x] Trait + registry used by T031-T034 standard models (port them)
- [x] Harness runs against at least one reference curve file

## Log
- 2026-10-25 coordinator: created by decomposing epics T040-T044/T051 after T004.
- 2026-10-25 agent-T045-T035: done, status review (branch task/T045-T035).
  - Files: `crates/wing-blocks/src/model.rs` (`Model` trait: id/prepare/process(l,r,sidechain)/latency/reset/
    set_param(schema slot, `ParamValue`)/param/load_defaults/commit/telemetry/is_stub/todo; catalogs
    `FILTER_TOOL_IDS`, `DYN_IDS` (33 incl. hidden 3CMP), `EQ_IDS`, `FX_IDS` (65 incl. NONE) from models.md;
    `create_model` factory; `StubModel` passthrough with owning task; `ModelRegistry` one preallocated
    instance per model, RT-safe `select`), `src/std_models.rs` (GATE/DUCK, COMP/EXP, STD EQ ch/aux/bus,
    LC/HC filter, input stage behind `Model`; DSP untouched, adapters only, params staged + applied on commit),
    `src/fidelity.rs` (magnitude response via IR+DTFT, DC static curve, 10-90 % attack/release, `Deviation`,
    reference loaders `FreqCurves`/`DynStatic` for the T00A `wing-reference-curves/1` format with a built-in
    JSON reader, `cascade_db` for the reference biquads; missing files -> `Ok(None)` = skip),
    `tests/fidelity_reference.rs`, `tests/model_no_alloc.rs`.
  - Engine integration (T03A landed first): `impl RtModel for Box<dyn Model>` + `model_factory(kind)` /
    `block_kind(group)` in `crates/wing-engine/src/params/std_models.rs` replace T03A's interim adapters, so
    every schema model of every group gets an instance (stubs pass through). `RtModel` gained
    `process_keyed` (side-chain) and `latency` (defaults keep old impls working). The wing-blocks-side slot
    wrapper I first wrote was dropped in favour of T03A's `ModelSlot` (crossfade + on/mix/gain) to avoid two
    wrappers.
  - Fidelity results (current DSP, printed by `cargo test -p wing-blocks --test fidelity_reference -- --nocapture`):
    LC all slopes <= 0.045 dB (asserted < 0.1); GATE static law 0.000 dB (asserted < 0.1); KNOWN GAPS for
    T03C (reported, not asserted): HC up to 16 dB near Nyquist (WING HC is not bilinear), STD EQ median
    ~3-5 dB (Q/shelf convention), COMP knee up to 2.7 dB (knee width 2 dB/step vs WING 7.2 dB/step), EXP up to
    92 dB (90 dB floor vs none). T03C can tighten the asserts as it calibrates.
  - Test: `make check` (passes). Debug builds thin the reference grids 4x (`THIN`) to keep `make check` fast.
  - Proposed follow-ups: time-constant comparisons once T00A exports per-model posted DSP parameters (needs
    T = 1/fs hook); FX models need `prepare`-time max latency reporting for delay compensation (T050);
    filter-tool references exist in filter.json for T046.
