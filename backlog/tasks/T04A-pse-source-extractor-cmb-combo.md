---
id: T04A
title: PSE source extractor + CMB combo
stage: S4
status: done
owner: engineer-T04A
depends: [T045, T04F]
branch: task/T04A
---

## Goal
Per models.md §3.1 (identify PSE behaviour; document assumptions).

## Acceptance criteria
- [ ] Behaviour tests

## Log
- 2026-10-25 coordinator: created by decomposing epics T040-T044/T051 after T004.
- 2026-10-25 engineer: implemented PSE + CMB in crates/wing-blocks/src/models/pse.rs (SourceExtractor, PseKind). Research: PSE is in the gate slot (PUBLIC.snap `gate.mdl=PSE`), controls match the Waves Primary Source Expander -> soft 2:1 downward expander limited by depth, fast/peak = timing/detector; CMB = PSE + LA (OptoComp) in series (sources + assumptions in docs/VERIFY-ON-HARDWARE.md). Files: pse.rs, benches/pse.rs, tests/pse_no_alloc.rs, model.rs (registration; stub tests now use WAVE), models/mod.rs, Cargo.toml bench block. Test: `make check` passes (9 unit tests + no-alloc test). Bench: pse 0.29 us, cmb 1.7 us per 64-sample stereo block.
- Proposed follow-ups: confirm PSE law/timing on hardware (T072).
