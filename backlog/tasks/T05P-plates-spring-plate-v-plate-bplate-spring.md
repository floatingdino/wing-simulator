---
id: T05P
title: Plates/spring: PLATE, V-PLATE, BPLATE, SPRING
stage: S5
status: done
owner: agent-T05P
depends: [T05R]
branch: task/T05P
---

## Goal
Per models.md §6.

## Acceptance criteria
- [x] Decay/brightness tests

## Log
- 2026-10-25 coordinator: created by decomposing epics T040-T044/T051 after T004.
- 2026-10-25 agent-T05P: done, status review.
  - Plates (crates/wing-blocks/src/models/fx_reverb/plate.rs) on the T05R FDN core, 16 lines each, configured as
    plates (short lines, Dattorro-length input diffusers, strong in-loop allpasses, ER window 0.3..15 ms, bright
    hf_ratio): PLATE (Lexicon plate, ROOM-like with `att` -> build-up shape and recirculating pre-echoes),
    V-PLATE (EMT 140: fixed geometry, no chorus; `col` -> damping 5k x 2^(col/10) + hi cut 8k x 2^(col/10)),
    BPLATE ("Blue Plate": hardware reference still unknown after a web search; designed as a modern modulated
    plate on the HALL-family parameter set, documented [I]). Every WING param mapped (tables in the file docs).
  - Core API extensions (core.rs, minimal): `CoreParams::xover_hz` (0 = character's; glided, BPLATE `xover`),
    `MAX_PRE_MS` = 250 (V-PLATE pre-delay), `Glide`/`LaneBiquads`/`flush` made `pub(super)` for spring.rs.
    concert.rs gained `..CoreParams::default()` for the new field.
  - SPRING (spring.rs, own DSP): 4 springs per side as 8 SIMD lanes; each = 80 stretched first-order allpasses
    A(z^K) (K = round(fs/9600), chirp band edge ~4.8 kHz, group delay rising with frequency) in a loop with an
    inverting round trip (27.8..44.9 ms "length", a = 0.62..0.71 "tension", 0.04 ms wobble, Lagrange read) and an
    in-loop HF damping shelf; decay by z^-1 -> r z^-1 on every loop delay (exact T60 at all frequencies);
    4th-order LP at the band edge; `dens` fades in springs 2..4; `low`/`high` = per-sample-ramped first-order
    shelves (+ treble sets in-loop damping).
  - Registration: model.rs (4 lines + fx_implemented line), fx_reverb/mod.rs (`pub mod plate; pub mod spring;`,
    constructors, `create`), Cargo.toml `[[bench]] fx_plate`.
  - Tests: fx_reverb/tests_t05p.rs (14: T60 vs dcy +-10 % for all 4 models incl. size/density extremes, actual
    errors <= 4.4 %; bass mult / BPLATE crossover / damping; V-PLATE colour + SPRING treble brightness; plate
    echo density >= 0.97 at 60 ms; spring group delay 1.7 -> 2.2 -> 3.4 -> 8.3 ms at 0.6/1.5/2.5/3.5 kHz and
    matching the analytic chain; drips per round trip; silence -> exact zero; extremes stable; click-free sweeps;
    wet level; param round trip), spring.rs unit tests (2), tests/fx_plate_no_alloc.rs (zero alloc).
    T05R test `create("PLATE").is_none()` changed to `create("GEQ")`.
  - Bench (`cargo bench -p wing-blocks --bench fx_plate`, 64-sample stereo block @48k): PLATE/V-PLATE/BPLATE
    5.9 us, SPRING 14.0 us.
  - `make check` and `make check-full` pass.
  - Proposed follow-ups: verify plate/spring voicing and BPLATE identity against a console capture.
