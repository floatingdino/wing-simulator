---
id: T070
title: Perf suite: full scene 48ch + 16 FX
stage: S7
status: review
owner: agent-T070
depends: [T03E, T050]
branch: task/T070
---

## Goal
Benchmarks + profiling; target <30% cycle at 64 samples.

## Acceptance criteria
- [x] Bench report (docs/perf.md; `wing-render --bench`, `cargo bench -p wing-render --bench perf`)

## Log
- 2026-10-25 coordinator: created.
- 2026-10-25 agent-T070: done, status review.
  - Suite: `apps/wing-render/src/perf.rs` + `wing-render --bench` (scenes public / everything-on, 64/128,
    worker scaling 0..12 hot, paced RT p99/p99.9, `--profile` per-node costs + level bound, `--models` survey
    with output digests, `--digest`, `--no-lpt`, `--set`), criterion bench `apps/wing-render/benches/perf.rs`,
    tests `apps/wing-render/tests/perf_scenes.rs` (serial == parallel bit-identity, LPT on/off).
    Engine hooks: `Engine::process_cycle_profiled`, `Graph::num_outputs_of`, `Console::cycle_profiled`/`engine_mut`.
  - Optimisations (all bit-exact; scene + model digests identical to b3e2264): NEON fixed-order dots
    (`wing_dsp::simd`), batched half-band stages with register-resident taps, 76LA/NSTR control path split
    (vector curve pass, register ballistics, batched FIR), `lin_to_db_fast4`, TapLine slice copies, LPT claim
    order in the level scheduler (`Engine::set_load_balancing`).
  - Headline (M4 Max, background load present): everything-on 64 samples paced RT 7/8/10 workers:
    258/239/216 µs mean = 19/18/16 % of 1333 µs, p99.9 <= 24 % (before 360/339/306 µs, p99.9 32-35 %).
    Serial work 2606 -> 1664 µs. PUBLIC 140 -> 119 µs at 7 workers. 76LA 19.9 -> 10.1 µs, NSTR 20.4 -> 11.7 µs.
  - Test: `make check` (passes), `cargo test --workspace --release -- --include-ignored` (passes).
  - Proposed follow-ups (output-changing or design work, see docs/perf.md): ENHANCE libm log1p/exp per sample
    (16.7 µs; 28 µs inside *MASTER* on PUBLIC's critical path), amp sims f64 4x chains, 76LA/NSTR ballistics at
    16x, reverb ER/Lagrange reads, DAG scheduler / split strips at heavy inserts, default worker cap 7 -> 8-10.
