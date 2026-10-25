---
id: T05D
title: Delays: ST-DL, TAP-DL, TAPE-DL, OILCAN, BBD-DL
stage: S5
status: review
owner: claude-T05D
depends: [T050]
branch: task/T05D
---

## Goal
Per models.md §6; tap tempo sync.

## Acceptance criteria
- [x] Delay-time accuracy, feedback stability

## Log
- 2026-10-25 coordinator: created by decomposing epics T040-T044/T051 after T004.
- 2026-10-25 claude-T05D: started on task/T05D. Schema has no global tempo/tap key (checked data/schema), so each
  model uses its own time params (ST-DL / TAP-DL `fact` note factors cover dotted and triplet values).
- 2026-10-25 claude-T05D: done, status review.
  - Files: `crates/wing-blocks/src/models/fx_delay.rs` (ST-DL upgraded + TAP-DL, TAPE-DL, OILCAN, BBD-DL with
    every schema param; module doc lists all [I] mappings), `crates/wing-blocks/src/model.rs` (4 registration
    lines + fx_implemented list), `crates/wing-blocks/tests/fx_delay.rs` (max-feedback stability, no clicks on
    time change for all 5), `crates/wing-blocks/tests/fx_delay_no_alloc.rs`, `crates/wing-blocks/benches/fx_delay.rs`
    (all 5 models, with time changes during the bench).
  - Time changes: ST-DL / TAP-DL crossfade (30 ms, queued if mid-fade; ST-DL previously glided);
    TAPE-DL / OILCAN / BBD-DL rate-limited glide (f64 state; pitch bend <= 1.6x / 1.6x / 1.8x).
  - TAP-DL [I]: UltraTap-style, `time*fact` = span of `rep` evenly spaced taps after `pdel`; `slp` = level
    first->last (loudest tap 0 dB); `mode` = pan trajectory (MOVE sweep, JUMP alternate, FOCUS wide->centre,
    SPREAD centre->wide) x `wid`; `diff` = 4-stage allpass smear blend. Mono-summed input line.
  - TAPE-DL [I]: speed-dependent HF loss / head bump / wow-flutter depth, drive into soft saturator in the loop,
    loop gain normalised by the head-bump peak (max 0.95). OILCAN [I]: 40..500 ms, 3-point smear, wobble
    (delay + amplitude), tone LP 0.8..6.4 kHz. BBD-DL [I]: 4096 stages, 20..300 ms, BW = min(5 kHz,
    0.38 f_clk), S&H at f_clk, NE570-style feedback-detector compander (expander detector sees the same
    signal one delay later, so it tracks; a feed-forward compressor version mistracked by +1 dB and
    self-oscillated at FEED 100), hiss + clock feed-through companded.
  - Tests: `cargo test -p wing-blocks fx_delay` (17 unit + 2 integration + no-alloc), `make check` green.
    Bench (M1, stereo 64): ST-DL 0.7 µs, TAP-DL 16 taps+diff 2.4 µs, TAPE/OILCAN/BBD ~1.3 µs.
  - Proposed follow-ups: measure a console (impulse / tone sweeps per model) to replace the [I] tapers
    (OILCAN/BBD knob->ms, TAP-DL mode semantics, TAPE-DL speed-dependent EQ), and confirm whether ST-DL
    glides or crossfades on hardware.
