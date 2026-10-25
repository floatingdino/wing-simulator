---
id: T084
title: RTA support for WING-Edit ("RTA in use! Check mixer settings.")
stage: S6
status: review
owner: agent-T084
depends: []
branch: task/T084
---

## Goal
WING-Edit's EQ pages show "RTA in use! Check mixer settings." Find what WING-Edit expects (cfg/rta/* nodes: rtasrc, rtatap, decay, detector, range, gain, auto; the RTA meter report type in the meter protocol; any $-nodes signalling RTA availability) from the binary (strings/symbols, Unicorn emulation if needed — never run WING-Edit natively), protocol docs and libwing; then implement an RTA in the daemon: FFT analyser on the selected source/tap (off the RT thread: RT side copies the tapped block into a lock-free ring), band layout and dB scaling as WING sends, decay/peak detector, streamed via the meter transport at WING's rate. The EQ page RTA overlay in WING-Edit should then work.

## Acceptance criteria
- [x] Root cause of the message documented
- [x] RTA data packets match WING's format (tests); live: overlay visible in WING-Edit (manual check by user: pending)
- [x] Zero RT alloc, analyser cost reported

## Log
- 2026-10-26 coordinator: created from user request.
- 2026-10-26 agent-T084: done, status review.
  - Root cause (docs/research/rta.md §1): WING-Edit's `RtaGraph::paint` draws "RTA in use! Check mixer
    settings." unless the console-owned status `cfg/rta/$src == rtasrc`, `$tap == rtatap` and
    `$dec == rtadecay` (positions at `Cpar+0xac`). WING-Edit writes `rtasrc`/`rtatap` itself; the
    simulator never updated the `$` leaves (power-on `$dec` SLOW vs `rtadecay` MED already failed).
    Fix: `apps/wing-sim/src/rta.rs` `sync_status` mirrors rtasrc/rtatap/rtadecay/rtadet into
    `$src/$tap/$dec/$det` (origin local -> change events to every client) every 50 ms.
  - RTA data (rta.md §2): `AA` index 0, 120 BE int16 words; WING-Edit reads them as 1/128 dB
    (PEAK as is; RMS/AVG doubled then 1/256) [M, from its auto-gain code]. protocol.md updated.
  - Implementation: `crates/wing-engine/src/rta.rs` (RtaFeed: request/latch selection + lock-free
    overwrite ring; RtaProbe in ChannelStrip/MixStrip for IN/FILT/PRE EQ/POST EQ/keys/PRE FDR/
    PRE TAP/mix POST; RtaAnalyser: 120 semitone 4th-order band-passes, PEAK/RMS/AVG, decay);
    `apps/wing-sim/src/audio/console.rs` (latch per cycle, port taps: ch/aux POST, SOLO, MON.PH/
    SPK, FXIN); `apps/wing-sim/src/rta.rs` (RtaService thread `wing-rta`, writes the bank's AA
    block at 50 ms while audio is attached; synthetic RTA otherwise); main.rs wiring;
    RealMeters no longer overwrites AA; synthetic RTA now 1/128 dB.
  - Tests: wing-engine `rta::tests` (band layout, band accuracy at centres incl. neighbour
    rejection, PEAK/RMS/AVG levels, 44.1 kHz, decay, ring/lapping, latch/probe); wing-sim
    `rta::tests` (status mirror satisfies WING-Edit's condition; runner -> bank);
    `tests/rta.rs` (USB In 1 -> ch 1 -> AA UDP packet: size 4+240, id, bin, level for IN/PRE EQ/
    POST within 0.5 dB; source switch empties the display); `tests/audio.rs`
    `rt_path_does_not_allocate` now runs with the RTA feed and tap changes (0 allocations).
  - Cost: bench `meters` `rta/analyser_64` 5.5 us per 64-sample block (~0.4 % of a core at
    48 kHz, release); RT side = one relaxed load per strip + one block copy for the selected tap.
  - `make check`: pass. Not run: `make check-full`; live WING-Edit overlay check (user).
  - Assumptions [H/L]: filter shape (1/12 oct, 4th order), AVG = mean |y|, decay 6/12/24 dB/s,
    SOLO = monitor 1 solo bus. Proposed follow-ups: FXOUT tap (FX return port); verify the
    1/128 dB word scale and decay rates against a console capture.
