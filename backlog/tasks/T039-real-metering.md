---
id: T039
title: Real metering
stage: S3
status: review
owner: agent-T039
depends: [T015, T035, T026]
branch: task/T039
---

## Goal
Feed real meters (inputs, strip taps, GR, busses, mains) to WING-Edit at protocol rate.

## Acceptance criteria
- [x] Meters match signal levels within 0.5 dB

## Log
- 2026-10-25 coordinator: created.
- 2026-10-25 agent-T039: implemented, status review.
  - Design: RT side folds per-block abs peaks (vectorised integer-max `abs_max`) into a lock-free
    `wing_engine::meters::LevelBank` (one AtomicU32 f32-bits cell per word, `fetch_max`, zero cells skipped,
    no alloc/locks). Writers: `ChannelStrip` (IN/PFL L/R, gate/dyn key + GR, via `set_meter`),
    `ProvisionalMain` (every strip's post-fader out, main 1 in/out, `$MAIN` 1/2), `PortMeters` after each
    cycle (USB In 1..48, OSC 1/2, USB Out 1..48). Enabled by `ConsoleConfig::meters`.
  - Sender: `wing_sim::meters::RealMeters` runs in `MeterStreamer::spawn_with_source`'s per-tick hook
    (50 ms): take interval peak -> dBFS 1/256 (floor -128 dB), ballistics = instant attack + 11.8 dB/s
    release (IEC 60268-18; protocol docs specify none), GR word = 256*GR/full-scale (gate 60 dB, dyn 20 dB),
    V2 gate LED / dyn state = GR > 0.5 dB [H], automix 0. No RMS (WING reports none).
    Synthetic fallback while no audio session is attached (`LevelBank::is_attached`, set by the audio
    control thread) or with `--no-audio`.
  - Gaps (documented in apps/wing-sim/src/meters.rs): bus 1-16, main 2-4, mtx, $BUS/$MTX/USR sources
    silent until T036/T037 write their `StripMeter`/ports (hooks + cells exist; see wing_engine::meters
    docs); non-USB output groups silent (not rendered); monitor = main 1 until T038; DCA/FX/RTA silent.
  - Files: crates/wing-engine/src/{meters.rs (new), channel.rs, lib.rs}, benches/meters.rs (new),
    tests/channel_strip.rs; crates/wing-proto/src/nrp/meter.rs (gr_to_meter), server/meters.rs
    (spawn_with_source); apps/wing-sim/src/{meters.rs (new), audio/console.rs, audio/driver.rs, lib.rs,
    main.rs}, tests/meters.rs (new), tests/audio.rs (metering on in alloc/load tests).
  - Tests: `cargo test -p wing-sim --test meters` (sine -> UDP packets within 0.5 dB at ch in/out, main
    in/out, USB source + output groups; GR word == strip telemetry within 0.5 dB), `--test audio`
    (rt_path_does_not_allocate with meters on), wing-engine strip meter test. `make check` passes.
  - Bench: `cargo bench -p wing-engine --bench meters`: abs_max(64) 2.6 ns, strip +~10 ns, 98 ports
    0.29 us; full console ~1 us/cycle (0.07-0.08% of the 1.33 ms period; release test prints 30.1 vs 31.0 us).
  - Proposed follow-ups: confirm GR word scale/sign, LED semantics and real-console ballistics against a
    WING capture; T036 to call `StripMeter` for bus/main/mtx and add mix-output ports to `PortMeters`
    (and replace ProvisionalMain's metering); meter non-USB output groups if they become rendered.
