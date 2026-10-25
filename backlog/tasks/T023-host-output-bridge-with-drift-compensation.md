---
id: T023
title: Host output bridge with drift compensation
stage: S2
status: done
owner: agent-T023
depends: [T022]
branch: task/T023
---

## Goal
CoreAudio client to any host device; mapping config 'USB out 45/46 -> Scarlett 2i4 1/2' (also sim internal outputs/monitor). Adaptive resampling for clock drift; configurable latency.

## Acceptance criteria
- [ ] Hear REAPER audio on Scarlett via the sim
- [ ] Drift test: 1h no buffer under/overflow

## Log
- 2026-10-25 coordinator: created.
- 2026-10-25 agent-T023: implemented new crate `crates/wing-hostio` (status: review).
  - Design: engine -> `BridgeProducer::push_with` -> lock-free SPSC planar ring (wait-free, seqlocked
    `(host time, write index)` push stamp) -> `BridgeConsumer::render` in the device's HAL IO proc:
    fill estimate (producer modelled as a line via a lower-envelope filter on `push_time - index/fs`,
    removing block/burst sawtooth and wake-up jitter) -> PI controller (0.015 Hz, zeta 1, 1 s error
    LPF, +-2000 ppm clamp, 200 ppm/s slew, anti-windup) -> variable-ratio polyphase Kaiser-sinc
    resampler (64 taps, 512 phases, linear phase interpolation; no alloc in callback). Target fill =
    device buffer + producer burst + `target_latency` (default 5 ms). Priming/underrun -> silence and
    re-prime; large excess -> resync. Stats (`BridgeStats`: state, fill, target, correction ppm,
    ratio, underruns, resyncs, overflow, callbacks, device rate/buffer, reconnects) are atomics.
  - CoreAudio: hand-written HAL FFI (no coreaudio-sys -> no bindgen/libclang; no AUHAL needed).
    `coreaudio::HostOutput` opens a device by UID/name (exact, case-insensitive, substring), maps
    1-based device channels across output streams, checks f32 virtual format, optional buffer-size
    request. A supervisor thread re-opens on device-list / IsAlive / nominal-rate / buffer-size
    notifications (HAL notification thread enabled) plus a 500 ms poll -> unplug/replug and 44.1/48k
    changes (resampler rebuilt for the new rate). The IO proc thread is already a member of the
    device's os_workgroup; `HostOutput::io_workgroup()` exposes it (retained, as `RawWorkgroup`) for
    helper threads. The engine thread deliberately does not join it (different clock domain).
  - Resampler: custom instead of rubato (fixed-chunk API would need an extra staging FIFO for the
    variable-size pull callback; API churn). Measured THD+N < -110 dB (f32-limited, -114..-139 dB) at
    ratios 1, +-200 ppm, 0.1 %, 48<->44.1k; passband +-0.05 dB to 20 kHz (18 kHz for 48->44.1);
    23 kHz alias < -80 dB.
  - Config (`config.rs`, revised per coordinator: monitor-style taps): source side is a general tap
    reference, stable enum `Tap { Monitor(1=A,2=B), Main(1-4), Bus(1-16), Matrix(1-8), UsbOut, LocalOut }`
    + `TapChannel { tap, lane }` (stereo taps: lanes L/R, `.L`/`.R` picks one side; flat taps: channel
    numbers). Lines: `mon/1 -> "Scarlett 2i4 USB" 1/2`, `main/1`, `bus/3.L`, `usb_out/45-46` (legacy
    `usb_out 45/46` accepted for flat taps). Engine resolves taps to ports later (T024; monitor/solo
    in T038 - not implemented here). Default config `crates/wing-hostio/hostio.example.toml`
    (`EXAMPLE_TOML`, `HostIoConfig::example()`): monitor A -> Scarlett 1/2. TOML `[hostio]` with `latency_ms`, `buffer_frames`,
    `[[hostio.route]]` (`map = ...` or `source`/`device`/`channels`, per-route `latency_ms`);
    `HostIoConfig::devices()` groups per device and rejects double-mapped device channels.
  - Files: Cargo.toml (member, serde/toml workspace deps, dev opt-level 2 for wing-hostio so the hour
    simulations take ~2 s), crates/wing-hostio/{Cargo.toml, src/{lib,ring,resampler,control,bridge,
    config}.rs, src/coreaudio/{mod,sys,device,output}.rs, tests/drift.rs, benches/resampler.rs,
    examples/{list_devices,tone_to_device}.rs}, CLAUDE.md + docs/ARCHITECTURE.md (crate list).
  - Tests (`make check` passes): tests/drift.rs simulates 1 h at +200 ppm (256-frame callbacks), 1 h
    at -200 ppm (128), 10 min 44.1 kHz device at +150 ppm, 10 min bursty producer (8x64 back to back),
    10 min with the virtual clock also off (+80 ppm vs host); 300 us producer jitter, 40 us callback
    stamp jitter. Result: 0 underruns/overflows/resyncs, |fill - target| <= 2.5 frames after settling,
    mean correction within 0.05 ppm of the true drift, steady-state wander < 3 ppm p-p, per-callback
    change < 0.1 ppm. Unit tests: ring, resampler quality, controller, bridge prime/underrun/
    reconnect-at-new-rate, config parsing, device enumeration smoke test.
  - Hardware: `cargo run -p wing-hostio --example tone_to_device -- "Scarlett 2i4 USB" 1 2` run once
    (440 Hz, -30 dBFS, 3 s): Scarlett 48 kHz/512, fill held at 816/816 frames (17 ms), corr ~-3 ppm,
    303 callbacks, 0 underruns (audibility for the user to confirm). `--example list_devices` works.
  - Not done: acceptance "hear REAPER audio on Scarlett via the sim" needs T024 (usb_out source ->
    engine port routing + daemon wiring). Live unplug/replug not exercised on hardware.
  - Proposed follow-ups: (1) T024/daemon: resolve `TapChannel` -> engine `Port` (T038 for `Tap::Monitor`), create one bridge per
    `DevicePlan` in wing-sim, push from `CycleIo::after_cycle` with `host_time_ns()`, expose
    `StatsSnapshot` on the status API; (2) hardware soak: replug the Scarlett and switch it to 44.1 kHz
    while running; (3) optional set-rate to match the sim rate when no other client uses the device.
