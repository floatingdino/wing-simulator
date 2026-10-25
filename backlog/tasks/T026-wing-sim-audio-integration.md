---
id: T026
title: wing-sim audio integration (shm -> engine -> USB outs + host bridge)
stage: S2
status: review
owner: agent-T026
depends: [T020, T022, T023, T024, T03A, T035]
branch: task/T026
---

## Goal
Wire the realtime audio path into apps/wing-sim: DaemonLink (wing-shm) drives the Engine (LiveClock, workers joined to the driver/device workgroup, time-constraint policy); per cycle: read 48 USB inputs → routing (T024) → channel/aux strips (T035) bound to the ParamStore via the T03A pipeline → (busses/mains come in T036; until then a provisional post-fader stereo sum to main 1 is acceptable, clearly marked) → output patching → write USB outs back to the driver and feed host bridges (wing-hostio, config file e.g. `--hostio hostio.toml`, default monitor A/main 1 -> Scarlett 1/2 when present). CPU load/xrun stats in the status line and via OSC. Fallback behaviour when the driver isn't present (offline clock for testing).

## Acceptance criteria
- [ ] With the driver installed: REAPER plays the session, channel processing audible on the host device, edits in WING-Edit take effect live
- [x] Load %, xruns reported; zero RT allocations
- [x] Integration test with an offline clock and synthetic input

## Log
- 2026-10-25 coordinator: created — gap found: no task owned daemon audio integration. T025 milestone depends on it.
- 2026-10-25 agent-T026: implemented; status review. Merged main (T017, T021) first.
  - Files: `apps/wing-sim/Cargo.toml` (lib target + engine/shm/hostio deps), `src/lib.rs`, `src/audio/{mod,console,driver,hostio,stats,workgroup}.rs`,
    `src/main.rs` (flags `--no-audio --hostio PATH --no-hostio --shm-name --block --workers --no-rt`; audio status in the periodic status; stop = detach),
    `examples/audio_only.rs` (audio path only, no ports: safe next to a running wing-sim), `tests/audio.rs`.
  - Design: control thread `wing-audio` (attach/reattach every 1 s, `Segment::is_current` check every 1 s, routing refresh 10 ms, telemetry drain,
    console rebuild on rate change keeping the ParamPipeline, host outputs per session); RT thread `wing-audio-rt` per session: time-constraint policy
    sized to the driver's IO buffer (`input_frames`, re-applied only on a buffer change), member of an `AudioWorkIntervalCreate` work interval that the
    engine workers also join, `os_workgroup_interval_start/finish` around each burst; heartbeat → `DaemonLink::wait(10ms)` → `service()` = every
    complete 64-frame block: USB In → pipeline.apply → engine → USB Out → commit, + bridge push. Partial blocks wait for the next driver cycle.
  - Graph: externals USB In 1..48 → osc + input router (T024) → 40 ch + 8 aux `ChannelStrip` (T035, bound via T03A) → **provisional main 1**
    (`ProvisionalMain`, TODO T036: strip fdr/mute/linear-balance pan × `main/1/on,lvl`, then main/1 fdr/mute/pan; per-block linear ramps) →
    USB-out router with taps MAIN.1/2 and ch/aux PRE taps wired. Host taps: `main/1`, `mon/1..2` (= main 1 until T038), `usb_out/n`; others silent.
  - Host bridges: `--hostio file.toml` (wing-hostio format); default = `main/1 -> "Scarlett 2i4 USB" 1/2` only if that device exists; started
    per session (released when detached), producer burst sized max(512, driver buffer).
  - Version mismatch: never replaces the driver's segment; logs once e.g. "driver shm /wingsim.audio.v1 has layout v1, this daemon needs v2: the
    installed Wing Simulator driver is outdated ..." and stays control-only, retrying (verified against the installed v1 driver with
    `cargo run -p wing-sim --example audio_only -- --seconds 3`). Not run live end-to-end (v2 driver not installed yet); ports 2222/2223 untouched.
  - Status line (every 10 s while attached, and in the 60 s status): `audio: 48000 Hz, driver running, load x% (peak y%), cycle p50 a us p99 b us,
    xruns n (driver underruns u, overruns o), slow cycles s, rt ok (N fr), work interval, bridges: "Scarlett 2i4 USB" Running fill .. corr .. underruns ..`.
  - Tests (`cargo test -p wing-sim`): offline sine USB1 → ch1 (+6 dB trim, then STD EQ +6 dB @1k live via the store) → main → USB Out 1/2 per patch
    (±0.1 / ±0.6 dB), USB Out 3 direct patch bit-exact, host bridge consumer level, mute, live re-patch; RT path zero allocations/frees (counting
    allocator around `Console::cycle` + shm `service` with param changes, model swap, re-patch between passes); load 48 strips all blocks on
    (release: ~2 % avg, p99 40 us per 64-frame cycle with 7 workers; debug ~20 %); `AudioService` vs `DriverSim` on a private shm (real RT thread +
    os_sync wakes): attach, bit-exact passthrough via the rings, 0 underruns, 48k→44.1k rebuild, shutdown → loopback; v1 segment → control-only
    and untouched, "reinstall" → attach, segment recreated → reattach. `make check` passes.
  - Not done / proposed follow-ups: OSC read-only stats nodes (no schema node for DSP load; needs a wing-proto virtual-node hook — not trivial);
    C++ selftest harness not used as fake driver (its phases hard-code gain -0.5 per channel, not expressible as a WING routing; DriverSim, the
    Rust port of ShmLink, is used instead); live REAPER/WING-Edit acceptance once the v2 driver is installed (`audio_only --default-hostio` or
    wing-sim itself); bridge burst sizing is fixed at session start (a later larger driver buffer may underrun the bridge until restart);
    RoutingControl keeps mailboxes of readers from consoles dropped by rate-change rebuilds (tiny leak per rebuild; needs an unsubscribe in
    wing-engine routing/swap).

