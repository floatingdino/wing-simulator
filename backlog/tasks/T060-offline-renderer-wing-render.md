---
id: T060
title: Offline renderer (wing-render)
stage: S6
status: review
owner: engineering-agent
depends: [T026, T037, T038]
branch: task/T060-T061
---

## Goal
Multitrack WAVs (or REAPER-rendered stems) + .snap -> mains/stems faster than realtime with identical DSP.

## Acceptance criteria
- [x] Bit-identical to realtime path for same input (`apps/wing-render/tests/determinism.rs`)

## Log
- 2026-10-25 coordinator: created.
- 2026-10-25 engineering-agent: implemented on `task/T060-T061` (with T061).
  - Shared console path: no refactor needed. `wing_sim` (lib) already owns `Console`/`prepare`; wing-render
    depends on it and builds the console exactly like the daemon's audio service (`prepare` → routing refresh →
    `Console::build` → `warm_up(SESSION_WARM_UP_CYCLES)` → `Console::cycle` per block), driven back-to-back
    (offline clock: no waiting). wing-sim behaviour unchanged apart from T061 additions. Note: the engine's
    `OfflineClock` drives a bare `Engine` and skips the parameter pipeline/telemetry that `Console::cycle` runs, so
    the renderer loops `Console::cycle` itself rather than going through `Clock::tick`.
  - CLI: `wing-render --scene mix.snap (--inputs dir|files | --rpp X.RPP) [--map usb:1=f.wav,usb:3-4=st.wav,usb:5=f.wav:2|:mix]
    --out DIR [--taps main/1,bus/3,mon/1,usb_out/1-48] [--rate 48000] [--threads N] [--start/--end/--tail SECS]
    [--block 64] [--set path=value] [--dry-run] [--quiet]`. Inputs: leading track numbers in file names
    (`03-Kick.wav` → USB 3) else sorted order; stereo = 2 inputs. 16/24/32-bit PCM + 32/64-bit float WAV
    (own dependency-free reader/writer in `apps/wing-sim/src/wav.rs`, shared with T061). No resampling.
  - REAPER import (`apps/wing-render/src/rpp.rs`, read-only): track NAME/MUTESOLO/HWOUT (`&1023` dest, `|1024` mono,
    source-channel field), first WAV item (POSITION/SOFFS/LENGTH/MUTE), FILE relative to project dir then RECORD_PATH.
    Limits (documented in the module): first audio item per track only, no fades/volumes/pan/FX/envelopes, mono out
    from stereo source = (L+R)/2, MASTERHWOUT ignored, WAV only. Dry-run against the user's
    `2025-06-29_AM.RPP` (read-only) maps 23 tracks correctly and reports muted tracks.
  - Outputs: 32-bit float WAV per tap group (stereo per main/bus/mtx/mon, mono per usb_out lane) +
    `render_report.json` (per tap peak dBFS, integrated LUFS (BS.1770-4 K-weighting + gating, `loudness.rs`,
    EBU 3341 cases tested), frames, render seconds, speed ×realtime, input map, notes).
  - Determinism test: PUBLIC.snap (channels switched to USB, USB Out 45-48 = MAIN 1/BUS 1), 5 WAV sources (24-bit
    and float, mono+stereo), offline render vs the real `AudioService` (RT thread + workers) on `DriverSim` with a
    private shm name: all 48 USB outs bit-identical for 49000 frames; render identical for 0 vs 3 workers.
  - Benchmark (release, this Mac, 48 x 60 s 24-bit inputs, PUBLIC.snap + `--set io/altsw=1`): 21.5x realtime with
    taps main/1,bus/1,mon/1 (7 workers; 18.4x with 3, 7.0x single-threaded); 16.5x with all 48 usb_out lanes too.
  - Files: apps/wing-render/{Cargo.toml,src/lib.rs,src/main.rs,src/inputs.rs,src/rpp.rs,src/render.rs,
    src/loudness.rs,tests/determinism.rs}; apps/wing-sim/src/{wav.rs,lib.rs}.
  - Test: `make check` (passes); `cargo test -p wing-render`; manual: `cargo run --release -p wing-render --
    --scene fixtures/PUBLIC.snap --inputs <dir> --out /tmp/r --taps main/1 --set io/altsw=1`.
  - Proposed follow-ups: the daemon only heartbeats between driver bursts, so a console slower than realtime
    (debug build) that is fed continuously never leaves `service()` and the driver drops to loopback after 100 ms
    (the determinism test paces its feed); consider a heartbeat per block in `driver::service`. Resampling of
    non-48k inputs; RF64 for >4 GiB renders; multi-item REAPER tracks.
