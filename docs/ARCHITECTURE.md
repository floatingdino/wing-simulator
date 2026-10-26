# Wing Simulator — Architecture (v0, coordinator draft)

Goal: a macOS "virtual Behringer WING" that (1) appears to the OS as a 48x48 USB-style audio device,
(2) is controlled by the unmodified **WING-Edit 3.3.4** app over the network exactly like a real console,
(3) processes all audio with faithful emulations of every WING processing block and model, and
(4) can route any of its outputs to a real host device (e.g. USB out 45/46 -> an audio interface's outputs 1/2).

Status: decisions marked (D) are fixed unless research invalidates them; (?) are open and owned by a backlog task.

## Process layout

```
 REAPER ──CoreAudio──► [Wing Simulator HAL driver]  (AudioServerPlugIn, runs in coreaudiod)
   ▲                     48 out (DAW->WING "USB in") / 48 in (WING "USB out" -> DAW)
   │                          │ lock-free SPSC rings in POSIX shared memory (/dev/shm-like, mmap)
   │                          ▼
   └──────────────── [wing-sim daemon]  (Rust)
                        ├─ RT engine thread + N worker threads (os_workgroup joined, time-constraint policy)
                        ├─ control server: UDP 2222 discovery, TCP 2222 native protocol, UDP 2223 OSC
                        ├─ parameter model (full WING tree, snapshot load/save, .snap round-trip)
                        └─ host output bridge: CoreAudio client to real device(s) w/ drift-compensating resampler
 WING-Edit ──TCP/UDP──┘
```

- (D) **Driver**: C++17 AudioServerPlugIn built on **libASPL** (MIT). Thin: no DSP, only copies to/from shm rings,
  publishes timestamps. Device name "Wing Simulator", 48 in / 48 out, 44.1/48 kHz (WING runs 48k; 44.1 optional).
- (D) **Engine/daemon**: Rust workspace (perf, safety, threads). No allocation, locks or syscalls on RT threads.
  Parameter changes flow control->RT via lock-free queues; smoothed per-sample/per-block on the RT side.
- (D) **Parallelism**: per-block DAG: channel strips (40 ch + 8 aux) in parallel -> bus/main/mtx summing -> FX
  slots -> bus/main strips -> outputs. Fixed worker pool joined to the device's os_workgroup; work claimed via
  atomic counters (no rayon on RT path). Target: full scene incl. 16 FX at 48k/64-sample blocks < 30% of one cycle
  wall time on M4 Max.
- (D) **Clocking**: driver clock is master for the DAW side. Host output bridge uses an adaptive resampler
  (drift estimation) to the external device. Extra latency budget: 1 block in the engine.
- (D) **Offline render**: same engine driven faster-than-realtime from files (bounce mixes, regression tests).
- (?) Exact native-protocol framing, node IDs/hashes, metering transport -> T002/T003.
- (?) Model catalog and parameter ranges for every block -> T003/T004.

## Repository layout (target)

```
crates/wing-proto    discovery, native binary protocol, OSC codec + server
crates/wing-model    parameter tree schema (generated), values, snapshot (.snap JSON) load/save, routing tables
crates/wing-dsp      primitives: biquads, SVF, envelope/detectors, oversampling, smoothing, delay lines (SIMD)
crates/wing-blocks   WING processing blocks + all alternate models (filter, gate, eq, dyn, insert FX, FX rack)
crates/wing-engine   graph, routing, busses, scheduling, metering, RT threading
crates/wing-shm      shared-memory ring layout shared with the driver (also a C header)
crates/wing-hostio   host output bridge: CoreAudio client, SPSC ring, PI-steered resampler, route mapping
apps/wing-sim        daemon binary (CLI + config: host routing, scene file)
apps/wing-render     offline renderer (multitrack WAVs + .snap -> stems/mix)
driver/              HAL plugin (C++/libASPL), installer scripts
data/schema          extracted parameter schema (JSON) — generated from research
fixtures/            PUBLIC.snap and test audio
docs/research        findings (protocol, schema, signal flow, models, dsp reuse, driver)
backlog/             the task system (see backlog/README.md)
```

## Fidelity strategy
The firmware DSP cannot be lifted directly (see docs/research/firmware.md). Fidelity is achieved by:
1. Exact parameter semantics/ranges/tapers from WING-Edit and the snapshot schema.
2. Signal-flow order and tap points from the WING manual + routing tree.
3. Model emulations built from public descriptions of the hardware each model references, reusing permissive
   open-source DSP where available, and validated by measurement tests (frequency/phase response, GR curves,
   time constants) against documented curves.
