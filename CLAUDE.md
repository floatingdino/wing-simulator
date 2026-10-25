# Wing Simulator: agent/developer guide

See docs/ARCHITECTURE.md for the design and backlog/README.md for the task protocol.

## Commands
- `make check`: `cargo fmt --check`, `cargo clippy --all-targets -D warnings`, `cargo test` (fast set; ~2 min; run routinely, must pass before review)
- `make check-full`: same plus `cargo test --workspace --release -- --include-ignored` (~2 min incl. build); run before finishing a DSP task. Keep `make check` under 5 min: any test slower than ~10 s in dev must be `#[ignore = "slow: run with make check-full"]` with a sampled fast variant covering the same path. (wing-blocks/wing-hostio build at opt-level 2 in dev, see Cargo.toml.)
- `make bench`: criterion benches (`cargo bench -p wing-dsp`)
- `make fmt`: format; `make build`: release build
- `make app`: WingSimMenu menu-bar app (Swift, `apps/WingSimMenu/build.sh`), its Swift tests and a launch/quit smoke test (not part of `make check`)
- Toolchain: stable (rust-toolchain.toml). `.cargo/config.toml` sets `target-cpu=apple-m1` on aarch64-apple-darwin.
- Release profile: thin LTO, codegen-units=1, panic=abort.

## Crates
- `crates/wing-proto`: discovery, native binary protocol, OSC codec + server
- `crates/wing-model`: parameter tree schema, values, .snap load/save, routing tables
- `crates/wing-dsp`: DSP primitives (biquads, SVF, detectors, oversampling, smoothing, delays)
- `crates/wing-blocks`: WING processing blocks and alternate models
- `crates/wing-engine`: graph, routing, busses, scheduling, metering, RT threading
- `crates/wing-shm`: shared-memory ring layout shared with the C++ driver
- `crates/wing-hostio`: host output bridge (CoreAudio HAL IO proc, SPSC ring, drift-compensating resampler, route config)
- `apps/wing-sim`: daemon binary (`<state-dir>/status.json` + `control.sock` for front ends); `apps/wing-render`: offline renderer
- `apps/WingSimMenu`: SwiftUI menu-bar app (start/stop, status, host-output routing, scenes, diff)
- `driver/` (C++ HAL plugin), `data/schema`, `fixtures/`, `docs/research` as per ARCHITECTURE.md

## Conventions
- Keep dependencies minimal; criterion is dev-dependency only. Declare shared deps in `[workspace.dependencies]`.
- Each crate has a module doc and tests. Work on branch `task/<ID>`; follow backlog/README.md rules.
- Never commit WING-Edit.app, firmware binaries or large audio. Never run sudo.

## Real-time rules (audio threads)
- No allocation, locks, syscalls or logging on audio threads.
- Control-to-RT communication via lock-free queues; smooth parameters on the RT side.
- Every DSP block has a unit test and a bench.
