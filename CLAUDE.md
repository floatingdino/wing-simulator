# Wing Simulator: agent/developer guide

See docs/ARCHITECTURE.md for the design and backlog/README.md for the task protocol.

## Commands
- `make check`: `cargo fmt --check`, `cargo clippy --all-targets -D warnings`, `cargo test` (must pass before review)
- `make bench`: criterion benches (`cargo bench -p wing-dsp`)
- `make fmt`: format; `make build`: release build
- Toolchain: stable (rust-toolchain.toml). `.cargo/config.toml` sets `target-cpu=apple-m1` on aarch64-apple-darwin.
- Release profile: thin LTO, codegen-units=1, panic=abort.

## Crates
- `crates/wing-proto`: discovery, native binary protocol, OSC codec + server
- `crates/wing-model`: parameter tree schema, values, .snap load/save, routing tables
- `crates/wing-dsp`: DSP primitives (biquads, SVF, detectors, oversampling, smoothing, delays)
- `crates/wing-blocks`: WING processing blocks and alternate models
- `crates/wing-engine`: graph, routing, busses, scheduling, metering, RT threading
- `crates/wing-shm`: shared-memory ring layout shared with the C++ driver
- `apps/wing-sim`: daemon binary; `apps/wing-render`: offline renderer
- `driver/` (C++ HAL plugin), `data/schema`, `fixtures/`, `docs/research` as per ARCHITECTURE.md

## Conventions
- Keep dependencies minimal; criterion is dev-dependency only. Declare shared deps in `[workspace.dependencies]`.
- Each crate has a module doc and tests. Work on branch `task/<ID>`; follow backlog/README.md rules.
- Never commit WING-Edit.app, firmware binaries or large audio. Never run sudo.

## Real-time rules (audio threads)
- No allocation, locks, syscalls or logging on audio threads.
- Control-to-RT communication via lock-free queues; smooth parameters on the RT side.
- Every DSP block has a unit test and a bench.
