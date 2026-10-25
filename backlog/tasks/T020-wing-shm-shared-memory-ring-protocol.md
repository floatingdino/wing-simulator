---
id: T020
title: wing-shm: shared-memory ring protocol
stage: S2
status: done
owner: claude-T020
depends: [T001, T006]
branch: task/T020
---

## Goal
Productionize the driver<->daemon shared-memory layout: versioned header, 48x48 planar float rings, timestamps/sample-time, heartbeat, xrun counters, C header generated/kept in sync with Rust.

## Acceptance criteria
- [x] Rust + C++ tests exchanging audio across processes
- [x] Lock-free, wait-free on RT side

## Log
- 2026-10-25 coordinator: created.
- 2026-10-25 claude-T020: done, ready for review.
  - Layout v2 (`WING_SHM_VERSION 2`, name unchanged `/wingsim.audio.v1`): writer-separated 128-byte
    cache lines; ring ctl split into producer and consumer lines; new fields `segment_id`, `creator_pid`,
    `cycle_seq` (wake word), `input_frames`, `input_time` (record-only clock), `daemon_waiting`;
    `magic` published last (release); validate also checks header_bytes and offsets.
  - Wake-up: `os_sync_wait_on_address`/`wake_by_address_all` (SHARED) on `cycle_seq`. The driver
    wakes only while the daemon is waiting. Fallback: nanosleep polling, chosen automatically after
    3 undelivered wakes. Measured p50 16 µs / p99 195 µs (os_sync) vs 82/128 µs (100 µs poll);
    table in docs/research/driver.md §4.1.
  - Record-only clients (no output IO): the daemon clocks from `input_time`.
  - Driver: ShmLink signals/wakes, publishes input_time; Open waits ≤100 ms for a concurrent
    creator; selftest gained an optional `--record-only` phase.
  - Rust `crates/wing-shm`: `layout` (repr(C) mirror plus const asserts), `Segment` (attach/create/
    replace/is_current/status), `DaemonLink` (wait / next_block / read_input / write_output /
    commit / heartbeat / stats; no alloc/locks/syscalls except the blocking wait), `wake`,
    `sim::DriverSim` (Rust port of ShmLink for tests), examples `gain_client` and `wake_latency`.
  - Files: driver/include/wing_shm.h, driver/src/ShmLink.{hpp,cpp},
    driver/tools/wing_shm_{client,selftest}.cpp, crates/wing-shm/** , Cargo.toml (libc workspace
    dep), docs/research/driver.md.
  - Test: `make check`, which includes `tests/cpp_selftest.rs` (compiles the C++ selftest with real
    ShmLink.cpp and runs it against the Rust daemon in another process: 0 mismatches, 0 underruns,
    epoch restart, record-only, SIGTERM detach), `tests/layout.rs` (compiles wing_shm.h as C11 and
    C++17 and compares all offsets and constants), and `tests/rt_alloc.rs` (0 allocations on the RT
    path). `driver/build.sh --test` also passes (2/2). Latency:
    `cargo run --release -p wing-shm --example wake_latency`.
  - Proposed follow-ups:
    - After install (human), verify os_sync wakes inside the real coreaudiod sandbox. Run
      `gain_client` against the installed driver: from_daemon underruns should stay flat and
      CPU ~1–2 %.
    - T022: run `DaemonLink` on a time-constraint thread and join the device os_workgroup.
    - T021: authenticate the daemon (the shm is still 0666).
