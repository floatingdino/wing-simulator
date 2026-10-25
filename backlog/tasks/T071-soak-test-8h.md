---
id: T071
title: Soak test 8h
stage: S7
status: done
owner: eng-agent-T071
depends: [T03B]
branch: task/T071
---

## Goal
Long run with xrun/drift/memory monitoring.

## Acceptance criteria
- [x] Report (docs/soak.md)

## Log
- 2026-10-25 coordinator: created.
- 2026-10-26 eng-agent-T071: added `apps/wing-sim/examples/soak.rs` (harness), `AudioOptions::device_factory` /
  `audio::hostio::DeviceFactory` (seam replacing the CoreAudio side of host bridges; default None, no behaviour change;
  two test struct literals updated), docs/soak.md, docs/soak/run1-60min*. How to test: `cargo run --release -p wing-sim
  --example soak -- --minutes 60 --shm-name /wingsim.soak.$$` (see docs/soak.md). `make check` passes. 60 min run: 0 settled
  xruns, 0 RT allocations, p99.9 250 us, RSS +1.9 MB/h; one harness classification bug found and fixed (see docs).
  A rerun of the 60 min with the fixed harness was in flight at hand-off: scratchpad run60/summary.txt (not committed).
  Proposed follow-ups: IO restart gives ~1 bridge underrun (consider keeping the bridge primed across epochs); run the 8 h version.
- 2026-10-26 coordinator: first 60-min run: p99.9 250 µs, 0 settled xruns, 0 RT allocs, RSS +1.9 MB/h; rerun summary in scratchpad/run60/summary.txt.
