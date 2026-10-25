---
id: T027
title: Host bridge latency convergence + startup underruns
stage: S2
status: done
owner: agent
depends: [T023, T026]
branch: task/T027
---

## Goal
Live v0.2 run (docs/milestones/v0.2.md, log scratch/wing-sim-v0.2.log): with REAPER at 512-frame buffers the Scarlett bridge ring fill drifted 23 -> 30 ms (target ~5 ms extra) and the PI ratio correction swung ±500 ppm instead of settling near the true drift; 1 bridge underrun and 2 driver underruns at startup. Fix: fill estimation robust to large/bursty producer blocks (producer block 512 vs device block e.g. 128/256), target latency derived from max(producer block, consumer block) + margin, PI tuning (anti-windup, slower integrator, deadband), startup priming so no underruns. Also report end-to-end latency (REAPER out -> Scarlett) in the status line.

## Acceptance criteria
- [x] Simulated test with 512-frame producer / 128-frame consumer: fill settles within 1 ms of target, correction within ±20 ppm of true drift after 30 s
- [ ] No startup underruns in driver or bridge (test + live log) -- tests done; live log pending
- [ ] Live: fill stable for 10 min with REAPER at 512 and 128

## Log
- 2026-10-25 coordinator: created from v0.2 live run.
- 2026-10-25 agent: diagnosis (live log + code + new sim). (1) The fill estimate was a lower envelope of
  per-push stamps (fast down 0.1 s / slow up 10 s), so mid-burst stamps, DAW/daemon jitter and producer phase
  steps biased it asymmetrically. (2) The PI (0.015 Hz, Kp ~4 ppm per frame, anti-windup only at the 2000 ppm
  clamp) turned any fill step into a demand of ~2000 ppm, slew-limited at 200 ppm/s while the integrator kept
  winding: overshoot and a slowly decaying oscillation at the loop frequency (~67 s period). Live: fill sat at
  target for 110 s (drift < 0.1 ppm), then +10 ms at 170-180 s (one whole REAPER buffer landing at once: HAL
  cycle skip / late daemon) and +-500 ppm swings with a ~70 s period. (3) Startup: the daemon's first bursts after
  idle ran cold (RT policy set lazily on the first wake, workers asleep); the one bridge underrun coincides with a
  driver underrun (daemon late), which also re-primed with a fragile envelope estimate.
- Repro (tests/drift.rs rewritten as a scenario sim: driver cycles of 512 on host time, DAW work + 1.5 ms jitter,
  daemon 160 us/block pushing blocks, occasional 3.2 ms late wakes, device 128/256/512 at +-200 ppm with stamp
  jitter and late IO wakes). Old code: 512/128 converged after 35-41 s, 512/256 after 587-596 s (corr wander up to
  +-34 ppm); the +1-buffer step test drove the correction to +1664 ppm (drift 150). An initial harsher run
  (whole-cycle late wakes + recurring 3 ms DAW steps) reproduced +-450 ppm and 14-53 underruns.
- Fix: crates/wing-hostio: ring `stamp()`; producer `end_burst(now, burst)` (blocks pushed unstamped, one stamp per
  driver cycle, burst size reported); consumer fill = median over the last 5 burst stamps of the extrapolated
  producer position; target = burst + device buffer + margin (sum: in the worst phase they add), retargets and
  re-primes when the driver buffer changes; priming waits for 3 fresh bursts; a ring that runs dry because the
  producer stopped is not an underrun (one that stalls and resumes within 100 ms is). control.rs replaced: FLL
  drift estimate (box-averaged windows of 2 s, median of 5 rejects phase steps, 8-window long baseline when it
  agrees, acquisition then 15 s tracking) + P phase correction (tc 8 s, 3 s low-pass, 3-frame deadband, clamped to
  150 ppm); no integrator, so no windup. Stats: ring_frames, drift_ppm, burst, output latency, `latency_ms()`.
  coreaudio output records device latency + safety offset + buffer. Daemon: Console pushes unstamped and
  `end_burst` once per service() burst with the driver buffer; RT policy set before the first wait; console
  warm-up (32 cycles at session start, 8 on each new driver epoch before its first buffer); status line now
  `fill x/target ms e2e y ms corr ... (drift ...)` where e2e = daemon burst time + ring (fill - burst) +
  resampler + device output latency.
- After (sim, same scenarios): converged after 21-25 s in every live case (512/128, 512/256, 512/512, 128/128,
  +-200 ppm), smoothed fill within 13 frames, correction wander about +-10 ppm, 0 underruns from start; the step
  test drains +1 buffer / +2 ms at the 150 ppm phase limit, no overshoot, last 60 s within 5 frames / 4.7 ppm.
- Files: crates/wing-hostio/src/{bridge,control,ring,lib}.rs, src/coreaudio/output.rs, tests/drift.rs;
  apps/wing-sim/src/audio/{console,driver,hostio}.rs, tests/audio.rs. Test: `make check` (passes);
  `cargo test -p wing-hostio --test drift -- --nocapture` prints the sim numbers.
- Not done (needs the user): live acceptance (10 min at REAPER 512 and 128, startup without underruns). The
  running wing-sim was not touched. Restart it from this branch and watch `e2e`, `fill x/target`, `drift`.
- Proposed follow-ups: driver-side startup underrun accounting at epoch start (driver/C++ ShmLink) if the live
  log still shows them after the daemon warm-up; consider lowering the default 5 ms margin once live jitter is
  measured (bridge `ring_frames` minimum would show the real headroom).
