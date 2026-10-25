# Performance (T070)

Realtime budget: 48 kHz, 64-sample blocks = **1333 µs per cycle**. Target: the "everything on"
scene under 50 % of a cycle (667 µs) with 7–10 workers (ARCHITECTURE.md's aspiration: 30 %).

**Result: met with a wide margin.** "Everything on" at 64 samples, real scheduler, RT policy, paced
like the driver: **216–258 µs mean (16–19 %), p99.9 ≤ 24 %** at 7–10 workers (before T070: 306–360 µs,
p99.9 32–35 %). Outputs are bit-identical to before (every buffer of both scenes, every model).

## Tools

| what | how |
|---|---|
| report (this page) | `wing-render --bench [--scene public,everything] [--block 64,128] [--workers 0-12] [--cycles N] [--paced N --paced-workers 7,8,10] [--profile] [--models [--model ID,..]] [--digest N] [--no-lpt] [--set path=value]...` |
| criterion | `cargo bench -p wing-render --bench perf` (one console cycle per scene/block/workers, then a paced p99/p99.9 summary) |
| bit-identity | `--digest N`: FNV hash of every output channel of every graph node over N cycles (0 vs 7 workers); `--models` prints a per-model output digest too. `apps/wing-render/tests/perf_scenes.rs` checks serial == parallel (with and without LPT) |
| profiling | `CARGO_PROFILE_RELEASE_DEBUG=line-tables-only cargo build --release -p wing-render`, run `--bench ... --cycles 20000`, `sample <pid> 8` (line-level call tree). `--profile` prints the per-node serial cost, per-level sums and the level-schedule lower bound |

Code: `apps/wing-render/src/perf.rs` (scenes, timing, digests, survey, CLI).

### Scenes

* **public**: `fixtures/PUBLIC.snap` as is (82 graph nodes, 16 FX in use).
* **everything**: factory defaults plus [`everything_on`](../apps/wing-render/src/perf.rs): 40 ch + 8 aux on USB
  inputs; ch: filter (LC/HC/tilt `AP2`), gate `NSTR`, PEQ, EQ `MACH4`, dyn `76LA`; aux: EQ `PULSAR`, dyn `76LA`;
  all sends (16 bus, 4 main, 8 mtx) on; 16 bus / 4 main / 8 mtx: EQ `PIA`, dyn `76LA`, delay on, all
  bus→main/mtx and main→mtx sends on; FX 1..16 loaded and inserted on ch 1..16:
  `*MASTER*` (TAPE+MACH4+ENHANCE+LIMITER all on, the heaviest FX), `ANGEL, EXCITER, UKROCK, SPRING, DELUXE,
  RACKAMP, VSS3`, `*MASTER*`, `ENHANCE, ANGEL, EXCITER, UKROCK, DELUXE, RACKAMP, JAZZC` (lite slots 9–16 only
  take lite models). Models chosen from the model survey below.
* Input: deterministic noise ≈ −18 dBFS RMS on all 48 USB inputs, meters on (as the daemon).

### Regimes

* **paced RT** (the production regime): one cycle per period, time-constraint policy on the driver and the
  workers, workers sleep between cycles (wake-up latency included).
* **hot**: cycles back to back, no RT policy. Used for worker scaling; its tails are not meaningful (non-RT
  threads get preempted, e.g. the 10 ms outliers) and the kernel throttles RT threads that run flat out.

Machine: M4 Max (12 P + 4 E cores), macOS. **Measured under background load** (the user's running wing-sim
at ~50–80 % CPU, REAPER, WING-Edit, other agents compiling), so single runs vary by ±10 %; model costs are
A/B-interleaved (min of 3 rounds) between the baseline build (`b3e2264`, tooling only) and the final build.

## Results

### Paced RT, 64 samples (period 1333 µs) — µs, % of period

| scene | workers | before mean / p99 / p99.9 | after mean / p99 / p99.9 | after % (mean / p99.9) |
|---|---|---|---|---|
| everything | 4 | 559 / 1030 / 1228 | 376 / 409 / 425 | 28.2 / 31.9 |
| everything | 6 | 433 / 562 / 1113 | 299 / 344 / 368 | 22.4 / 27.6 |
| everything | 7 | 360 / 406 / 430 | **258 / 303 / 319** | **19.3 / 23.9** |
| everything | 8 | 339 / 398 / 423 | **239 / 293 / 309** | **17.9 / 23.2** |
| everything | 10 | 306 / 371 / 461 | **216 / 278 / 321** | **16.2 / 24.1** |
| public | 4 | 135 / 162 / 180 | 119 / 156 / 199 | 8.9 / 14.9 |
| public | 7 | 140 / 288 / 521 | 119 / 155 / 244 | 8.9 / 18.3 |
| public | 10 | 141 / 204 / 341 | 120 / 159 / 298 | 9.0 / 22.3 |

Everything at fewer workers (after): 1 worker 849 µs (64 %, p99.9 71 %), 2: 598 µs (45 %, p99.9 50 %),
3: 446 µs (33 %), 5: 329 µs (25 %), 9: 228 µs (17 %), 11: 234 µs (p99 28 %).

### Paced RT, 128 samples (period 2667 µs)

| scene | workers | before mean / p99.9 | after mean / p99.9 |
|---|---|---|---|
| everything | 7 | 771 / 1826 | 530 / 1101 (19.9 % / 41.3 %) |
| everything | 8 | 686 / 1344 | 499 / 1448 |
| everything | 10 | 685 / 1686 | 443 / 1198 (16.6 % / 44.9 %) |
| public | 7 | 284 / 1189 | 236 / 1053 |

At 128 the tails are dominated by wake-ups after the longer sleep and by the background load (p99 varies
2× between runs); the means are stable.

### 32 samples (period 667 µs), after

everything: 7 workers 155 µs mean (23 %), p99.9 466 µs (70 %); public: 71 µs (11 %), p99.9 12 %.

### Serial work and the level-schedule bound (64 samples)

`--profile`: per-node serial cost; bound = Σ over levels of max(longest node, level sum / threads).

| scene | serial work before → after | level 2 (48 input strips) | bound 8 thr | bound 11 thr |
|---|---|---|---|---|
| everything | 2606 → **1664 µs** (−36 %) | 2038 → 1313 µs | 343 → 220 µs | 263 → 169 µs |
| public | 296 → 246 µs (−17 %) | 176 → 136 µs | 100 → 87 µs | 100 → 87 µs |

Paced at 7 / 10 workers lands 17 % / 28 % above the bound (barrier hand-offs, wake-ups, the 16 busses
on 11 threads needing two waves). PUBLIC's critical path is `main4` (35 µs: its `*MASTER*` insert with
ENHANCE + TAPE on, 28 µs) at level 6, so PUBLIC does not scale past ~4 workers.

### Hot worker scaling, everything, 64 (mean µs; after)

| workers | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10 | 11 | 12 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| before | 2604 | 1335 | 932 | 704 | 723 | 521 | 472 | 418 | 369 | 360 | 383 | 421 | 446 |
| after | 1679 | 852 | 604 | 455 | 390 | 336 | 304 | 265 | 242 | 247 | 297 | 316 | 325 |

Beyond 9 workers the non-RT hot runs spill onto the E-cores / compete with the background load; the paced
RT runs keep improving up to 10 workers.

### Model survey (ns per stereo 64-sample block, default parameters; heaviest)

| model | before | after | | model | before | after |
|---|---|---|---|---|---|---|
| dyn NSTR | 20438 | **11684** (57 %) | | fx DELUXE | 12815 | 11327 (88 %) |
| dyn 76LA | 19903 | **10076** (51 %) | | fx RACKAMP | 12089 | 10643 (88 %) |
| fx ENHANCE | 18189 | 16727 (92 %) | | fx JAZZC | 10078 | 8415 (84 %) |
| fx ANGEL | 17548 | 16026 (91 %) | | fx VSS3 | 9577 | 9601 |
| fx EXCITER | 16025 | 14282 (89 %) | | fx TAPE | 8204 | 6943 (85 %) |
| fx UKROCK | 15057 | 13546 (90 %) | | dyn WARM | 4663 | 3790 (81 %) |
| fx SPRING | 13639 | 13622 | | fx HALL | 5681 | 5753 |

Every other model is ≤ 7.2 µs (reverbs 4–7 µs, everything else ≤ 3 µs; EQs ≤ 0.8 µs). Full list:
`wing-render --bench --models --scene none`.

## What changed (all bit-exact)

1. **Fixed-order dot products as explicit NEON** (`wing_dsp::simd`): the FIR dots were written with 8 / 4
   lane accumulators "to vectorise", but LLVM kept them as 8 scalar chains. `dot_acc8` / `dot_acc4` perform
   the same IEEE operations lane by lane (mul then add, never fused; same reduction tree), and
   `dot_acc8_each` keeps a fixed tap set in registers across a batch of windows. Scalar definitions stay as
   the reference and the non-aarch64 fallback; tests compare bits.
2. **Half-band oversampler stages batched** (`oversample.rs`): a batch of input is written once,
   newest-first, in front of the history; every output's window is a slice (no per-sample mirrored ring
   push followed by an overlapping load); the two stage designs (K = 20, 7) get their tap count at compile
   time. Benefits every oversampled model (76LA/NSTR, amps, enhancers, TAPE, WARM, ...). A reference copy of
   the old per-sample stage is kept in the tests.
3. **76LA / NSTR control path** (`comp_fet.rs`): the 768 kHz detector loop is split into a vectorised,
   branch-free static-curve pass (`lin_to_db_fast4` + knee selects; the ALL dip falls back to scalar), a
   ballistics pass with the state in registers (it was reloaded through `&mut self` every control sample)
   and a batched GR decimation FIR. ~2× faster per instance (the everything scene runs 76 × 76LA and 40 × NSTR).
4. **`TapLine` push/read** (slot latency alignment) as slice copies instead of per-sample masked indexing.
5. **LPT claim order** in the level scheduler (`graph.rs`, `sched.rs`): every node run is timed into a
   smoothed cost; every 64 cycles the driver re-sorts each level longest-first before publishing the epoch.
   PUBLIC −7 % mean (its heavy channels 26/27 were claimed last); neutral on "everything" (already
   heavy-first). `Engine::set_load_balancing(false)` / `--no-lpt` restores schedule order.

## Max sustained configuration at 64 samples

"Everything on" is the console's maximum load (every strip with its heaviest models, every FX slot
owned and loaded with the heaviest models). It is sustained at 64 samples with **p99.9 < 25 % of the
period at 4–10 workers** (4: 32 %), and still under 50 % p99.9 with **2 workers** (49.9 %). At 32-sample
blocks it holds at 7 workers (mean 23 %, p99.9 70 %). Recommended: 7–9 RT workers (10 gives the lowest
mean, 216 µs; 11 starts to lose to E-core spill / contention).

## Follow-ups (not done: they change DSP output or need a design change)

* **ENHANCE (Ultra Enhancer)**: 16.7 µs/block, dominated by per-sample libm `log1p` / `exp`; inside
  `*MASTER*` it puts 28 µs on PUBLIC's critical path. Fast approximations would change output (needs a
  fidelity re-check).
* **Amp sims** (ANGEL, UKROCK, DELUXE, RACKAMP, JAZZC: 8–16 µs): f64 per-sample 4×-oversampled stage
  chains; f32 and stereo-SIMD would change output.
* **76LA / NSTR**: now ~10–12 µs, floor ≈ 4 µs of sequential f64 ballistics at 16× (768 kHz). Running the
  ballistics at 2×/4× with an interpolated GR would halve it again (output change).
* **Reverbs** (HALL family 4–7 µs, more with large sizes): ER tap loop with fractional reads and scattered
  Lagrange reads of the FDN frame buffer.
* **Scheduler**: a dependency-driven (DAG) scheduler instead of level barriers, and splitting a strip node
  at a heavy insert, would shorten critical paths like PUBLIC's `main4` (35 µs alone at level 6).
* **Default worker count**: `EngineConfig::default()` caps workers at 7; on 12 P-core machines 8–10 measured
  better for heavy scenes.
