# Soak test (T071)

A long-run harness for the whole audio path: the real `AudioService` (control thread, RT thread,
engine workers, meters, RTA, host-output bridge) against the Rust simulated HAL driver, with the
timing and client load of a live night, monitored for xruns, cycle-time tails, memory growth, bridge
behaviour and allocations on the audio threads.

## Run it

```sh
# 60 minutes (the T071 reference run), low priority, private shm name
nice -n 10 cargo run --release -p wing-sim --example soak -- \
    --minutes 60 --shm-name /wingsim.soak.$$ --scene fixtures/PUBLIC.snap

# the full version: 8 hours
nice -n 10 cargo run --release -p wing-sim --example soak -- \
    --minutes 480 --shm-name /wingsim.soak.$$ --scene fixtures/PUBLIC.snap \
    --out target/soak/8h --quiet
```

Build once and run the binary (`target/release/examples/soak`) if cargo must not rebuild during a
long run. Exit status 0 = PASS, 1 = FAIL, 2 = bad arguments. Ctrl-C ends the run early and still
writes the summary. Everything goes to `--out` (default `target/soak/<unix time>/`):

| file | content |
|---|---|
| `soak.csv` | one row per `--interval-secs` (10 s): time, phase/burst size, settled flag, RSS, cycle p50/p99/p99.9/max over the interval, load %, slow cycles, driver xruns (total / settled / transition), bridge state, fill and target (ms), ratio correction and drift estimate (ppm) against the true simulated device error, bridge underruns/resyncs/overflow, harness lateness, RT allocation events, client counters |
| `summary.txt`, `summary.json` | the pass/fail summary printed at the end |
| `soak.log` | log of the run with a full audio status line every 5 minutes |

Options (defaults): `--minutes 1`, `--warmup-secs 30` (extra, excluded from the measurements),
`--scene fixtures/PUBLIC.snap`, `--interval-secs 10`, `--bursts 64,512` and `--phase-secs 300` (the
driver buffer size alternates every phase), `--settle-secs 15`, `--drift-ppm 200`,
`--drift-period-min 20`, `--device-buffer 256`, `--workers N`, `--no-rt` (no time-constraint policy:
for a machine that must not be disturbed at all, at the price of realism), `--no-storm` (no client
load), `--max-rss-growth-mb-h 4`, `--max-fill-dev-ms 2`, `--quiet` (one log line per interval off).

**Safety.** The harness refuses the production shm name. It never opens a CoreAudio device (the
bridge's device side is replaced by a simulated one through `AudioOptions::device_factory`), does not
bind ports 2222/2223 (servers use ephemeral loopback ports), does not start discovery and keeps its
control socket and `status.json` in `$TMPDIR/wingsoak.<pid>/`. The service's RT threads do run with
the time-constraint policy unless `--no-rt` is given; `nice` does not lower that.

## What is simulated

* **Driver.** `wing_shm::sim::DriverSim` on the private segment, driven by a thread that behaves like
  the HAL IO proc: one cycle per buffer on the host clock (absolute deadlines), WriteMix of 48-channel
  noise (about -19 dBFS RMS on all USB inputs), then ReadInput once the daemon has delivered the
  capture (up to 90 % of the period, else the read goes ahead and the driver counts an underrun). The
  buffer size alternates 64 / 512 frames every `--phase-secs`. A phase change is a DAW stop/start: IO
  stops for 500 ms, a new epoch begins, the first cycle comes one buffer later (the daemon warms up
  meanwhile).
* **Host bridge.** The default route shape (`mon/1 -> device 1/2`) with a simulated device pulling
  256-frame callbacks on a clock that wanders sinusoidally by +-200 ppm against the host clock (period
  20 min, so a 60 min run sees both extremes three times). The device thread has the time-constraint
  policy like a HAL IO thread; its lateness is recorded (`dev_late_cb`).
* **Servers.** In-process native TCP and OSC UDP servers on ephemeral loopback ports, the meter source
  (as the streamer's tick, without UDP), the RTA analyser, the control socket and `status.json`.
* **Clients** (one thread, deterministic seed): a trickle of about 50 random parameter sets per second
  per protocol (OSC text values and native tokens, over the ch/aux/bus/main/mtx/fx trees); a 3 s
  **storm** of about 3000 sets per second per protocol every 45 s; a **model swap** (random EQ / dynamics
  / gate / FX model selector over OSC) every 10 s; an **A/B toggle** (`Snapshot::apply_diff`, the
  `ab.rs` path) every 30 s between the scene and a copy with 400 parameters and 30 models changed; a
  full **scene recall** (`Snapshot::load`) every 5 min; a `status` poll over the control socket every 5 s.
  A second native connection only observes (receives the change fan-out).
* **Allocation counting.** A global allocator counts allocations, frees and reallocations on threads
  named `wing-audio-rt` and `wing-worker-*` after the warm-up, and records the call stacks of the first
  eight offenders (resolved with `backtrace_symbols` in the summary). A self-test at start checks that
  the counter sees an audio-named thread and ignores others.

## Pass / fail

FAIL if any of: driver xruns (underruns + overruns) in settled periods (outside the `--settle-secs`
after each IO restart); a loopback fallback or detached status while running; the daemon heartbeat
still fresh after shutdown; any allocation event on an audio thread; cycle p99.9 over 50 % of the period (the T070
budget); host-bridge underruns or resyncs
in settled periods; bridge fill more than `--max-fill-dev-ms` from target in settled periods; RSS
growth above `--max-rss-growth-mb-h` (least-squares slope after the first 2 minutes, evaluated when at
least 20 minutes remain). A cycle longer than one engine period is a warning (`WARN`) only (it is an xrun
only if it makes the driver underrun). Informational: drift-tracking error and the transition counts.

## Results

### 60-minute reference run (2026-10-26, M4 Max, `nice -n 10`, the machine in live use)

Raw data: `docs/soak/run1-60min.csv` (362 rows), `docs/soak/run1-60min-summary.txt`. PUBLIC.snap, 7 workers, RT policy
on, 64/512 alternating every 5 min (11 IO restarts), +-200 ppm device drift, client load as above.

| metric | result |
|---|---|
| engine cycles measured | 2,695,508 (60.0 min) |
| cycle time p50 / p99 / p99.9 | 80 / 190 / 250 us (period 1333 us); worst single cycle 1163 us |
| mean load / worst 10 s interval | 6.5 % / 12.0 %; cycles over one period: 0 |
| driver xruns (underruns + overruns) | 0 in settled periods; 1 in the 15 s after the IO restarts (11 restarts) |
| daemon dropped to loopback / detached | never; clean detach at shutdown |
| allocations on RT threads | 0 (alloc, free and realloc; 2.7 M cycles, 120 A/B toggles, 12 recalls, 362 model swaps, 80 storms) |
| RSS | 241.1 MB at start, 248.6 MB at the end; trend +1.94 MB/h (limit 4) |
| host bridge | 0 underruns / 0 resyncs when settled; 6 underruns in the transition windows (IO restarts); fill within 1 ms of target; drift estimate within 39 ppm of the true device error while it swept +-200 ppm |
| client load | 340 k OSC sets, 339 k native sets, 0 errors; 725 control-socket polls |

Findings:

* The IO restart (stop 500 ms, new epoch) costs about one bridge underrun and occasionally one driver
  underrun at the first cycle; that is outside the settled-period criterion and reported separately.
* Cycle times at a 512-frame burst are the same 64-frame cycles; no driver read ever went ahead of the daemon.
* The first full run reported FAIL on "bridge fill": one 10 s sample (t = 300 s) fell in the 500 ms stop gap
  between phases before the harness had switched phase, so the bridge was `Priming` with a stopped
  producer (3.97 ms below target) yet counted as settled. That was a harness classification bug, not a
  service fault (every other settled sample is within 1 ms). It is fixed (samples spanning an IO restart
  are transition samples); the numbers above are from that first run, whose only other deviation is that
  sample. A rerun with the fixed harness was started afterwards; its output is in
  `target/soak`-style files named in the T071 task log.

### Full 8 h run

Not run here. Use the command above (`--minutes 480`); at 10 s sampling the CSV is ~2900 rows. Compare its
RSS trend (`rss_mb` column) and `rt_alloc_events`; the RSS limit is only evaluated for runs of 20+ minutes.
