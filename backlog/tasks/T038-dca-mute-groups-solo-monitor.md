---
id: T038
title: DCA, mute groups, solo/monitor
stage: S3
status: review
owner: engineer-T038
depends: [T036]
branch: task/T038
---

## Goal
DCAs, mute groups, solo (PFL/AFL modes), monitor section output to host device.

## Acceptance criteria
- [x] Monitor section per scene cfg/mon + cfg/solo: monitor A/B source (e.g. MAIN.1), srclvl/srcmix, dim, mono, delay, limiter, monitor EQ; solo modes (LIVE/etc.), ch/bus/main/mtx taps (PFL/AFL), exclusive vs additive, solo-in-place where WING supports it
- [x] Solo priority: while any solo is active the monitor outputs carry the solo bus (mixed per srcmix), returning to the monitor source when cleared, click-free
- [x] Monitor A/B are selectable as host-bridge sources (T023 mapping, e.g. `mon/1 -> "Scarlett 2i4 USB" 1/2`) so the host output behaves like console monitors with solo priority; this is the recommended default host mapping
- [x] Solo state round-trips with WING-Edit (solo buttons, clear solo, solo indicators)
- [x] Behaviour tests

## Log
- 2026-10-25 coordinator: created.
- 2026-10-25 coordinator: user requested monitor-output-with-solo-priority workflow; acceptance expanded.
- 2026-10-25 engineer-T038: done, status review.
  - Files: `crates/wing-engine/src/mix/{logic.rs (new), monitor.rs (new), mod.rs, nodes.rs, build.rs}`,
    `src/routing/mod.rs` (doc), `tests/{monitor.rs, monitor_alloc.rs}` (new), `benches/monitor.rs` (new) +
    `Cargo.toml` bench block; app: `apps/wing-sim/src/audio/{mod.rs (ConsoleControl = routing + MixLogic,
    `prepare` hooks both, monitor lvl -oo -> 0 dB), console.rs (mon/1|2 taps = monitor node), driver.rs,
    hostio.rs (DEFAULT_ROUTE `mon/1 -> "Scarlett 2i4 USB" 1/2`)}`, `src/main.rs` (doc), `tests/audio.rs`.
  - Design: `MixLogic` (control thread, store hook + `refresh` polled by the audio control loop every 10 ms)
    resolves strip `tags` (#Dn/#Mn), `dca/*`, `mgrp/*`, `cfg/dcamgrp`, `cfg/solo/*`, `$ctl/cfg/soloexcl`,
    strip/DCA `$solo`, `mon`, `solosafe` into per-strip DCA gain (dB sum, -oo if any DCA -oo, +10 clamp),
    external mute (mgrp | DCA mute w/ dcamgrp | SIP) and solo flags (mon1/mon2, PFL/AFL) in `MixShared`
    (atomics + generation); every strip node polls the generation once per block (5 ms ramps). Writes back
    `*/$sololed` (1 own, 2 via DCA/srcsolo), `dca/*/$sololed`, `$stat/solo`; exclusive solo (last pressed wins);
    `$stat/solo`=0 from a client = clear solo. `MonitorNode` (after mtx; MON legs 1..4): per monitor source
    (cfg/mon/m/src POST) x srclvl / srcmix(when solo) + solo bus (PFL x -pfldim, AFL) -> 8-band EQ (SmoothedBiquad)
    -> lvl/dim($dim)/pan/wid/mono($mono)/inv -> delay (m/343) -> limiter (lim dBFS). All gains 5 ms ramps:
    click-free solo in/out. Semantics marked [I] in logic.rs/monitor.rs docs (A/B vs PH/SPK mapping, STUDIO = LIVE,
    SIP scope, pfldim meaning).
  - Tests: `cargo test -p wing-engine --test monitor --test monitor_alloc`, `cargo test -p wing-sim --test audio`
    (bridge on mon/1: main level, solo priority to +6 dB PFL, clear). DCA dB summing/clamp/-oo/bus members,
    DCA smoothing, mute groups + DCA mutes + dcamgrp, PFL vs AFL (ch + bus taps, pfldim), solo priority click
    test (max sample step < 3x steady), srcmix/srclvl, mon 2 routing, exclusive/additive/clear/DCA solo +
    indicators, SIP + solosafe, monitor lvl/dim/mono/pan/inv, EQ +6 dB @1k flat @100 Hz, limiter, delay 4800
    samples, zero RT heap ops with a control thread toggling solos/DCAs/mgrps/tags/monitor params. `make check`
    passes.
  - Bench (`cargo bench -p wing-engine --bench monitor`, PUBLIC serial, M-series): ~72-100 us/cycle with
    monitor EQ/delay/limiter, no solo vs 10 solos indistinguishable (noise); mix-only serial was ~63 us.
  - Proposed follow-ups: verify A/B vs PH/SPK, STUDIO vs LIVE, pfldim and SIP scope on hardware; `cfg/solo/mute`,
    `$flip`, `eqbdtrim`, `dirin`, source-solo input selection (`$srcsgrp/$srcsin`), `$presolo`, `$mute`/`$fdr`
    mirrors, `muteovr`; smooth monitor delay changes; mtx direct inputs MON.PH/MON.SPK can now be fed.
- 2026-10-25 engineer-T038: merged main (T039 metering, T04E/T04G/T04H) into task/T038; conflicts in
  `mix/mod.rs` (MixConfig `logic` + `meters`), `apps/wing-sim/src/audio/console.rs`, `tests/audio.rs` resolved
  keeping both. Metering: MonitorNode now has 8 outputs (4 monitor legs + solo bus 1/2 L/R);
  `PortMeters::for_mix` meters `MON` source legs 1..4 and two new solo cells (`meters::solo_cell`, `SOLO_BASE`,
  CELLS + 2); the daemon's monitor meter block (A9) = solo L/R, mon1 L/R, mon2 L/R from those cells.
  Test `monitor_and_solo_meters`. `make check` passes.
