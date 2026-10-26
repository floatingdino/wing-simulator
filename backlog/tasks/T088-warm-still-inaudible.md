---
id: T088
title: BUG: Soul Warmth Pre distortion still inaudible live (125% drive, 100% mix, gate slot ch21)
stage: S4
status: review
owner: claude-T088
depends: [T085]
branch: task/T088
---

## Goal
User (latest build via menu app, 2026-10-26 10:15): on VOX channel 21 with WARM in the gate slot, drive 125 % and mix 100 %, the distortion is inaudible; the WARM trim knob audibly changes level. T081/T085 tests pass, so either (a) the live path differs from the tested path (wrong model actually active in the slot, block on/bypass state, the gate slot's wrapper mix still applied, scene values from WING-Edit's push e.g. `gate/on`, `wmix`, legacy `mix`), or (b) the saturation law is too gentle at real vocal levels (~-30..-18 dBFS RMS). Do both: add a debug introspection command (control socket + OSC) that dumps, for a strip, the active model per slot, its effective parameter values as the DSP sees them, and the measured in/out RMS and THD of the live block; use it against a reproduction loaded from the user's latest scene (scratch/live-mix-v0.6.snap or the autosave in ~/Library/Application Support/WingSimulator/state.snap) with realistic vocal-level programme. Then make WARM's character match the referenced SSL VHD-style "Soul Warmth" behaviour: drive adds clearly audible harmonics at nominal levels (document target THD vs drive at -18 dBFS, cite sources), keep level roughly compensated. Regression test at -24 dBFS RMS vocal-like input asserting audible THD (> ~1 %) at max drive.

## Acceptance criteria
- [x] Root cause identified with evidence (path vs law)
- [x] Audible saturation at realistic levels (test); introspection command available
- [ ] User confirms live

## Log
- 2026-10-26 coordinator: created from user retest.
- 2026-10-26 claude-T088: done, status review.
  - **Evidence first (path vs law)**: loaded the user's autosave (`~/Library/Application Support/WingSimulator/
    state.snap`, 10:20; copied into the worktree's git-ignored `scratch/`) into the daemon's own `Console`
    (`wing_render::build_console_with`), fed a vocal-like programme on USB 33-35 (ch 20-22 run their USB alt
    source) and captured each channel's gate slot with the new probe. **The live path is correct**: WARM
    (enum 12) selected and implemented, `on`, effective wet `w = 1.0` although block `mix` = 5 % (T085 fix
    live), `owns_mix`, the model's params exactly as the store holds them. The running menu-app binary was
    built 10:12 (after the T085 merge). Live log of the 10:15 session: the user tested **ch 20** (and ch 22),
    not 21: DRIVE 125 + MIX 100 with HARMONICS still at +100 (3rd-harmonic end), then hrm -100, trim, colour,
    then back to drive 100 / MIX 5. The autosave keeps ch 20/21 at `wmix 5` (inaudible by design: residual
    < 0.5 %) and ch 22 at 125/100/-100.
  - **Root cause = the saturation law** (T081 law: max shaper gain 6, small-signal normalised). At realistic
    vocal levels it was gentle, especially at the +100 end where the user listened first, and it *lost level*
    as it saturated. Sine THD at 125 % drive (1 kHz, H2..H10): hrm +100 2.2 % / hrm 0 5.8 % at -24 dBFS RMS;
    0.6 % / 3.0 % at -30; level -0.5..-2 dB. User scene, ch 21 vocal at -24 dBFS RMS, drive 125 / MIX 100 /
    hrm -100: probe residual 15.9 %, level -2.8 dB (at -30: 9.7 %, -1.8 dB). A soft, mostly 2nd/3rd-order
    change that also turns the channel down reads as "quieter", not "distorted".
  - **Fix** (crates/wing-blocks/src/models/dyn_special.rs): drive law `g = 0.5 * 32^((drv-33)/92)` (max 16, +30 dB
    over the default; the default is unchanged), plus `warm_makeup(g, b)`: RMS make-up so a -18 dBFS RMS sine
    keeps its level at every DRIVE/HARMONICS setting. Quiet passages come up by as much as +6 dB at 125 %, and
    peaks at -12 dBFS go down 5 dB, as with a driven preamp plus output trim. The make-up costs 64 `tanh`,
    cached per (drive, hrm) value.
  - **Targets** (documented with sources in the module docs). SSL publishes no VHD THD figures (Sound On Sound,
    XLogic VHD Pre review). Reviews describe it as neutral at moderate gain, with overt 2nd/3rd harmonics at
    high gain. Tan/Moore/Zacharov (JAES 2003) covers audibility. Targets: default ~1 % at -18 dBFS RMS;
    125 % > 20 % at -18, > 8 % at -24 and > 2 % at -30; THD monotonic in drive; level at -18 dBFS within 1 dB.
    Achieved at -24 dBFS RMS (hrm 0 / -100 / +100): 13.9 / 20.2 / 11.4 % (was 5.8 / 11.2 / 2.2). At -18:
    24.7 / 26.9 / 24.0 % with 0.0 dB level change. Vocal on the user's ch 21 at -24: residual 25.8 % at
    +0.5 dB (was 15.9 % at -2.8 dB).
  - **Introspection** (new): `wing_engine::probe`. `SlotProbeFeed`/`SlotProbe` make a one-shot RT capture of an
    input strip's gate/eq/dyn slot input and output (16384 samples). They also record the slot as the DSP sees
    it: model, on/mix/gain, effective wet, owns_mix, latency and the model's param values. RT cost while idle
    is one atomic load per slot per block, with no allocation (covered by `rt_path_does_not_allocate`).
    `analyse()` gives in/out RMS, peak, gain and the nonlinear residual: output minus the best 96-tap linear
    least-squares fit, after a 40 Hz high-pass. That is THD+N on a sine and a conservative measure on
    programme. Daemon (apps/wing-sim/src/probe.rs): control socket `probe ch/21/gate` (or `ch/21`,
    `aux/N/eq|dyn`) and OSC `/wingsim/probe ,s ch/21/gate`. Both reply with a one-line JSON array that shows
    `dsp` vs `store` param values. Usage on the running daemon, while the vocal plays: `echo 'probe ch/20/gate' | nc -U
    ~/Library/Application\ Support/WingSimulator/control.sock`.
  - **Tests**:
    - apps/wing-render/tests/warm_live.rs: the user's ch 21 strip through the whole console, -24 dBFS RMS
      vocal, drive 125 / MIX 100 at hrm 100/0/-100. It asserts the path, a residual > 25 % (before: 13-15 %)
      and |level| < 1 dB (before: -1.4..-2.8 dB). The saved MIX 5 and drive 10 stay < 1.5 %.
    - dyn_special: `warm_drive_targets_at_nominal_level`, `warm_thd_table`. Alias bounds re-measured in
      `warm_is_alias_free`: a 10 kHz tone at 125 % / -20 dBFS now aliases at -67 dBc and the extreme corner
      at ~-50 dBc; moderate drive stays < -110 dBc.
    - wing-engine probe unit tests; wing-sim `probe_command_reports_the_live_slot`; control-command parsing.
  - Files: crates/wing-blocks/src/models/dyn_special.rs (+ tests.rs), crates/wing-engine/src/probe.rs (new),
    channel.rs, lib.rs, mix/{mod,build}.rs, params/strip.rs (`wet_value`); apps/wing-sim/src/probe.rs (new),
    control.rs, main.rs, lib.rs, audio/console.rs (+ tests/audio.rs and the ConsoleConfig literals in tests);
    apps/wing-render/src/{render,perf,lib}.rs (`build_console_with`), tests/warm_live.rs;
    docs/VERIFY-ON-HARDWARE.md.
  - Test: `make check` pass, `make check-full` pass.
  - Gotcha: I smoke-tested the release daemon on private ports (32222/32223, private shm, temp state dir). It still
    bound discovery UDP 2222, because no flag moves it, and answered one WING-Edit discovery probe before I
    stopped it (~1 min). The user's daemon (pid 19443), its shm and its control socket were not touched. Test the
    probe in-process (tests above), not by spawning wing-sim.
  - Proposed follow-ups:
    - `--discovery-port` / `--no-discovery` so a second daemon can run side by side.
    - Probe support for bus/main/mtx strips (MixStrip).
    - User retests live (drive 125 at hrm +100 and -100, MIX 100) and can run `probe ch/20/gate` to report the
      real vocal level that reaches the slot.
