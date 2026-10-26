# Verify the simulator against a real WING (no REAPER)

The simulator assumes things about the WING that WING-Edit cannot tell us
([docs/VERIFY-ON-HARDWARE.md](../../docs/VERIFY-ON-HARDWARE.md)). This kit measures them. For each scene you
recall a generated snapshot on the console, play one 50 s test signal through channel 1 over USB, record the
USB return, and `wing-verify` compares the recording with `wing-render` output for the same scene and signal.
Quick tier: 13 scenes, about 20 minutes. Full tier: 36 scenes.

## One-time

- WING on USB (USB card, 48 kHz session rate), `brew install sox` (or use any recorder, see step 4).
- `tools/verify-hardware/verify.sh gen kit` (add `all` for the full tier). Creates in `kit/`:
  `suite.wav` (pilot noise, 20 Hz to 20 kHz sweep, level staircase, level steps, noise bursts, 1 kHz and 100 Hz
  tones), `scenes/*.snap` (one per assumption: channel 1 from USB 1, straight to MAIN, USB out 1/2 = MAIN L/R,
  only the block under test switched on), `run-sheet.md`, `manifest.json`.

## Per run

1. Get the `.snap` files into the console's snapshot library: `kit/scenes/` onto a USB stick, then on the WING
   Library > Snapshots > import (or WING-Edit, File/Library > import). If the console refuses a file, note which
   (that tests the "header accepted by a real WING" assumption) and carry on with the ones it takes.
2. For each scene in `kit/run-sheet.md`: recall it with **all recall scopes on and global safes off** (the scene
   sets I/O routing and channel 1). Leave faders and the monitor alone; the level is read from the USB return.
3. Record: `tools/verify-hardware/capture.sh kit <scene> rec`. About 1 min per scene. Set
   `WING_DEVICE="<name in Audio MIDI Setup>"` if the console is not called `WING`.
4. Other recorder instead of sox: play `kit/suite.wav` into USB 1 and record USB out 1 (a stereo file is fine,
   channel 1 is used), 48 kHz, starting at least 1 s before playback and ending 12 s after it; save as
   `rec/<scene>.wav`. Play and record on the same device so the clocks agree.

## Compare

```
tools/verify-hardware/verify.sh compare kit rec verify-report --update-doc
```

Renders every scene that has a recording, aligns the recording with the render on the pilot burst (the
console's I/O latency shows up as `latency`, it is not an error), checks drift with the end pilot, and measures:

| Test | Measures | Tolerance |
|---|---|---|
| `fr` | 1/3-octave band levels over the sweep (EQ, filters, tone shaping) | 1 dB |
| `stairs` | settled output level for inputs -60 to -4 dBFS (static curve, thresholds, make-up) | 1 dB, shape 0.75 dB |
| `steps` | 10-90 % settling time after +30 / -30 dB steps (attack, release), settled level, 5 ms envelope | 6 dB on the time ratio (x2), 4 dB envelope |
| `bursts` | noise-burst levels and envelope (gates, detectors) | 1.5 dB, 4 dB envelope |
| `tone` | harmonics 2-5 and THD of 1 kHz and 100 Hz at -12 dBFS (saturation, colouration) | 6 dB |

All values are recording minus render in dB. Output: one line per scene on screen, `verify-report/report.md`
with every metric and where it was worst, and (with `--update-doc`) the per-scene table in the Results block of
`docs/VERIFY-ON-HARDWARE.md`. Read a `DEVIATES` as: the assumption named for that scene (same file) is wrong or
the parameter mapping differs. First look at `00-thru`: if it is off (level, latency far from the others,
drift), fix the chain (clip, wrong scene, 44.1 kHz) before trusting anything else.

## WARM calibration (~5 min, T090)

Fits the simulator's SOUL WARMTH PRE (drive curve, 2nd/3rd balance, make-up) to your console. You do not need
to patch anything: every scene routes USB in 1 -> ch 1 (WARM in the gate slot, MIX 100 %, trim 0) -> MAIN ->
USB out 1/2. **Recalling a scene replaces the console's state, so save your current show/scene first.**

1. Once: `tools/verify-hardware/verify.sh warm-gen warm-kit` (writes `warm-kit/`: `suite.wav` of 15 s, 8
   scenes, `run-sheet.md`). Import `warm-kit/scenes/*.snap` into the WING's snapshot library (USB stick, or
   WING-Edit > Library > import).
2. For each scene, in order `warm-thru`, `warm-d010`, `warm-d033`, `warm-d066`, `warm-d100`, `warm-d125`,
   `warm-d125-hm100`, `warm-d125-hp100`: recall it on the WING (all scopes, safes off), then run
   `tools/verify-hardware/capture.sh warm-kit <scene> warm-rec` (~20 s each; needs `brew install sox`, set
   `WING_DEVICE` if the console's CoreAudio name is not `WING`).
3. `tools/verify-hardware/verify.sh fit-warm warm-rec`: prints the fitted law and PASS/FAIL (every point's THD
   within +-2 dB), writes `warm-report/warm-fit.md` (recording vs fitted vs previous law, per scene and
   signal) and rewrites `data/calibration/warm.json`. Rebuild (`make build` or `make app`) so the simulator
   uses it; commit the JSON.

Signals per scene: 1 kHz sines at -36 / -24 / -18 / -12 / -6 dBFS RMS, 100 Hz at -18, SMPTE IMD (60 Hz + 7 kHz,
4:1) at -18, a vocal-like excerpt at -24. The fit uses the 1 kHz points (THD H2..H10, H2, H3 and the level
change vs `warm-thru`); 100 Hz, IMD and vocal residual are reported. `verify.sh warm-gen warm-kit all` adds
HARMONICS +-100 at 66 % and COLOR +-1 at 100 % (reported, colour is not fitted). Another recorder works as in
step 4 above: play `warm-kit/suite.wav` into USB 1, record USB out 1 at 48 kHz from 1 s before to 3 s after,
save as `warm-rec/<scene>.wav`.

## What the kit cannot check

Listed in the docs file (input balance and stereo-pair laws, snapshot/protocol/OSC rows, DS902/DEQ, 3CMP, time-
variant FX). The simulator's own internal fidelity against WING-Edit data is in
[docs/fidelity-report.md](../../docs/fidelity-report.md) (`make fidelity-report`).

## Developing the kit

`apps/wing-render/src/verify/`: `suite.rs` (signal layout), `scenes.rs` (catalog: add a scene = one line, parameter
paths are checked when `gen` runs), `analyze.rs`, `mod.rs` (gen, render, align, metrics, reports). Tests:
`cargo test -p wing-render --test verify_kit` (synthetic recordings; with `--release -- --include-ignored` every
scene is rendered and compared with a delayed copy of itself).
WARM calibration: `warm.rs` (suite, scenes, measurement, fit, report); the law it fits is `WarmLaw` in
`crates/wing-blocks/src/models/dyn_special.rs`, loaded from `data/calibration/warm.json` at build time. Tests:
`cargo test -p wing-render --test verify_warm` (a synthetic console with another law is fitted back).
