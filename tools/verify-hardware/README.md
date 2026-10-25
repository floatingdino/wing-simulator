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

## What the kit cannot check

Listed in the docs file (input balance and stereo-pair laws, snapshot/protocol/OSC rows, DS902/DEQ, 3CMP, time-
variant FX). The simulator's own internal fidelity against WING-Edit data is in
[docs/fidelity-report.md](../../docs/fidelity-report.md) (`make fidelity-report`).

## Developing the kit

`apps/wing-render/src/verify/`: `suite.rs` (signal layout), `scenes.rs` (catalog: add a scene = one line, parameter
paths are checked when `gen` runs), `analyze.rs`, `mod.rs` (gen, render, align, metrics, reports). Tests:
`cargo test -p wing-render --test verify_kit` (synthetic recordings; with `--release -- --include-ignored` every
scene is rendered and compared with a delayed copy of itself).
