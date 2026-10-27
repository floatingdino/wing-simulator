# Record on a WING, mix on the simulator

This is the "virtual soundcheck at home" loop: multitrack a show (or rehearsal) from a real WING over
USB, save the console's scene, then replay the tracks into the simulator with that exact scene and
mix in WING-Edit as if you were at the console. Changes you make can go back to the real console.

It uses the WING's **ALT** input sources: every recorded channel keeps its normal (main) source for the
live show, and gets an ALT source on the USB channel it is recorded on. With the console's global ALT
switch off nothing changes for the show; switch it on (on the simulator, or on the real console for a
virtual soundcheck) and every channel plays back from the DAW instead.

REAPER is used below because the kit ships a ready REAPER project; any DAW works if you set it up the
same way (track N records DAW input N and plays to DAW output N, inputs/outputs mono, no master output).

## The kit

Installed with the release (macOS `~/Library/Application Support/WingSimulator/kit/integrations/recording`,
Linux `~/.local/share/wing-simulator/integrations/recording`, Windows
`%LOCALAPPDATA%\WingSimulator\integrations\recording`; in the repo `integrations/recording/`):

| file | what it is |
|---|---|
| `Recording-Scene.snap` | **Scoped** console scene holding only the recording patch: USB outputs 1-48 = the channels' sources, each channel's ALT source = its USB channel (ALT AUTOSW on), global ALT switch off, USB audio 48/48. Nothing else (no EQ, faders, names, FX), so loading it does not disturb the mix. |
| `Recording.RPP` | REAPER project: 48 mono tracks, track N records input N (= USB out N) and plays to output N (= USB in N), all record-armed, input monitoring off, no master send, 48 kHz, recordings in `Media/`. |

The shipped pair assumes the common stage-box layout: **channels 1-40 on AES50 A.1-40, aux 1-8 on the
local AUX inputs 1-8**. For your own patch, generate a matching pair from your show file (recommended):

```bash
wing-record-setup MyShow.snap --out ~/Desktop/rec-kit
```

It reads every channel's and aux's current main source and names from the show, gives each source a
USB channel (stereo sources take an odd/even pair, strips sharing a source share a channel), names the
REAPER tracks after the channels, prints the patch, and writes the same two files. Options:
`--channels 1-32`, `--aux none`, `--dry-run` (print the patch only). Strips whose source is OFF,
already USB, or a bus/main/matrix re-input are skipped and listed. Get `MyShow.snap` by saving the
show on the console's Library (to a USB stick) or by saving the scene from WING-Edit while it is
connected to the console.

## Before the show: load the recording scene on the WING

1. Copy `Recording-Scene.snap` to a USB stick, plug it into the WING, open the Library, select the
   file and **load** it. Because the file contains only the recording patch, only these settings
   change: USB outputs 1-48, the channels' ALT sources, ALT AUTOSW, the global ALT switch (off).
   - The console's own scope controls also work: when you save a snapshot with **Save + Scope**
     (select the channels, and in Contents only the input/source and USB output parts), a recall of it
     only touches what is in scope. If you prefer, build the recording scene that way on the console
     once and keep it in the library.
   - First time, check before the show: Routing > USB outputs show the channel sources; a channel's
     input page shows the ALT source USB N; ALT is off (channels still on their live sources).
2. The scene sets the console's USB audio configuration to 48/48; if the console was on another
   setting, check it took effect (the console may need a restart for a USB configuration change).

## Record

3. **Connect the computer to the WING's USB port** and choose the WING as the DAW's audio device:
   REAPER > Settings > Audio > Device: input and output device "WING" (CoreAudio on macOS, the
   WING's ASIO driver on Windows), 48 kHz, buffer 256 or more.
4. **Open `Recording.RPP`** (or your generated copy). Save it under a new name in its own folder first
   (File > Save project as) so the recordings land next to it in `Media/`.
5. All tracks are already armed with monitoring off. Check the track meters move with the band, then
   **record**. Stop when done.
6. **Save the REAPER project.**

## Save the show

7. On the WING, **save a full snapshot** of the show as it is now (Library > Save, normal scope: all).
   This "combined" snapshot has your mix (channels, EQ/dynamics, busses, FX, ...) **and** the
   recording patch (ALT sources on USB). Copy it to the computer (USB stick, or save it from WING-Edit
   while connected to the console).

## Mix on the simulator

8. **Start the simulator** (WingSimMenu > Start wing-sim, or `wing-sim`).
9. **Switch the DAW's audio device** to **Wing Simulator** (REAPER > Settings > Audio > Device), 48 kHz.
   The project's track outputs now feed the simulator's USB inputs 1-48.
10. **Open the combined snapshot in WING-Edit** while it is offline (not connected), so WING-Edit holds
    your show scene.
11. **Connect WING-Edit to the simulator:** select `WINGSIM`, SYNC DIRECTION = **PC -> Mixer**, CONNECT.
    The simulator now runs your show scene.
12. **Switch ALT on** so the channels take the USB playback: turn on the console's global ALT INPUT
    switch in WING-Edit, or send OSC to the simulator: `/io/altsw ,i 1` (UDP port 2223). Only channels
    with ALT AUTOSW on follow the switch (the recording scene turned it on for every recorded channel).
13. **Press play in REAPER.** The tracks play into the channels they were recorded from, through the
    show's processing; you hear the monitor on your Mac's default output (see README, host output).
    Mix in WING-Edit.

Taking it back: save the result (WingSimMenu > Save As, or `/wingsim/export ,s /path/mix.snap` for a
console-ready file with a change report, see [ab-compare-and-export.md](ab-compare-and-export.md)),
load it on the WING, and **switch ALT off** before the next show.

## Notes and limits

- **Sample rate:** keep everything at 48 kHz (console, DAW project, simulator).
- **Latency:** the DAW compensates its own recording latency; playback into the simulator adds about
  1.3 ms plus the driver's input delay, as on a console fed from USB.
- **Virtual soundcheck on the real console:** with the same project and the combined scene loaded on
  the WING, switching the console's ALT on plays the recording through the real desk. Switch it off
  before the band plays.
- **What the recording captures:** USB outputs carry each channel's *source* (pre-gain-trim,
  pre-processing; preamp gain applies, as on the console's source). Playback re-enters through the same
  channel, so the channel's trim, EQ and dynamics apply once, exactly like live.
- **More than 48 sources:** only 48 USB channels exist; `wing-record-setup` skips the rest (listed) —
  narrow it with `--channels` / `--aux`.
- **Scoped scene files** are written as snapshots that contain only the recording settings. Recalling
  such a file on a real WING is expected to change only those settings; this is
  [listed for hardware verification](../VERIFY-ON-HARDWARE.md). Check the first time (step 1) before
  relying on it in a show.
- Other DAWs: arm 48 mono tracks, track N input = device input N, output = device output N (no master
  output), record at 48 kHz. `wing-render --inputs <folder of stems> --scene combined.snap --set
  io/altsw=1` also renders a bounce offline from exported stems (`01 Kick.wav` feeds USB 1), on any
  platform.
