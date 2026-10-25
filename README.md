# Wing Simulator

A virtual Behringer WING for macOS. It shows up to your Mac as a 48-in / 48-out USB audio device
("Wing Simulator"), is controlled by the unmodified **WING-Edit 3.3.4** over the network exactly like
a real console, runs the audio through emulations of the WING's channel strips, busses, mains,
matrices and FX, and can play its monitor or any other output on a real audio interface. Typical use:
mix a REAPER session on the "console" with WING-Edit, or do virtual soundchecks, without a WING in
the room.

Not affiliated with Behringer/Music Tribe. WING-Edit is not included: download it from Behringer.
How faithful the emulation is, measured: [docs/fidelity-report.md](docs/fidelity-report.md).

## Requirements

- Apple silicon Mac, recent macOS (developed on macOS 26; the HAL driver and menu-bar app need a
  modern release). Intel Macs are untested.
- Administrator rights, once, to install the audio driver.
- WING-Edit **3.3.4** on the same Mac or on the same network (discovery uses UDP broadcast on port
  2222; the TCP control port is 2222, OSC is UDP 2223).
- REAPER (or any CoreAudio host), plus any audio interface if you want to hear the monitor.
- Building from source additionally needs Rust (rustup, the repo pins stable), Xcode command-line
  tools (Swift 6.2+) and `brew install cmake`.

## Install

From a release folder `WingSimulator-<version>/`:

```
./install.sh
```

It copies WingSimMenu.app to `~/Applications` and the command-line tools to `~/.local/bin` (no
privileges), then explains and asks before installing the audio driver into
`/Library/Audio/Plug-Ins/HAL` with sudo (this restarts macOS audio: quit REAPER first). `./install.sh --help`
lists options; `./uninstall.sh` reverses everything. The first launch of the app may need
right-click > Open because the build is ad-hoc signed, not notarised.

From source: `make dist` builds everything into `dist/WingSimulator-<version>/` (see
[docs/user/install-checklist.md](docs/user/install-checklist.md) for a step-by-step check of a fresh
machine).

## First run: REAPER + WING-Edit

1. **Start the simulator.** Open WingSimMenu (menu-bar icon), choose **Start wing-sim**. The menu shows
   "wing-sim running" and the audio status. (Command line alternative: `wing-sim`.)
2. **Point REAPER at the device.** REAPER > Settings > Audio > Device: audio system CoreAudio,
   input and output device **Wing Simulator**. Sample rate 48 kHz. REAPER now has 48 outputs: output
   N feeds console channel N ("USB In N"), and REAPER's 48 inputs are the console's "USB Out 1..48".
3. **Route tracks to the console.** Send each REAPER track to its own hardware output (track 1 to
   output 1, and so on), mono per output. Disable the master track's hardware output to 1/2,
   otherwise your whole mix lands on console channels 1 and 2.
4. **Connect WING-Edit.** Start WING-Edit; the simulator is listed by name (default `WINGSIM`) with
   firmware 3.3.4. Select it, set **SYNC DIRECTION = PC -> Mixer**, press **CONNECT**. WING-Edit pushes
   its whole scene (about 28,700 parameters in well under a second) into the simulator; meters start
   moving and every edit takes effect in the audio at once. Choose PC -> Mixer on purpose: it makes
   the simulator take your WING-Edit scene. (Mixer -> PC, pulling the simulator's state into
   WING-Edit, is implemented but not yet verified against WING-Edit.)
5. **Play something.** Press play in REAPER; the channel meters move and the main bus plays through
   the host output (below). Change an EQ or compressor in WING-Edit and hear it.

### Host output routing: hearing the console

The simulator plays chosen console outputs ("taps") on real audio devices through a drift-correcting
bridge. By default the monitor A bus (`mon/1`, which follows the console's monitor source, MAIN by
default, with solo priority) plays on "Scarlett 2i4 USB" 1/2 if that interface exists. To change it:
menu-bar app > **Host Output Routing...**, add routes (tap such as `main/1`, `bus/3`, `mon/1` or a
`usb_out/N` pair, to a device and channel pair), **Apply & Reload**. The routes live in
`~/Library/Application Support/WingSimulator/hostio.toml`; the format and examples:
`crates/wing-hostio/hostio.example.toml`, e.g. `map = 'main/1 -> "MacBook Pro Speakers" 1/2'`.
Do not route to "Wing Simulator" itself.

### Solo and monitor

Soloing a channel or bus in WING-Edit works as on the console: solo has priority on monitor A, so a
route from `mon/1` plays the soloed signal (PFL/AFL as set in the console's monitor settings) and
returns to the monitor source when you release solo. The simulated PHONES and SPEAKERS knobs
(monitor level of monitor A and B) default to 0 dB. Set them from WING-Edit's monitor section,
or over OSC `/wingsim/mon/1/lvl ,f -12` (also `2`), or the control socket (`monitor-level 1 -12`);
`status.json` reports `monitor_levels_db`. If the monitor is too hot or quiet, change that level,
not the interface gain.

### Scenes, A/B, diff, export

- **Scenes:** the console library page in WING-Edit (and OSC, REAPER scripts) store and recall scenes,
  kept in `~/Library/Application Support/WingSimulator/scenes/`. State autosaves two seconds after a
  change and is restored on the next start. The menu-bar app has **Open... / Save / Save As...** for
  `.snap` files (files from a real WING work, firmware 1.x ones are migrated).
- **Diff:** menu-bar app > **Copy Diff vs Scene File** copies a readable list of every parameter that
  differs from the scene file. `wing-diff a.snap b.snap [--format text|markdown|json]
  [--filter ch/1..40,bus] [--ignore-ui]` does the same for two files.
- **A/B and export:** two in-memory slots you can toggle while listening, and a console-ready
  `.snap` export with a change report. See [docs/user/ab-compare-and-export.md](docs/user/ab-compare-and-export.md).

### Offline render

`wing-render` runs the same engine faster than real time from files, for bounces and regression
checks, with no driver or daemon needed:

```
wing-render --scene mix.snap --rpp project.RPP --out bounce/ --taps main/1,bus/3 --tail 3
wing-render --scene mix.snap --inputs stems/ --out bounce/        # WAV files by leading track number
```

It writes 32-bit float WAVs per tap and `render_report.json` (peaks, loudness, speed). `--dry-run`
prints which file feeds which USB input; `--set path=value` tweaks parameters (for example
`--set io/altsw=1` for ALT INPUT virtual soundcheck). `wing-render --help` has all options.

### Recording

Capture console taps to disk while REAPER plays:

```
wing-sim record start main/1,bus/3,usb_out/1-48 ~/Recordings/take1
wing-sim record stop          # also: wing-sim record status
```

One 32-bit float WAV per stereo tap or channel group. The same commands exist over OSC
(`/wingsim/record/start ,ss <taps> <dir>`) and the menu-bar app's status line shows recording state.

### Scene recall from REAPER markers

Name a REAPER marker or region `WING:Verse` (or `WING:#0` for the first library scene) and the
simulator recalls that scene as playback crosses it. Setup (control-surface config or scripts, plus
fader automation) is in [docs/user/reaper-integration.md](docs/user/reaper-integration.md); the demo
project is `integrations/reaper/demo/WingSim-Demo.RPP`.

## Troubleshooting

| Symptom | Fix |
|---|---|
| WING-Edit does not list the simulator | The daemon must be running (menu shows "running"). Same network/subnet as WING-Edit; allow incoming connections for `wing-sim` in the macOS firewall prompt; no VPN capturing broadcasts. On one Mac it works over loopback; if several network interfaces exist, start with `--ip A.B.C.D` of the one WING-Edit uses. |
| WING-Edit refuses to connect or says the firmware is unsupported | The firmware gate: WING-Edit 3.3.4 only talks to consoles reporting a compatible firmware. The simulator advertises `3.3.4-0-g0c2b9d4a:release` by default; do not change `--fw` unless you run another WING-Edit version (use that version's firmware string). |
| "Lost connection! Auto reconnect is active." and it never reconnects | After the daemon restarts or closes the connection WING-Edit does not retry by itself: press OK in the dialog, select the console again, set SYNC DIRECTION = PC -> Mixer, press CONNECT. Idle connections are dropped after 10 s of silence (WING-Edit keeps them alive). |
| REAPER has no "Wing Simulator" device | The driver is not installed or coreaudiod was not restarted: run `./install.sh --driver-only`, or check Audio MIDI Setup. See the checklist. |
| Silence from REAPER, meters flat, but device is present | The daemon is not running or cannot attach to the driver. Start it from the menu-bar app. With no daemon the driver is a plain loopback (output N returns on input N), which is also the safe fallback if the daemon stalls or crashes: audio crossfades to loopback and back when the daemon recovers. |
| Everything is in channels 1 and 2 | REAPER's master track is sending to hardware outputs 1/2; disable that. |
| Crackles, dropouts on the host output | Raise the extra latency in Host Output Routing (default 5 ms; try 10-20 ms) and REAPER's buffer to 128 or 256. The simulator adds one engine block (64 frames, 1.3 ms at 48 kHz) plus the driver's input delay (128 frames by default, reported to REAPER for recording compensation). `wing-sim --block 128` raises the engine block. xruns show in the menu status and `status.json`. |
| Output device missing from the routing list | Press **Refresh Devices**; a route to an unplugged device is kept and reports "not connected" until the device returns. |
| Monitor too loud or quiet | See the monitor level knobs above; the default is 0 dB. |
| Daemon logs and state | Menu > **Log** and **State Folder** (`~/Library/Application Support/WingSimulator`). |

Known limits: sample rate is 48 kHz (44.1 kHz is accepted by the driver but not a tested path);
WING-Edit's MIXER -> PC direction and several simultaneous WING-Edit clients are unverified; some
behaviour assumed about real hardware is listed in [docs/VERIFY-ON-HARDWARE.md](docs/VERIFY-ON-HARDWARE.md).

## Documentation

- [docs/user/install-checklist.md](docs/user/install-checklist.md): fresh-machine install and smoke test
- [docs/user/reaper-integration.md](docs/user/reaper-integration.md): REAPER scripts, markers, automation, OSC reference
- [docs/user/ab-compare-and-export.md](docs/user/ab-compare-and-export.md): A/B slots, export to a console
- [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md), [docs/fidelity-report.md](docs/fidelity-report.md), `CLAUDE.md` (developer guide)

## Licence and signing

Signed ad hoc only (no Apple Developer ID, not notarised); notarisation is future work and needs a
paid developer account. Third-party: libASPL (MIT) for the driver.
