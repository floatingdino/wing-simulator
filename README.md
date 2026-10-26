# Wing Simulator

A virtual Behringer WING. On a Mac it shows up as a 48-in / 48-out USB audio device ("Wing
Simulator"), is controlled by the unmodified **WING-Edit 3.3.4** over the network exactly like a real
console, runs the audio through emulations of the WING's channel strips, busses, mains, matrices and
FX, and plays its monitor (or any other output) on your speakers or audio interface.

Typical uses:

- **Virtual soundcheck / mix practice without the console:** record a multitrack on a real WING, then
  mix it at home on the simulator with WING-Edit and the exact show scene, and take the result back to
  the console. A step-by-step walkthrough and ready-made helper files:
  [docs/user/recording-workflow.md](docs/user/recording-workflow.md).
- **Show prep:** build and check scenes, compare them (`wing-diff`), export console-ready `.snap` files.
- **Offline renders:** run a scene over a set of stems faster than real time (`wing-render`).

It works with any DAW that can play to and record from an audio device. REAPER is the best supported
(project templates, scene-recall markers, scripts), but nothing requires it.

Not affiliated with Behringer/Music Tribe. WING-Edit is not included: download it from Behringer.
How faithful the emulation is, measured: [docs/fidelity-report.md](docs/fidelity-report.md).

## Platforms

| | macOS (Apple silicon) | Windows, Linux |
|---|---|---|
| Virtual 48x48 USB audio device (DAW <-> console) | yes | no |
| WING-Edit / OSC control, scene library, A/B, export | yes | yes (control-only: no live audio) |
| Offline render of a scene over stems (`wing-render`) | yes | yes |
| Scene diff (`wing-diff`), recording kit (`wing-record-setup`) | yes | yes |
| Menu-bar app, playback on speakers/interface | yes | no |

The live audio path depends on a CoreAudio driver, so it is macOS only. On Windows and Linux,
`wing-sim` still answers WING-Edit (connect, edit, store scenes) and the file tools work fully.
Intel Macs are untested. Sample rate: 48 kHz.

## Install

**macOS** (Apple silicon, recent macOS) and **Linux**, in Terminal:

```bash
curl -fsSL https://github.com/floatingdino/wing-simulator/releases/latest/download/get.sh | bash
```

**Windows**, in PowerShell:

```powershell
irm https://github.com/floatingdino/wing-simulator/releases/latest/download/get.ps1 | iex
```

The script downloads the release, checks it against the release's `SHA256SUMS.txt` and installs:

- macOS: `WingSimMenu.app` to `~/Applications`, the command-line tools (`wing-sim`, `wing-render`,
  `wing-diff`, `wing-record-setup`) to `~/.local/bin`, the recording kit, DAW scripts and docs to
  `~/Library/Application Support/WingSimulator/kit`. It then explains and **asks** before installing
  the audio driver into `/Library/Audio/Plug-Ins/HAL` with `sudo` (this restarts macOS audio for a
  second: quit your DAW first). No Apple developer account or code-signing setup is needed: the
  builds are ad-hoc signed and the installer clears the download quarantine on its own copies.
- Linux: the tools to `~/.local/bin`, kit and docs to `~/.local/share/wing-simulator`.
- Windows: everything to `%LOCALAPPDATA%\WingSimulator` (its `bin` is added to your user PATH).

**Pin a version** (a frozen release): every release carries installers pinned to itself, so use that
release's URL, or set `WINGSIM_VERSION`:

```bash
curl -fsSL https://github.com/floatingdino/wing-simulator/releases/download/v0.3.0/get.sh | bash
```

Installer options go after `bash -s --` (macOS), for example `... | bash -s -- --no-driver`; see
`install.sh --help`. **Uninstall** (macOS): `uninstall.sh` from the release zip (removes the app, tools
and, after asking, the driver; scenes are kept unless `--purge-state`).

Prefer to do it by hand? Download the zip for your platform from the
[releases page](https://github.com/floatingdino/wing-simulator/releases), unzip, and run
`./install.sh` (macOS) or copy `bin/` somewhere on your PATH. A fresh-machine checklist:
[docs/user/install-checklist.md](docs/user/install-checklist.md).

You also need WING-Edit **3.3.4** (on the same computer or the same network) and, on macOS, a DAW.

## First run (macOS): a DAW + WING-Edit

1. **Start the simulator.** Open WingSimMenu (menu-bar icon), choose **Start wing-sim**. The menu shows
   "wing-sim running" and the audio status. (Command line alternative: `wing-sim`.)
2. **Point your DAW at the device.** In the DAW's audio settings choose **Wing Simulator** as input
   and output device, 48 kHz. (REAPER: Settings > Audio > Device, audio system CoreAudio.) The DAW
   now has 48 outputs and 48 inputs: output N is the console's **USB input N**, input N is the
   console's **USB output N**.
3. **Route tracks to the console.** Send each track to its own hardware output (track 1 to output 1,
   and so on), mono per output, and turn off the master/stereo bus output to 1/2, otherwise your whole
   mix also lands on USB inputs 1 and 2. How the USB inputs reach channels is the scene's input patch
   (channel main or ALT source = USB N), exactly as on the console.
4. **Connect WING-Edit.** Start WING-Edit; the simulator is listed by name (default `WINGSIM`) with
   firmware 3.3.4. Select it, set **SYNC DIRECTION = PC -> Mixer**, press **CONNECT**. WING-Edit pushes
   its whole scene (about 28,700 parameters in well under a second) into the simulator; meters start
   moving and every edit takes effect in the audio at once. Choose PC -> Mixer on purpose: it makes
   the simulator take your WING-Edit scene. (Mixer -> PC is implemented but not yet verified against
   WING-Edit.)
5. **Play something.** Press play; the channel meters move and the monitor plays on your Mac's default
   output (below). Change an EQ or compressor in WING-Edit and hear it.

For the full record-on-a-WING, mix-on-the-simulator loop, follow
[docs/user/recording-workflow.md](docs/user/recording-workflow.md).

### Host output: hearing the console

The simulator plays chosen console outputs ("taps") on real audio devices through a drift-correcting
bridge. By default monitor A (`mon/1`: the console's monitor source, MAIN by default, with solo
priority) plays on outputs 1/2 of the Mac's **default output device** (whatever is selected in System
Settings > Sound when the simulator starts; never the simulator's own device). To change it:
menu-bar app > **Host Output Routing...**, add routes (a tap such as `main/1`, `bus/3`, `mon/1` or
`usb_out/N`, to a device and channels; "System default output" is in the device list), **Apply &
Reload**. The routes live in `~/Library/Application Support/WingSimulator/hostio.toml`; format and
examples: [crates/wing-hostio/hostio.example.toml](crates/wing-hostio/hostio.example.toml), e.g.
`map = 'main/1 -> "MacBook Pro Speakers" 1/2'` or `map = 'mon/1 -> "My Interface" 3/4'`.

### Solo and monitor

Soloing a channel or bus in WING-Edit works as on the console: solo has priority on monitor A, so a
route from `mon/1` plays the soloed signal (PFL/AFL as set in the console's monitor settings) and
returns to the monitor source when you release solo. The simulated PHONES and SPEAKERS knobs
(monitor level of monitor A and B) default to 0 dB. Set them from WING-Edit's monitor section,
over OSC `/wingsim/mon/1/lvl ,f -12` (also `2`), or the control socket (`monitor-level 1 -12`).
If the monitor is too hot or quiet, change that level, not the interface gain.

### Scenes, A/B, diff, export

- **Scenes:** the console library page in WING-Edit (and OSC, DAW scripts) store and recall scenes,
  kept in the state folder's `scenes/` (macOS `~/Library/Application Support/WingSimulator`, Linux
  `~/.local/share/wing-simulator`, Windows `%APPDATA%\WingSimulator`; `--state-dir` overrides). State
  autosaves two seconds after a change and is restored on the next start. The menu-bar app has
  **Open... / Save / Save As...** for `.snap` files (files from a real WING work; firmware 1.x ones
  are migrated). `wing-sim --scene show.snap` loads one at start.
- **Diff:** menu-bar app > **Copy Diff vs Scene File** copies a readable list of every parameter that
  differs from the scene file. `wing-diff a.snap b.snap [--format text|markdown|json]
  [--filter ch/1..40,bus] [--ignore-ui]` does the same for two files.
- **A/B and export:** two in-memory slots you can toggle while listening, and a console-ready
  `.snap` export with a change report. See [docs/user/ab-compare-and-export.md](docs/user/ab-compare-and-export.md).

### Offline render

`wing-render` runs the same engine faster than real time from files, for bounces and regression
checks, with no driver or daemon (any platform):

```bash
wing-render --scene mix.snap --rpp project.RPP --out bounce/ --taps main/1,bus/3 --tail 3
wing-render --scene mix.snap --inputs stems/ --out bounce/ --set io/altsw=1   # WAVs by leading track number
```

It writes 32-bit float WAVs per tap and `render_report.json` (peaks, loudness, speed). `--inputs`
works with stems from any DAW (file `07 Snare.wav` feeds USB input 7); `--rpp` reads a REAPER
project's tracks and hardware outputs. `--dry-run` prints which file feeds which USB input; `--set
path=value` tweaks parameters (`io/altsw=1` switches the console to ALT inputs, i.e. USB playback).
`wing-render --help` has all options.

### Capturing console outputs

Capture console taps to disk while the DAW plays (macOS):

```bash
wing-sim record start main/1,bus/3,usb_out/1-48 ~/Recordings/take1
wing-sim record stop          # also: wing-sim record status
```

One 32-bit float WAV per stereo tap or channel group. The same commands exist over OSC
(`/wingsim/record/start ,ss <taps> <dir>`), and the menu-bar app shows the recording state. Your DAW
can also record the console's USB outputs directly (DAW input N = USB output N).

### DAW integration: scene changes on the timeline

Everything is plain OSC on UDP 2223, so any DAW or controller that sends OSC can recall scenes
(`/wingsim/scene/recall ,s Verse`) or follow markers (`/wingsim/marker ,s WING:Verse`). For REAPER
there is a ready setup: name a marker or region `WING:Verse` (or `WING:#0` for the first library
scene) and the simulator recalls that scene as playback crosses it, plus scripts to store/step scenes
and optional fader automation. Setup and the OSC reference:
[docs/user/reaper-integration.md](docs/user/reaper-integration.md); demo project:
`integrations/reaper/demo/WingSim-Demo.RPP`.

### Ports, and running a second copy

By default the daemon binds UDP 2222 (discovery), TCP 2222 (native protocol) and UDP 2223 (OSC). To
run another instance without disturbing a live one (tests, scripts), give it private ports:

- `--port N`: native TCP port; discovery follows it unless `--discovery-port` is given, so a
  non-default `--port` never binds UDP 2222. `--port 0` picks free ports.
- `--discovery-port N`: UDP discovery port on its own.
- `--no-discovery`: no discovery socket at all (WING-Edit will not list it).
- `--osc-port N` / `--no-osc`: OSC port (default 2223).

`wing-sim --help` lists everything else (name, IP, firmware string, block size, host outputs).

## Troubleshooting

| Symptom | Fix |
|---|---|
| WING-Edit does not list the simulator | The daemon must be running (menu shows "running"). Same network/subnet as WING-Edit; allow incoming connections for `wing-sim` in the firewall prompt; no VPN capturing broadcasts. On one computer it works over loopback; with several network interfaces start it with `--ip A.B.C.D` of the one WING-Edit uses. |
| WING-Edit refuses to connect or says the firmware is unsupported | WING-Edit 3.3.4 only talks to consoles reporting a compatible firmware. The simulator advertises `3.3.4-0-g0c2b9d4a:release` by default; change `--fw` only for another WING-Edit version (use that version's firmware string). |
| "Lost connection! Auto reconnect is active." and it never reconnects | After the daemon restarts WING-Edit does not retry by itself: press OK, select the console again, SYNC DIRECTION = PC -> Mixer, CONNECT. Idle connections are dropped after 10 s of silence (WING-Edit keeps them alive). |
| The DAW has no "Wing Simulator" device | The driver is not installed or macOS audio was not restarted: run the installer again (or `./install.sh --driver-only`), or check Audio MIDI Setup. See the install checklist. |
| Silence from the DAW, meters flat, but the device is there | The daemon is not running or cannot attach to the driver: start it from the menu-bar app. With no daemon the driver is a plain loopback (output N returns on input N), which is also the safe fallback if the daemon stalls: audio crossfades to loopback and back when it recovers. |
| Channels get no signal although the DAW plays | Check the scene's input patch: a channel only hears USB N if its main source, or its ALT source with the ALT switch on, is USB N. |
| Everything is in channels 1 and 2 | The DAW's master/stereo output is going to hardware outputs 1/2; turn that off. |
| Crackles, dropouts on the host output | Raise the extra latency in Host Output Routing (default 5 ms; try 10-20 ms) and the DAW's buffer to 128 or 256. The simulator adds one engine block (64 frames, 1.3 ms at 48 kHz) plus the driver's input delay (128 frames, reported to the DAW for recording compensation). `wing-sim --block 128` raises the engine block. xruns show in the menu status and `status.json`. |
| Nothing on my speakers | The default route plays on the Mac's default output device as it was when the simulator started: pick the device in Host Output Routing, or select it in System Settings > Sound and restart the simulator. |
| Output device missing from the routing list | Press **Refresh Devices**; a route to an unplugged device is kept and reports "not connected" until the device returns. |
| `wing-sim` on Windows/Linux says "audio device is macOS-only" | Expected: control-only mode. Use `wing-render` for audio. |
| Daemon logs and state | Menu > **Log** and **State Folder**. |

Known limits: 48 kHz only (44.1 kHz is accepted by the driver but not a tested path); WING-Edit's
MIXER -> PC direction and several simultaneous WING-Edit clients are unverified; behaviour assumed
about real hardware is listed in [docs/VERIFY-ON-HARDWARE.md](docs/VERIFY-ON-HARDWARE.md).

## Documentation

- [docs/user/recording-workflow.md](docs/user/recording-workflow.md): record on a WING, mix on the simulator
- [docs/user/install-checklist.md](docs/user/install-checklist.md): fresh-machine install and smoke test
- [docs/user/reaper-integration.md](docs/user/reaper-integration.md): REAPER scripts, markers, automation, OSC reference
- [docs/user/ab-compare-and-export.md](docs/user/ab-compare-and-export.md): A/B slots, export to a console
- [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md), [docs/fidelity-report.md](docs/fidelity-report.md),
  [CLAUDE.md](CLAUDE.md) (developer guide)

## Building from source

Needs Rust (rustup; the repo pins stable). On macOS the driver and menu-bar app additionally need the
Xcode command-line tools (Swift 6.2+) and `cmake` (`brew install cmake`).

```bash
make check     # fmt, clippy, tests (any platform: cargo fmt/clippy/test work the same)
make dist      # macOS: everything into dist/WingSimulator-<version>/ (app, driver, tools, installer)
make app       # macOS: just the menu-bar app
cargo build --release -p wing-sim -p wing-render -p wing-diff   # tools only, any platform
```

Releases are built by [.github/workflows/release.yml](.github/workflows/release.yml) when a `vX.Y.Z`
tag matching `[workspace.package] version` is pushed.

## Licence and signing

MIT (see [LICENSE](LICENSE)). Third-party: libASPL (MIT) for the driver; parts of `wing-proto` are
derived from libwing (Apache-2.0, see [crates/wing-proto/NOTICE](crates/wing-proto/NOTICE)).
Builds are signed ad hoc only (no Apple Developer ID, not notarised) and Windows binaries are
unsigned; notarisation is future work and needs a paid developer account.
