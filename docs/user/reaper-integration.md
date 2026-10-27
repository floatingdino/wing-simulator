# Driving the Wing Simulator from REAPER

The simulator can follow REAPER's timeline: markers named `WING:<scene>` recall library scenes as
playback crosses them, scripts store/recall scenes from REAPER, and (optionally) REAPER track
volume/mute automation drives WING channel faders/mutes. Everything goes over the simulator's
OSC port (UDP 2223) on the same machine. Files: `integrations/reaper/` (T065).

Other DAWs: nothing here is REAPER-only on the simulator side. Any DAW or controller that can send OSC
(or a small helper script) can recall scenes with `/wingsim/scene/recall ,s <name>` or forward marker
names to `/wingsim/marker`; see the OSC reference below. For recording a WING multitrack and replaying
it into the simulator, see [recording-workflow.md](recording-workflow.md).

| file | what it is |
|---|---|
| `WingSim-Markers.ReaperOSC` | OSC control-surface pattern config: marker/region names only |
| `WingSim-Faders.ReaperOSC` | the same plus tracks 1-40 volume/mute → `/ch/N/fdr`, `/ch/N/mute` |
| `wing_osc.lua` | shared helper for the scripts (OSC encoder, sender); not an action itself |
| `WING - Recall scenes at markers.lua` | background script: recall scenes when playback crosses `WING:` markers |
| `WING - Store scene.lua` | store the current state as a scene, optionally add a `WING:` marker |
| `WING - Recall scene.lua` | recall a scene by name or `#N` |
| `WING - Next scene.lua`, `WING - Previous scene.lua` | step through the library |
| `demo/WingSim-Demo.RPP` | demo project: markers Intro/Verse/Chorus, a Bridge region, a `WING:#1` marker, kick volume automation |

## Scene markers

Name a marker or region `WING:<scene name>` (for example `WING:Verse`) or `WING:#<n>`. The prefix
is case-insensitive and blanks around the name are ignored. Names are matched exactly first, then
case-insensitively. `#n` is the **0-based** position in the library (`#0` is the first scene),
the same convention as `wing-sim --recall #N` and `$ctl/lib/$actionidx`. Markers without the
prefix are ignored, so normal song markers can stay. A region triggers at its start.

Scenes live in the simulator's library (`<state dir>/scenes/NNN Name.snap`, T017;
`wing-sim --list-scenes` prints them). A marker naming a scene that does not exist does nothing
(the simulator logs `no scene "..."`).

## Two ways to follow the timeline (pick one)

**A. Control surface (no scripts, recommended for scene changes).** REAPER's OSC surface sends
the name of the last marker/region at or before the play cursor whenever it changes; the
simulator recalls `WING:` names. Nothing has to keep running inside REAPER. Semantics: a
recall happens when the "last marker" changes, i.e. when playback crosses a marker, when you
seek/click into another section (also while stopped: the edit cursor counts), and once when the
surface starts. Looping back to the same marker with no other marker in between does not
re-recall (the name did not change).

**B. `WING - Recall scenes at markers.lua` (explicit crossings).** A background script that
watches the play position (only while playing): each `WING:` marker/region start the playhead
crosses is recalled; on play start, a loop wrap or a seek it chases the last `WING:` marker at or
before the playhead, so loops re-recall their scene. It starts `nc` for each recall (macOS/Linux).
Run the action again to stop it (as a toolbar button it shows on/off).

Using both at once recalls every scene twice.

## Setup

1. Start the simulator as usual (`wing-sim`, OSC on UDP 2223 unless `--osc-port` is given).
2. **Scripts:** copy the whole `integrations/reaper/` folder (the scripts need `wing_osc.lua`
   next to them) into REAPER's `Scripts` folder (Options > Show REAPER resource path), then
   Actions > Show action list > New action > Load ReaScript and select the `WING - ...lua`
   files. Assign keys or toolbar buttons as you like.
   The scripts send to `127.0.0.1:2223`; to change that, run once in a ReaScript console
   or script: `reaper.SetExtState("WingSim", "port", "2224", true)` (key `host` for the address).
3. **Control surface (way A, or faders):** copy `WingSim-Markers.ReaperOSC` (or
   `WingSim-Faders.ReaperOSC`) into REAPER's OSC folder: Options > Preferences >
   Control/OSC/web > Add > Control surface mode "OSC (Open Sound Control)", next to "Pattern
   config" click "Open config directory", copy the file there, close and reopen the dialog, then:
   - Device name: `Wing Simulator`
   - Pattern config: `WingSim-Markers` (or `WingSim-Faders`)
   - Mode: `Configure device IP+local port`
   - Device IP: `127.0.0.1`, Device port: `2223`
   - Local listen port: any free port that is not 2222/2223 (for example 9010). The simulator
     does not send feedback to REAPER.
   - Leave "Allow binding messages to REAPER actions and FX learn" off.
4. Try the demo: open `integrations/reaper/demo/WingSim-Demo.RPP`. Create the scenes once: move
   the edit cursor to each `WING:` marker, set up the mix (WING-Edit or any OSC client) and run
   **WING - Store scene** (the dialog proposes the marker's scene name; answer `n` to "Add
   marker", the demo already has them). Then play from the start: the simulator logs
   `recalled scene #0 "Intro"` at 0 s, `Verse` at 16 s, `Chorus` at 32 s, `Bridge` at 48 s and
   scene `#1` at 56 s. Any WING client (WING-Edit, `$ctl/lib/$active`) follows.

## Automation (faders/mutes)

With `WingSim-Faders.ReaperOSC`, REAPER sends each track's volume in dB and its mute state as
control-surface feedback, including during automation playback, at the surface update rate:
REAPER track N → `/ch/N/fdr ,f <dB>` and `/ch/N/mute ,f 0|1` for N = 1..40. Values outside the
WING fader range clamp (-inf → -144 dB "-oo", above +10 dB → +10). The demo's Kick track ducks
by 6 dB through the verse.

Limits:
- One-way and positional: REAPER track N is WING channel N, whatever the names; the simulator's
  fader moves do not move REAPER. Pan, EQ, sends, busses are not mapped (the REAPER pattern
  types for those are normalised to REAPER's own curves, not WING units).
- REAPER sends the current value of every mapped track when the surface starts and on project
  changes, and a scene recall does not move REAPER's faders, so REAPER's values win on the next
  update of that track. Use the markers-only config if scenes should own the faders.
- REAPER only sends feedback for tracks that exist, and the update rate is REAPER's surface rate
  (roughly 30 Hz), not sample accurate.
- JSFX cannot send network messages, and REAPER's Lua has no sockets: the scripts start a
  short-lived `nc` process per message, which is fine for scene changes but not for continuous
  automation. Windows has no `nc`; on Windows use the control-surface configs only.

## OSC reference (simulator extensions, T065)

These are not WING console addresses. Successful commands are not acknowledged; the result is
written to the simulator log and visible as `$ctl/lib/$active` / `$activeid`.

| address | args | effect |
|---|---|---|
| `/wingsim/scene/recall` | `,s name` or `,s #N` / `,i N` | recall a scene |
| `/wingsim/scene/store` | none / `,s ""` / `,s name` | store: overwrite the active scene (or append `Scene N`) / store as `name` (overwrite or append) |
| `/wingsim/scene/next`, `/wingsim/scene/prev` | none or a number (0 = button release, ignored) | recall the scene after/before the active one |
| `/wingsim/scenes` | none | reply `/wingsim/scenes ,s...` with the scene names in order |
| `/wingsim/marker`, `/wingsim/region` | `,s marker name` | `WING:<scene>` recalls; other names and repeats of the previous name are ignored |
| `/$ctl/lib/$action` | `,s GO` / `GONEXT` / `GOPREV` / `NEXT` / `PREV` / `STORE[:name]` / `RENAME:name` / `DELETE` / `GO:name`, or `,i` enum index | the console library command with `/$ctl/lib/$actionidx` as the index, as over the native protocol (T017) |

Errors: missing argument → `/* ,s "INCOMPLETE DATA"`; unknown `/wingsim/` address, or a simulator
started without a library → `/* ,s "NODE NOT FOUND"`.

Quick test without REAPER (any OSC sender, e.g. Python `python-osc`):
`/wingsim/scene/store "Intro"`, change something, then `/wingsim/marker "WING:Intro"`.

## Testing

- `cargo test -p wing-sim osc_store_and_recall_loopback`: store/recall over UDP loopback on
  ephemeral ports (store by `/wingsim/scene/store` and `$action STORE:`, recall by marker,
  name, index, `$actionidx`+`GO`, `next`; fader/mute sets in REAPER's feedback form).
- `cargo test -p wing-proto osc::service`: routing and marker parsing.
- `cargo test -p wing-sim --test reaper_integration`: every address in the `.ReaperOSC` files is
  handled, the demo project's markers parse, and the Lua scripts pass `luac -p` when a Lua
  5.3/5.4 `luac` is on PATH (skipped otherwise; REAPER embeds its own Lua). The scripts use
  Lua 5.3+ features (`string.pack`, `math.type`, `&`), as in REAPER 6/7.

Unverified against a real REAPER in T065: the pattern-config names `LAST_MARKER_NAME`,
`LAST_REGION_NAME`, `TRACK_VOLUME (f, dB)` and `TRACK_MUTE` are taken from REAPER's
`Default.ReaperOSC`; if your REAPER version lacks one, compare with that file in the same folder.
