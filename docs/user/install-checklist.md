# Fresh-machine install checklist

Use this on a Mac that has never had the simulator. Tick every box; the "if it fails" lines say what to
check. Status: written in T073, the install steps are **not yet run on a clean machine** (the build was
verified on the developer Mac without touching the system driver).

## 0. Before you start
- [ ] Apple silicon Mac, administrator account, WING-Edit 3.3.4 installed (from Behringer), REAPER installed.
- [ ] Quit REAPER and other audio apps (installing the driver restarts macOS audio).
- [ ] Unzipped `WingSimulator-<version>/` (or built it with `make dist`). If it came from a download:
      `ls` shows `install.sh`, `uninstall.sh`, `bin/`, `WingSimMenu.app`, `WingSimulator.driver`.
- [ ] Optional integrity check: `cd WingSimulator-<version> && shasum -a 256 -c SHA256SUMS`

## 1. Install
- [ ] `./install.sh` (read what it says; answer `y` to the driver question; type your password into sudo).
- [ ] Output ends with "driver installed" and "Next steps". If it said "Skipped", run `./install.sh --driver-only`.
- [ ] `ls /Library/Audio/Plug-Ins/HAL/WingSimulator.driver` exists; `~/Applications/WingSimMenu.app` exists.

## 2. Driver
- [ ] Audio MIDI Setup lists **Wing Simulator** (48 in / 48 out). If not: `sudo killall coreaudiod`, wait 5 s; still
      missing: `log show --last 2m --predicate 'process == "coreaudiod"' | grep -i wing`.
- [ ] Loopback self-check: open REAPER (next step) or, from a repo checkout, `driver/verify.sh` (no sudo;
      Terminal needs Microphone permission) prints ALL PASSED.

## 3. App and daemon
- [ ] Open `~/Applications/WingSimMenu.app` (right-click > Open if Gatekeeper complains). A menu-bar icon appears.
- [ ] Menu > **Start wing-sim**: status becomes "wing-sim running"; audio shows the driver attached, 0 xruns.
      The app works without the repo: it uses its bundled copy of the daemon (check `Contents/Resources/bin` contains
      `wing-sim`, `wing-render`, `wing-diff`).
- [ ] `ls ~/Library/Application Support/WingSimulator` shows `status.json` and `control.sock`.

## 4. WING-Edit
- [ ] WING-Edit lists `WINGSIM` (firmware 3.3.4). SYNC DIRECTION = PC -> Mixer, CONNECT.
- [ ] Meters move, a fader change is accepted, connection stays up over 2 minutes.
      If not listed or refused: README troubleshooting (network, firmware gate, "Lost connection").

## 5. REAPER
- [ ] REAPER > Audio device = Wing Simulator, 48 kHz. Send a track to hardware output 1; master track output to 1/2 off.
- [ ] Play: console channel 1 meter moves in WING-Edit; main bus meters follow.
- [ ] Host output: with a Scarlett 2i4 connected, monitor A is audible on its 1/2. Other interface: set a route in
      menu > Host Output Routing (e.g. `main/1 -> "<device name>" 1/2`), Apply & Reload, hear the mix.
- [ ] Solo a channel in WING-Edit: only it plays on the monitor route; release returns the mix.
- [ ] Recording to REAPER: arm a track on input N, the console's USB Out N (for example MAIN L/R if patched) arrives.

## 6. Tools
- [ ] `~/.local/bin/wing-render --help` works (add `~/.local/bin` to PATH if you want it by name).
- [ ] Offline render: `wing-render --scene <a .snap> --inputs <a folder of wavs> --out /tmp/r --dry-run` lists inputs.
- [ ] A scene round trip: menu > Save As..., change something in WING-Edit, **Copy Diff vs Scene File** shows it.

## 7. Uninstall (also test it)
- [ ] Quit the menu-bar app, `./uninstall.sh` (answer `y` for the driver). The HAL folder no longer contains the driver
      and Wing Simulator disappears from Audio MIDI Setup. Scenes in `~/Library/Application Support/WingSimulator` are kept
      unless `--purge-state`.

## Distribution status
Everything is signed ad hoc (`codesign -s -`). There is no Developer ID signature and no notarisation, so a copy
downloaded by browser is quarantined and shows Gatekeeper's "cannot verify developer" dialog; `install.sh` clears the
quarantine flag on its own copies. Future work: Developer ID signing (hardened runtime) for the app and tools,
notarisation with `notarytool` and stapling, a signed `.pkg` installer, an update channel. These need Apple Developer
Program credentials, which the project does not have.
