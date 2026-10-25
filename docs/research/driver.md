# Wing Simulator HAL driver (T006 spike)

Status: the spike builds and passes its host-side tests. It has **not been installed yet**, because
installing needs root. A human has to run the install (see "Install" below).

## 1. What was built

`driver/` is a C++17 AudioServerPlugIn built on **libASPL v3.1.2** (MIT). CMake FetchContent pins it to tag
`v3.1.2`, commit `47f688ed`.

| Item | Value |
|---|---|
| Bundle | `driver/build/WingSimulator.driver` (universal arm64 + x86_64, ad-hoc signed) |
| Device name / UID | `Wing Simulator` / `WingSimulator_Device_UID` |
| Output (DAW → sim) | 1 stream × 48 ch, float32. Channel names `USB In 1..48` (WING "USB in") |
| Input (sim → DAW) | 1 stream × 48 ch, float32. Channel names `USB Out 1..48` (WING "USB out") |
| Rates | 48000 (default), 44100. A rate change set on either the device or a stream format updates both streams and the shm header |
| Input latency | 256 frames, reported as the input stream latency. This is the shm processing window (see section 4) |
| Without daemon | Loopback: input(T) = output(T − 256) |
| IPC | POSIX shm `/wingsim.audio.v1` (3 MiB). Layout is in `driver/include/wing_shm.h` |

Files:

- `driver/include/wing_shm.h`: versioned C layout and atomic helpers, shared with the future Rust `wing-shm` crate (T020).
- `driver/src/ShmLink.{hpp,cpp}`: driver-side ring logic. It has no CoreAudio dependency and is safe to call from the real-time thread: no allocation, no locks, no syscalls except `mach_absolute_time`.
- `driver/src/Driver.cpp`: the libASPL device, channel names, sample-rate plumbing and IO handler.
- `driver/tools/wing_shm_client.cpp`: a test "daemon". It attaches to the shm and applies a gain (default −1, which inverts polarity).
- `driver/tools/wing_shm_selftest.cpp`: drives `ShmLink` with simulated HAL cycles against a real `wing_shm_client` process.
- `driver/tools/wing_driver_smoketest.cpp`: `dlopen`s the built bundle and drives it through the AudioServerPlugIn C interface with a fake host.
- `driver/tools/wing_hal_probe.cpp`: post-install end-to-end probe through the public CoreAudio API. It plays a DC level 0.01·(c+1) on each output channel and measures the gain on each input channel.
- `driver/build.sh`, `driver/install.sh`, `driver/uninstall.sh`.

### Build and test results (2026-10-25, macOS 26.6.2, CMake 4.4.4, Xcode clang)

```
$ driver/build.sh --test
-- built driver/build/WingSimulator.driver   (valid on disk; x86_64 arm64)
1/2 shm_selftest ......... Passed   (loopback 50688 frames exact; daemon gain -0.5 76288 frames exact,
                                     0 underruns; epoch restart followed; detach -> loopback)
2/2 driver_smoketest ..... Passed   (Initialize, 1 device, 48/48 float32, 44.1k+48k, channel names,
                                     input latency 256, rate change 48k->44.1k->48k reaches streams+shm,
                                     StartIO bumps epoch, 50432 frames exact loopback via DoIOOperation)
```

## 2. IPC mechanism: POSIX shm, and why

Constraint: on current macOS (observed on 26.6.2), third-party HAL plug-ins do not run inside `coreaudiod` itself. They run in a
per-driver helper process, `Core Audio Driver (X.driver)`. That process is
`/System/Library/Frameworks/CoreAudio.framework/XPCServices/com.apple.audio.Core-Audio-Driver-Service.helper.xpc`
and runs as user `_coreaudiod`. On this machine it is visible for `ParrotAudioPlugin.driver`. The helper is
sandboxed. Its entitlements are only `com.apple.private.audio.driver-host` and
`com.apple.security.cs.disable-library-validation`. There is no app-sandbox container and no app groups,
so app-group shm names (`TEAMID.group/...`) do not apply. The helper binary contains the strings
`AudioServerPlugIn_MachServices` and `AudioServerPlugIn_IOKitUserClients` and the message "requires
extending the sandbox for the mach service named %s". This matches
`/System/Library/Sandbox/Profiles/com.apple.audio.coreaudiod.sb`, whose rules
`(allow mach-lookup (extension "com.apple.audio.driver-host.mach-service-name"))` and
`(allow iokit-open (extension "com.apple.audio.driver-host.iokit-user-client-class"))` exist to serve
driver hosts. So the coreaudiod profile is the policy that governs plug-ins.

What that profile (macOS 26.6.2) allows a plug-in to do:

| Mechanism | Verdict | Evidence |
|---|---|---|
| **POSIX shm** (`shm_open`/`mmap`) | **allowed, any name** | `(allow ipc-posix-shm)` is unconditional |
| POSIX named semaphores (`sem_open`) | denied | no `ipc-posix-sem` rule; `deny default` |
| File in `/tmp`, `/Users/Shared`, `~` (mmap file) | denied for write | `file-write*` only allows `/Library/Preferences{,/Audio}`. A test confirmed `fopen("/tmp/…","w")` → EPERM |
| File in `/Library/Preferences/Audio` | allowed for coreaudiod, but the daemon (a normal user) cannot write there without root | profile |
| Mach/XPC service | possible only for names listed in Info.plist `AudioServerPlugIn_MachServices`, and needs a launchd-registered service. That is a heavier lift (Background Music uses it) | profile + helper strings |
| Sockets | `(allow network*)` | works, but is a syscall per buffer on the real-time thread. Rejected |
| `os_sync_wait_on_address` / `os_sync_wake_by_address_any` with `OS_SYNC_*_SHARED` (macOS 14.4+) | not a sandbox operation; works on shm addresses | SDK header present. Candidate wake-up primitive for T020 |

Empirical check (no root): I ran a probe under
`sandbox-exec -f <profile that imports com.apple.audio.coreaudiod.sb + exec of the probe only>`.
- It opened an existing POSIX shm segment created by user `sam`, read-write: **OK**.
- It mapped and wrote it: **OK**.
- It created a new segment: **OK**.
- Writing a file in `/tmp`: **EPERM**.
- Writing a file in `/Users/Shared`: **EPERM**.

macOS pshm quirks found and handled:
- Names are limited to 31 chars (`PSHMNAMLEN`). `/wingsim.audio.v1` fits.
- The creation mode is applied verbatim: umask is not applied, and `0666` with umask 022 stayed `0666`.
- `fchmod()` on a pshm fd returns **EINVAL**, so the mode must be right at `shm_open`. The driver uses `0666` so the daemon (a different uid from `_coreaudiod`) can attach.
- `ftruncate()` works **only once**. A second call returns EINVAL. A segment with a wrong size or version therefore has to be `shm_unlink`ed and recreated.
- Segments persist until `shm_unlink` or reboot, so they survive `coreaudiod` restarts.

Ownership policy:
- The driver attaches to an existing compatible segment, or creates one.
- If shm fails entirely, it falls back to private memory with the same layout, which still gives loopback.
- The daemon attaches. `wing_shm_client --create` lets a daemon create the segment first.
- Security note: mode `0666` means any local user can read or inject audio. That is acceptable for this single-user dev tool and is noted for T021.

Remaining risk: the driver-host helper could in principle apply a stricter profile than coreaudiod.sb. The
`os_log` lines `shm /wingsim.audio.v1 attached` or `shm unavailable (...); loopback only` tell us after
install. If shm turns out to be blocked, the fallback is a Mach service through
`AudioServerPlugIn_MachServices`, which would carry a shm handle or memory entry.

## 3. Shared-memory layout (v1, `wing_shm.h`)

- A 4096-byte header, followed by two planar float32 rings of `48 × 8192` frames each:
  - `to_daemon`: the DAW playback mix.
  - `from_daemon`: what the DAW records.
- Each ring is indexed by **absolute device sample time**: `ring[c][t & 8191]`.
- Each ring has one producer, which publishes `write_time` (the exclusive end) with a release store.
- Header fields:
  - `io_epoch`, `sample_rate`, `io_running`, `loopback_active`: written by the driver.
  - `driver_heartbeat`, `daemon_heartbeat`: `mach_absolute_time` values.
  - Per-ring `epoch`, `read_time`, `overruns`, `underruns`.
- Atomics use `__atomic_*` builtins on plain fields, so the header is valid C, C++ and Rust (`AtomicU64` on the same addresses).
- `static_assert`s pin the offsets.

## 4. Timing contract

- libASPL restarts device sample time at 0 on every StartIO, and the HAL does the same on a rate change.
  At each of these the driver resets `to_daemon.write_time` and then increments `io_epoch` (release store).
  The daemon reacts to a new epoch by skipping its backlog and stamping `from_daemon.epoch`. The driver
  ignores `from_daemon` until that epoch matches.
- WriteMix(T) writes `to_daemon[T..T+n)`. ReadInput(T) reads `from_daemon[T−256..)` when the daemon is
  alive, or `to_daemon[T−256..)` in loopback.
- The fixed 256-frame delay is reported as the input stream latency, so REAPER's recording compensation
  stays correct in both modes.
- It gives the daemon a window of roughly the IO buffer plus 256 frames (≥ 5.3 ms at 48 kHz).
- Frames that are missing or late are delivered as zeros, and `from_daemon.underruns` is incremented.
- The daemon counts as alive if `daemon_heartbeat` is less than 100 ms old. When it exits it sets the
  heartbeat to 0, which switches the driver back to loopback immediately.

## 5. Install (human, needs sudo)

Do not run these from an agent. Quit REAPER first, then:

```bash
cd /Users/sam/code/wing-simulator      # or this worktree, before merge
driver/build.sh --test                 # as your user (fetches libASPL, builds, runs ctest)
sudo driver/install.sh
```

`install.sh` does the equivalent of:

```bash
sudo rm -rf /Library/Audio/Plug-Ins/HAL/WingSimulator.driver
sudo cp -R driver/build/WingSimulator.driver /Library/Audio/Plug-Ins/HAL/
sudo chown -R root:wheel /Library/Audio/Plug-Ins/HAL/WingSimulator.driver
sudo launchctl kickstart -k system/com.apple.audio.coreaudiod   # or: sudo killall coreaudiod
```

To uninstall: `sudo driver/uninstall.sh`. It removes the bundle and restarts coreaudiod.

Ad-hoc signing is enough. The driver host has `disable-library-validation`, so it loads plug-ins built
locally (BlackHole is distributed the same way when built from source).

## 6. Post-install verification (no sudo)

```bash
log stream --predicate 'subsystem == "com.wingsimulator.driver"' &   # expect "shm /wingsim.audio.v1 attached"
driver/build/wing_hal_probe --info            # PASS 48x48, rates 44100 48000, names USB In/Out n
driver/build/wing_hal_probe                   # loopback: PASS gain 1.0 on all 48 channels
driver/build/wing_shm_client &                # daemon stand-in, gain -1
driver/build/wing_shm_client --status         # epoch, heartbeats, write_time advancing, underruns
driver/build/wing_hal_probe --expect-gain -1  # PASS: audio went driver -> shm -> client -> shm -> driver
kill %2                                       # client detaches -> loopback again
driver/build/wing_hal_probe --rate 44100      # PASS at 44.1k
driver/build/wing_hal_probe --rate 48000
```

`wing_hal_probe` captures input, so the terminal app needs Microphone permission (System Settings →
Privacy & Security → Microphone). If `0 input frames` is reported, that permission is the cause.

REAPER check:
1. Preferences → Audio → Device: CoreAudio, set both input and output to "Wing Simulator", 48000.
2. Arm a track on input 1, play a test tone on another track to hw out 1, and record. The recording
   should be the tone, delayed and latency-compensated.
3. With `wing_shm_client` running, the recording should be polarity-inverted.

## 7. REAPER configuration findings (read-only)

`~/Library/Application Support/REAPER/reaper.ini` (REAPER 7.34, macOS arm64):

- `coreaudioindevnew=Scarlett 2i4 USB` and `coreaudiooutdevnew=Scarlett 2i4 USB`. The current device is
  the Scarlett, not a WING. REAPER stores the device by name, so selecting "Wing Simulator" works as soon
  as the device exists.
- `coreaudiosrate=48000` with `coreaudiosrateuse=0`: the rate is not forced, so REAPER follows the device's
  nominal rate. Our default is 48k.
- `coreaudiobs=512` with `coreaudiobsuse=0`: the buffer size is not forced.
- There are no channel-name aliases in the ini, so REAPER will show the driver's names
  (`Output 1 (USB In 1)` …).

Most recently opened project: `lastproject=/Users/sam/Documents/REAPER Media/2025-06-29_AM.RPP`. It is a
48-channel WING multitrack / virtual-soundcheck project. `PBLC_BRIS_WING.RPP` in Recent is on an unmounted
volume.

- `SAMPLERATE 44100 0 0`: the project rate is not enforced.
- 48 record tracks. **Track input n = mono hardware input n**, and **each track has a mono hardware output
  send `HWOUT 1024+(n−1)` = hardware output n**. Inputs and outputs run 1..48 contiguously and 1:1.
- The tracks sit under folder tracks: Band, Drums, Keys 1, Guitar 1, VOX, Spotify, FOH Link, Stream Link.
  The folders have no hardware outputs.
- Send mode (2nd HWOUT field):
  - `1` = pre-FX for 1–9, 17–21, 25–31, 33–35.
  - `0` = post-fader for the rest.
- Every track also has `MAINSEND 1` (sends to master). The master has `MASTERHWOUT 0 …` = stereo hardware out 1/2.

Mapping onto the simulator, with REAPER's device set to "Wing Simulator":

| REAPER | Driver channel | WING (simulated) |
|---|---|---|
| hw output n (track n playback, `HWOUT 1024+n−1`) | output ch n "USB In n" | USB input n. Patch WING channel n source = USB n for virtual soundcheck |
| hw input n (track n record, `REC … n−1`) | input ch n "USB Out n" | USB output n. WING's USB out n is normally patched to channel n direct out / pre-processing |
| master out 1/2 (`MASTERHWOUT 0`) | output ch 1–2 | **collides with USB In 1/2 (Kick, Snare)**: the master mix is summed onto those two WING inputs during playback |

Recommendations for using this project with the simulator:
- Remove the master hardware output (or route it elsewhere) during virtual soundcheck, otherwise
  USB In 1/2 carry Kick/Snare plus the full mix.
- WING-side routing state (USB in n → ch n, ch n → USB out n) belongs to the daemon's scene. That is T024.

## 8. Known limitations / proposed follow-ups

- The daemon is woken by polling. T020/T022 should use `os_sync_wait_on_address` (shared) or a
  driver-published zero timestamp to clock the engine, and join the device's `os_workgroup`.
- If a client only records (no output IO), `to_daemon.write_time` does not advance. The daemon must then
  clock from `driver_heartbeat` / sample time instead (T020).
- Underrun accounting is per ReadInput call and per client, so it is counted once per input client.
- No volume/mute controls are exposed, on purpose: the device behaves like a USB interface.
- The shm is mode 0666. Revisit in T021 (for example, verify the daemon uid via a handshake).
- Not yet verified on the real system: loading by the actual driver host, glitch-free 1 h runs (T021),
  and REAPER behaviour at 64/128/256 buffers.
