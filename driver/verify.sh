#!/bin/bash
# Post-install verification of the Wing Simulator HAL driver against the REAL
# installed driver in coreaudiod. No root needed; run by a human after
#   (quit REAPER)  driver/build.sh && sudo driver/install.sh
# as:
#   driver/verify.sh
#
# Checks (each prints PASS/FAIL lines; exit status = number of failed steps):
#   1. installed bundle == driver/build bundle; driver log shows the shm attach
#   2. device: 48x48, names USB In/Out n, rates, latency == configured input delay
#   3. loopback (no daemon): gain +1, strict (no glitches/xruns)
#   4. sample rates 44.1k / 48k, rate change reaches the shm header (daemon sees it)
#   5. Rust gain_client (--rt) attached: gain -1 end to end, strict
#   6. wake latency from inside the coreaudiod sandbox (wake_latency --attach
#      observer while the probe runs IO): wakes delivered, p99 <= 1 ms
#   7. no xruns for VERIFY_SECONDS (default 60) s at IO buffers 64/128/256 with
#      the daemon path active (HAL overloads, timestamp gaps, glitch frames,
#      driver-side from_daemon underruns all 0)
#   8. robustness: daemon stalled (SIGSTOP/SIGCONT) and killed (SIGKILL) during
#      IO -> no clicks; loopback after the crash; a restarted daemon is picked up
#
# Environment: VERIFY_SECONDS (xrun run length, default 60),
#              VERIFY_BUFFERS (default "64 128 256").
# The terminal app needs Microphone permission (wing_hal_probe captures input).
set -uo pipefail

here="$(cd "$(dirname "$0")" && pwd)"
root="$(cd "$here/.." && pwd)"
build="$here/build"
probe="$build/wing_hal_probe"
installed="/Library/Audio/Plug-Ins/HAL/WingSimulator.driver"
shm="/wingsim.audio.v1"
seconds="${VERIFY_SECONDS:-60}"
buffers="${VERIFY_BUFFERS:-64 128 256}"
logdir="$(mktemp -d "${TMPDIR:-/tmp}/wingsim-verify.XXXXXX")"
gc="$root/target/release/examples/gain_client"
wl="$root/target/release/examples/wake_latency"
gc_pid=""
failed=()

step() { printf '\n== %s\n' "$*"; }
result() { # status, name
    if [ "$1" -eq 0 ]; then echo "-- ok: $2"; else echo "-- FAILED: $2"; failed+=("$2"); fi
}
cleanup() {
    if [ -n "$gc_pid" ]; then kill -CONT "$gc_pid" 2>/dev/null; kill "$gc_pid" 2>/dev/null; wait "$gc_pid" 2>/dev/null; fi
}
trap cleanup EXIT

# Daemon heartbeat age in ms from `gain_client --status`, or "never".
daemon_age() {
    "$gc" --status --name "$shm" 2>/dev/null |
        awk '/^daemon /{ if ($0 ~ /never/) print "never"; else { sub(/.*heartbeat /,""); print $1 } }'
}
daemon_alive() { # heartbeat younger than 100 ms
    local a; a="$(daemon_age)"
    [ -n "$a" ] && [ "$a" != "never" ] && awk -v a="$a" 'BEGIN{ exit !(a < 100) }'
}
start_daemon() {
    # --rt: time-constraint thread, as the real daemon will run (T022). A
    # normal-priority client needs a much larger InputDelayFrames (driver.md 9.2).
    "$gc" --name "$shm" --rt >>"$logdir/gain_client.log" 2>&1 &
    gc_pid=$!
    for _ in $(seq 1 50); do daemon_alive && return 0; sleep 0.1; done
    return 1
}

# ---- 0. prerequisites ----
step "0. prerequisites"
if pgrep -xq REAPER; then
    echo "REAPER is running: quit it first (it holds the device and may force its buffer size)"
    exit 1
fi
if [ ! -x "$probe" ]; then echo "not built: run driver/build.sh first"; exit 1; fi
if [ ! -d "$installed" ]; then echo "driver not installed: sudo driver/install.sh"; exit 1; fi
(cd "$root" && cargo build --release -p wing-shm --examples) || { echo "cargo build failed"; exit 1; }
if daemon_alive; then
    echo "a daemon is already attached to $shm (wing-sim / gain_client / wing_shm_client): stop it first"
    exit 1
fi
default_delay="$(awk '/#define WING_SHM_DEFAULT_INPUT_DELAY_FRAMES/{ sub(/u$/,"",$3); print $3 }' "$here/include/wing_shm.h")"
delay="$(defaults read /Library/Preferences/Audio/com.wingsimulator.driver InputDelayFrames 2>/dev/null || echo "$default_delay")"
echo "expected input delay: $delay frames (default $default_delay)"
echo "logs: $logdir"

# ---- 1. installed build ----
step "1. installed bundle and driver log"
cmp -s "$build/WingSimulator.driver/Contents/MacOS/WingSimulator" "$installed/Contents/MacOS/WingSimulator"
result $? "installed driver binary == driver/build (else reinstall)"
log show --last 30m --style compact --predicate 'subsystem == "com.wingsimulator.driver"' 2>/dev/null |
    grep 'Wing Simulator' | tail -8 | tee "$logdir/driver_log_start.txt"
grep -q "shm $shm attached" "$logdir/driver_log_start.txt" ||
    echo "(note: no 'shm attached' line in the last 30 min of the driver log; check: log show --last 1h --predicate 'subsystem == \"com.wingsimulator.driver\"')"

# ---- 2. device properties ----
step "2. device properties"
"$probe" --info --expect-latency "$delay"
result $? "48x48, names, latency (input $delay, output 0)"
"$gc" --status --name "$shm" | tee "$logdir/status_start.txt"
grep -q "^segment        v2, 48 ch, ring 8192 frames, input delay $delay frames" "$logdir/status_start.txt"
result $? "shm segment v2 with input delay $delay"

# ---- 3. loopback ----
step "3. loopback (no daemon)"
"$probe" --seconds 3 --expect-gain 1 --strict --shm "$shm"
result $? "loopback gain +1, strict"

# ---- 4. sample rates ----
step "4. sample rates"
for r in 44100 48000; do
    "$probe" --rate "$r" --seconds 2 --expect-gain 1 --strict
    rc=$?
    "$gc" --status --name "$shm" | grep -q "rate $r," || rc=1
    result $rc "rate $r: device + shm header (daemon notification)"
done

# ---- 5. daemon path ----
step "5. gain_client (Rust daemon side) attached: gain -1"
start_daemon
result $? "gain_client attached (heartbeat fresh)"
"$probe" --seconds 5 --expect-gain -1 --strict --shm "$shm"
result $? "gain -1 end to end, strict"

# ---- 6. wake latency under the real sandbox ----
step "6. wake latency (driver in coreaudiod -> observer)"
"$probe" --seconds 13 --expect-gain -1 --shm "$shm" >"$logdir/probe_wake.txt" 2>&1 &
probe_pid=$!
sleep 1.5
"$wl" --attach "$shm" --seconds 10 --max-p99-us 1000
rc=$?
wait "$probe_pid" || rc=1
tail -4 "$logdir/probe_wake.txt"
result $rc "os_sync wakes delivered from the driver host, p99 <= 1 ms"

# ---- 7. xruns ----
for b in $buffers; do
    step "7. ${seconds} s at IO buffer $b frames (daemon path)"
    "$probe" --buffer "$b" --seconds "$seconds" --expect-gain -1 --strict --shm "$shm"
    result $? "no xruns/glitches/underruns at buffer $b"
done

# ---- 8. robustness ----
step "8. daemon stall + crash during IO: no clicks"
"$probe" --seconds 9 --transition --shm "$shm" >"$logdir/probe_transition.txt" 2>&1 &
probe_pid=$!
sleep 2.5; kill -STOP "$gc_pid"   # stall: crossfade to loopback
sleep 2;   kill -CONT "$gc_pid"   # resume: crossfade back
sleep 2;   kill -KILL "$gc_pid"   # crash: heartbeat goes stale -> loopback
wait "$gc_pid" 2>/dev/null; gc_pid=""
wait "$probe_pid"; rc=$?
cat "$logdir/probe_transition.txt" | grep -E 'xruns|continuity|shm|PASS|FAIL'
result $rc "stall/resume/crash transitions are click-free"
"$probe" --seconds 2 --expect-gain 1 --strict
result $? "loopback after daemon crash"
start_daemon
result $? "restarted gain_client attached"
"$probe" --seconds 3 --expect-gain -1 --strict --shm "$shm"
result $? "restarted daemon picked up (gain -1)"
cleanup; gc_pid=""

# ---- summary ----
step "driver log (StopIO totals, min daemon slack per IO session)"
log show --last 15m --style compact --predicate 'subsystem == "com.wingsimulator.driver"' 2>/dev/null |
    grep -E 'StopIO|StartIO|re-attached|config' | tail -12
step "summary"
if [ ${#failed[@]} -eq 0 ]; then
    echo "ALL PASSED"
else
    echo "FAILED (${#failed[@]}):"
    printf '  - %s\n' "${failed[@]}"
fi
exit ${#failed[@]}
