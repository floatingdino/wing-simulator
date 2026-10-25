#!/bin/sh
# Headless smoke test (T064): the built app launches with a private state dir, prints its
# smoke line and quits by itself within 15 s, without starting a daemon.
set -eu
HERE=$(cd "$(dirname "$0")" && pwd)
EXE="$HERE/build/WingSimMenu.app/Contents/MacOS/WingSimMenu"
[ -x "$EXE" ] || { echo "smoke: $EXE missing; run build.sh" >&2; exit 1; }
TMP=$(mktemp -d /tmp/wsm-smoke.XXXXXX)
OUT="$TMP/out.txt"

WINGSIM_STATE_DIR="$TMP/state" "$EXE" --smoke-test >"$OUT" 2>&1 &
PID=$!
n=0
while kill -0 "$PID" 2>/dev/null; do
    n=$((n + 1))
    if [ "$n" -gt 150 ]; then
        kill "$PID" 2>/dev/null || true
        echo "smoke: FAILED (app did not quit within 15 s)" >&2
        cat "$OUT" >&2
        exit 1
    fi
    sleep 0.1
done
rc=0
wait "$PID" || rc=$?
if [ "$rc" -ne 0 ] || ! grep -q "WingSimMenu smoke: ok (state dir $TMP/state" "$OUT"; then
    echo "smoke: FAILED (exit $rc)" >&2
    cat "$OUT" >&2
    exit 1
fi
if [ -e "$TMP/state/status.json" ] || [ -e "$TMP/state/wing-sim.log" ]; then
    echo "smoke: FAILED (a daemon was started)" >&2
    exit 1
fi
rm -rf "$TMP"
echo "smoke: ok (launched and quit, exit 0)"
