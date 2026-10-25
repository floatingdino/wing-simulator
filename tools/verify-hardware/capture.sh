#!/bin/sh
# Play the kit's test signal into the WING (USB in 1) and record USB out 1/2 to <recordings>/<scene>.wav.
#
#   capture.sh <kit-dir> <scene> [recordings-dir]
#
# Needs sox (brew install sox) and the WING connected over USB as a CoreAudio device (default name "WING",
# override with WING_DEVICE="name as shown in Audio MIDI Setup"). Any other recorder works too: record >= 12 s
# longer than the suite from USB out 1, 48 kHz, 24 bit, and save as <recordings>/<scene>.wav.
set -eu
KIT=${1:?kit dir}
SCENE=${2:?scene name (see run-sheet.md)}
REC=${3:-recordings}
DEV=${WING_DEVICE:-WING}
command -v sox >/dev/null 2>&1 || { echo "sox not found: brew install sox" >&2; exit 1; }
[ -f "$KIT/scenes/$SCENE.snap" ] || { echo "no such scene: $KIT/scenes/$SCENE.snap" >&2; exit 1; }
SECS=$(sed -n 's/.*"suite_seconds": *\([0-9]*\).*/\1/p' "$KIT/manifest.json")
DUR=$((SECS + 14))
mkdir -p "$REC"
echo "recording $DUR s from $DEV -> $REC/$SCENE.wav (scene must already be recalled on the WING)"
sox -q -t coreaudio "$DEV" -r 48000 -b 24 -c 2 "$REC/$SCENE.wav" trim 0 "$DUR" &
RECPID=$!
sleep 1.5
sox -q "$KIT/suite.wav" -t coreaudio "$DEV" remix 1 1 rate 48000
wait "$RECPID"
echo "done: $REC/$SCENE.wav"
