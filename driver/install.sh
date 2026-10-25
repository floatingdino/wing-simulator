#!/bin/bash
# Install the Wing Simulator HAL driver. REQUIRES ROOT - run by a human:
#   sudo driver/install.sh
# Copies driver/build/WingSimulator.driver into /Library/Audio/Plug-Ins/HAL and
# restarts coreaudiod (all audio apps lose their devices for a moment; quit
# REAPER first).
set -euo pipefail

here="$(cd "$(dirname "$0")" && pwd)"
src="$here/build/WingSimulator.driver"
dst_dir="/Library/Audio/Plug-Ins/HAL"
dst="$dst_dir/WingSimulator.driver"

if [ "${EUID}" -ne 0 ]; then
    echo "must run as root: sudo $0" >&2
    exit 1
fi
if [ ! -d "$src" ]; then
    echo "not built: run driver/build.sh first (as your user, not root)" >&2
    exit 1
fi

echo "-- installing $src -> $dst"
rm -rf "$dst"
cp -R "$src" "$dst_dir/"
chown -R root:wheel "$dst"
chmod -R go-w "$dst"

echo "-- restarting coreaudiod"
launchctl kickstart -k system/com.apple.audio.coreaudiod 2>/dev/null || killall coreaudiod

echo "-- done. Verify with: driver/build/wing_hal_probe --info"
