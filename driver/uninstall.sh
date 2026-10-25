#!/bin/bash
# Remove the Wing Simulator HAL driver. REQUIRES ROOT - run by a human:
#   sudo driver/uninstall.sh
set -euo pipefail

dst="/Library/Audio/Plug-Ins/HAL/WingSimulator.driver"

if [ "${EUID}" -ne 0 ]; then
    echo "must run as root: sudo $0" >&2
    exit 1
fi

if [ -d "$dst" ]; then
    echo "-- removing $dst"
    rm -rf "$dst"
else
    echo "-- $dst not installed"
fi

echo "-- restarting coreaudiod"
launchctl kickstart -k system/com.apple.audio.coreaudiod 2>/dev/null || killall coreaudiod

# The shm segment (/wingsim.audio.v1, ~3 MiB) persists until reboot; it is
# harmless and reused by a reinstalled driver of the same layout version.
echo "-- done"
