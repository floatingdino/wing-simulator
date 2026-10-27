#!/bin/bash
# Wing Simulator uninstaller (shipped inside dist/WingSimulator-<version>/).
#
#   ./uninstall.sh [--keep-driver] [--purge-state] [--app-dir DIR] [--bin-dir DIR] [--dry-run]
#
# Removes what install.sh installed. The driver removal needs sudo; the script shows the commands
# and asks first. Your scenes and settings in ~/Library/Application Support/WingSimulator are kept
# unless you pass --purge-state (then it asks again and moves them to the Trash, not deleting them).
set -euo pipefail

app_dir="$HOME/Applications"
bin_dir="$HOME/.local/bin"
keep_driver=0; purge=0; dry=0
while [ $# -gt 0 ]; do
    case "$1" in
        --keep-driver) keep_driver=1 ;;
        --purge-state) purge=1 ;;
        --app-dir) app_dir="$2"; shift ;;
        --bin-dir) bin_dir="$2"; shift ;;
        --dry-run) dry=1 ;;
        -h|--help) sed -n '2,9p' "$0"; exit 0 ;;
        *) echo "unknown option: $1" >&2; exit 2 ;;
    esac
    shift
done
run() { if [ "$dry" = 1 ]; then echo "   [dry-run] $*"; else "$@"; fi; }
ask() {
    local a
    [ -t 0 ] || return 1
    read -r -p "$1 [y/N] " a || return 1
    case "$a" in y|Y|yes|YES) return 0 ;; *) return 1 ;; esac
}

if [ "${EUID}" -eq 0 ]; then
    echo "Do not run this as root; it asks for sudo only for the driver." >&2
    exit 1
fi

state="${WINGSIM_STATE_DIR:-$HOME/Library/Application Support/WingSimulator}"
if pgrep -x wing-sim >/dev/null 2>&1; then
    echo "-- wing-sim is running."
    if ask "   Ask it to quit now (control socket 'quit')?"; then
        if [ "$dry" = 0 ]; then
            (echo quit | nc -U "$state/control.sock" >/dev/null 2>&1) || echo "   could not reach it; quit it from the menu-bar app"
        fi
    fi
fi

echo "-- removing app and tools"
run rm -rf "$app_dir/WingSimMenu.app"
for t in wing-sim wing-render wing-diff wing-record-setup; do run rm -f "$bin_dir/$t"; done

dst=/Library/Audio/Plug-Ins/HAL/WingSimulator.driver
if [ "$keep_driver" = 0 ] && [ -d "$dst" ]; then
    echo "-- the audio driver is installed at $dst"
    echo "   Removing it needs sudo and restarts coreaudiod (quit audio apps first):"
    echo "       sudo rm -rf $dst"
    echo "       sudo launchctl kickstart -k system/com.apple.audio.coreaudiod"
    if ask "   Remove the driver now?"; then
        if [ "$dry" = 1 ]; then
            echo "   [dry-run] would run the sudo commands"
        else
            sudo rm -rf "$dst"
            sudo launchctl kickstart -k system/com.apple.audio.coreaudiod 2>/dev/null || sudo killall coreaudiod
        fi
    else
        echo "   Left in place."
    fi
fi

if [ "$purge" = 1 ] && [ -d "$state" ]; then
    if ask "-- move $state (scenes, routing, logs) to the Trash?"; then
        run osascript -e "tell application \"Finder\" to delete POSIX file \"$state\"" >/dev/null
    fi
fi
echo "-- done (the shared-memory segment /wingsim.audio.v1 disappears at reboot; it is harmless)"
