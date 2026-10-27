#!/bin/bash
# Wing Simulator installer (shipped inside dist/WingSimulator-<version>/).
#
#   ./install.sh [--no-driver] [--driver-only] [--no-cli] [--app-dir DIR] [--bin-dir DIR] [--dry-run]
#
# Everything except the audio driver installs into your home folder and needs no privileges:
#   WingSimMenu.app -> ~/Applications        (the menu-bar app; it carries its own copy of the daemon)
#   wing-sim, wing-render, wing-diff, wing-record-setup -> ~/.local/bin   (command-line copies)
#   DAW scripts and the recording kit (integrations/) -> left in place; see docs/user/
# The audio driver is a CoreAudio plug-in and macOS only loads those from
#   /Library/Audio/Plug-Ins/HAL
# which is root-owned. This script never escalates silently: it tells you exactly what it is about to
# run with sudo, asks, and you type your password into sudo itself (not into this script). Say no and
# you get the commands to run later yourself. Installing the driver restarts coreaudiod, so all
# audio apps (your DAW, Music, browsers) lose their devices for a second or two: quit them first.
set -euo pipefail

here="$(cd "$(dirname "$0")" && pwd)"
app_dir="$HOME/Applications"
bin_dir="$HOME/.local/bin"
do_driver=1; do_user=1; do_cli=1; dry=0

while [ $# -gt 0 ]; do
    case "$1" in
        --no-driver) do_driver=0 ;;
        --driver-only) do_user=0 ;;
        --no-cli) do_cli=0 ;;
        --app-dir) app_dir="$2"; shift ;;
        --bin-dir) bin_dir="$2"; shift ;;
        --dry-run) dry=1 ;;
        -h|--help) sed -n '2,15p' "$0"; exit 0 ;;
        *) echo "unknown option: $1 (see --help)" >&2; exit 2 ;;
    esac
    shift
done

run() { if [ "$dry" = 1 ]; then echo "   [dry-run] $*"; else "$@"; fi; }
# ask "question": 0 for yes; the default is no, and without a terminal the answer is no.
ask() {
    local a
    [ -t 0 ] || return 1
    read -r -p "$1 [y/N] " a || return 1
    case "$a" in y|Y|yes|YES) return 0 ;; *) return 1 ;; esac
}

if [ "$(uname -s)" != Darwin ]; then echo "macOS only" >&2; exit 1; fi
if [ "${EUID}" -eq 0 ]; then
    echo "Do not run this as root: it installs into your home folder and asks for sudo only for the driver." >&2
    exit 1
fi
if [ ! -d "$here/WingSimMenu.app" ] || [ ! -d "$here/WingSimulator.driver" ]; then
    echo "run this from inside the unpacked WingSimulator-<version> folder" >&2
    exit 1
fi
if [ "$(uname -m)" != arm64 ]; then
    echo "note: these binaries are built for Apple silicon (arm64); this Mac reports $(uname -m)." >&2
fi

echo "Wing Simulator $(cat "$here/VERSION" 2>/dev/null || echo '?') installer"
echo

if [ "$do_user" = 1 ]; then
    echo "== 1. Menu-bar app and tools (no privileges needed)"
    echo "   WingSimMenu.app -> $app_dir"
    run mkdir -p "$app_dir"
    run rm -rf "$app_dir/WingSimMenu.app"
    run cp -R "$here/WingSimMenu.app" "$app_dir/"
    # A downloaded zip is quarantined; the build is ad-hoc signed (not notarised), so Gatekeeper
    # would refuse the first launch. Clearing the flag on our own copy is what right-click > Open does.
    run xattr -dr com.apple.quarantine "$app_dir/WingSimMenu.app" 2>/dev/null || true
    if [ "$do_cli" = 1 ]; then
        echo "   wing-sim wing-render wing-diff wing-record-setup -> $bin_dir"
        run mkdir -p "$bin_dir"
        for t in wing-sim wing-render wing-diff wing-record-setup; do
            run cp "$here/bin/$t" "$bin_dir/$t"
            run xattr -dr com.apple.quarantine "$bin_dir/$t" 2>/dev/null || true
        done
        case ":$PATH:" in
            *":$bin_dir:"*) ;;
            *) echo "   note: $bin_dir is not on your PATH (add it to ~/.zshrc to use the commands in Terminal)" ;;
        esac
    fi
    echo
fi

if [ "$do_driver" = 1 ]; then
    echo "== 2. Audio driver (needs administrator rights)"
    cat <<MSG
   The 'Wing Simulator' 48x48 audio device is a CoreAudio plug-in. macOS only loads those from
   /Library/Audio/Plug-Ins/HAL, which is owned by root. To install it I would run, with sudo:

       sudo rm -rf /Library/Audio/Plug-Ins/HAL/WingSimulator.driver
       sudo cp -R "$here/WingSimulator.driver" /Library/Audio/Plug-Ins/HAL/
       sudo chown -R root:wheel /Library/Audio/Plug-Ins/HAL/WingSimulator.driver
       sudo launchctl kickstart -k system/com.apple.audio.coreaudiod     # restarts macOS audio

   Restarting coreaudiod interrupts all audio on this Mac for a moment: quit your DAW and other
   audio apps first. The driver is ad-hoc signed (no Apple developer identity). Uninstall:
   ./uninstall.sh
MSG
    if ask "   Install the driver now (sudo will ask for your password)?"; then
        if [ "$dry" = 1 ]; then
            echo "   [dry-run] would run the sudo commands above"
        else
            dst=/Library/Audio/Plug-Ins/HAL
            sudo rm -rf "$dst/WingSimulator.driver"
            sudo cp -R "$here/WingSimulator.driver" "$dst/"
            sudo chown -R root:wheel "$dst/WingSimulator.driver"
            sudo chmod -R go-w "$dst/WingSimulator.driver"
            echo "   restarting coreaudiod"
            sudo launchctl kickstart -k system/com.apple.audio.coreaudiod 2>/dev/null || sudo killall coreaudiod
            echo "   driver installed"
        fi
    else
        echo "   Skipped. When you are ready, run ./install.sh --driver-only or the commands above yourself."
    fi
    echo
fi

cat <<MSG
Next steps
  1. Open WingSimMenu (in $app_dir, or via Spotlight). If macOS says it cannot verify the developer,
     right-click the app > Open once (the build is ad-hoc signed, not notarised).
  2. Choose Start in the menu-bar app, then follow the first-run walk-through in
     ${WINGSIM_DOCS_DIR:-$here}/README.md
  3. Check the install: ${WINGSIM_DOCS_DIR:-$here}/docs/user/install-checklist.md
MSG
