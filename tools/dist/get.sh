#!/bin/bash
# One-line installer for a released build of Wing Simulator (macOS and Linux):
#
#   curl -fsSL https://github.com/floatingdino/wing-simulator/releases/latest/download/get.sh | bash
#
# The copy attached to a release installs that release (a frozen version); the copy in the repo
# installs the latest release. Override with WINGSIM_VERSION=v0.3.0. Arguments after `bash -s --`
# go to the bundle's install.sh on macOS (e.g. `| bash -s -- --no-driver`).
#
# macOS (Apple silicon): downloads WingSimulator-macos-arm64.zip, checks it against SHA256SUMS.txt
#   and runs its install.sh: menu-bar app -> ~/Applications, tools -> ~/.local/bin, and (only after
#   asking, with sudo) the audio driver -> /Library/Audio/Plug-Ins/HAL. Builds are ad-hoc signed,
#   not notarised; install.sh clears the download quarantine on the copies it installs.
# Linux: command-line tools only (no virtual audio device) -> ~/.local/bin, kit and docs ->
#   ~/.local/share/wing-simulator.
# Testing/mirrors: WINGSIM_BASE_URL replaces the download folder URL, WINGSIM_SHARE_DIR the kit folder.
set -euo pipefail

repo="${WINGSIM_REPO:-floatingdino/wing-simulator}"
version="${WINGSIM_VERSION:-@VERSION@}"
if [ "$version" = "@""VERSION@" ] || [ "$version" = latest ]; then
    base="https://github.com/$repo/releases/latest/download"
    version=latest
else
    base="https://github.com/$repo/releases/download/$version"
fi
base="${WINGSIM_BASE_URL:-$base}"

os="$(uname -s)"
arch="$(uname -m)"
case "$os" in
    Darwin)
        asset=WingSimulator-macos-arm64.zip
        [ "$arch" = arm64 ] || echo "note: release builds are for Apple silicon; this Mac reports $arch" >&2
        ;;
    Linux)
        case "$arch" in
            x86_64 | amd64) asset=wing-sim-tools-linux-x86_64.tar.gz ;;
            aarch64 | arm64) asset=wing-sim-tools-linux-aarch64.tar.gz ;;
            *) echo "no Linux build for $arch; build from source (see README)" >&2; exit 1 ;;
        esac
        ;;
    *)
        echo "unsupported OS $os. On Windows run in PowerShell:" >&2
        echo "  irm https://github.com/$repo/releases/latest/download/get.ps1 | iex" >&2
        exit 1
        ;;
esac

tmp="$(mktemp -d)"
trap 'rm -rf "$tmp"' EXIT
echo "Wing Simulator ($version): downloading $asset"
curl -fL --progress-bar -o "$tmp/$asset" "$base/$asset"
if curl -fsSL -o "$tmp/SHA256SUMS.txt" "$base/SHA256SUMS.txt"; then
    want="$(awk -v f="$asset" '$2 == f || $2 == "*"f {print $1}' "$tmp/SHA256SUMS.txt")"
    if command -v shasum >/dev/null; then
        got="$(shasum -a 256 "$tmp/$asset" | awk '{print $1}')"
    else
        got="$(sha256sum "$tmp/$asset" | awk '{print $1}')"
    fi
    if [ -z "$want" ] || [ "$want" != "$got" ]; then
        echo "checksum mismatch for $asset (expected '${want:-none}', got $got)" >&2
        exit 1
    fi
    echo "checksum ok"
else
    echo "warning: no SHA256SUMS.txt in this release; not verified" >&2
fi

# The recording kit, DAW scripts and docs, kept where the docs say to look for them.
copy_kit() {
    mkdir -p "$share"
    rm -rf "$share/integrations" "$share/docs"
    cp -R "$dir/integrations" "$dir/docs" "$share/"
    cp "$dir/README.md" "$share/"
    echo "recording kit, DAW scripts and docs -> $share"
}

mkdir "$tmp/x"
if [ "$os" = Darwin ]; then
    ditto -x -k "$tmp/$asset" "$tmp/x"
    dir="$(find "$tmp/x" -mindepth 1 -maxdepth 1 -type d -name 'WingSimulator-*' | head -n 1)"
    [ -n "$dir" ] || { echo "unexpected archive layout" >&2; exit 1; }
    share="${WINGSIM_SHARE_DIR:-$HOME/Library/Application Support/WingSimulator/kit}"
    copy_kit
    # install.sh asks before using sudo; give it the terminal even though this script is piped.
    export WINGSIM_DOCS_DIR="$share"
    if [ -r /dev/tty ] && (: < /dev/tty) 2>/dev/null; then
        bash "$dir/install.sh" "$@" < /dev/tty
    else
        bash "$dir/install.sh" "$@"
    fi
else
    tar -xzf "$tmp/$asset" -C "$tmp/x"
    dir="$(find "$tmp/x" -mindepth 1 -maxdepth 1 -type d | head -n 1)"
    bin="$HOME/.local/bin"
    mkdir -p "$bin"
    cp "$dir"/bin/* "$bin/"
    echo "tools -> $bin"
    case ":$PATH:" in *":$bin:"*) ;; *) echo "note: add $bin to your PATH" ;; esac
    share="${WINGSIM_SHARE_DIR:-${XDG_DATA_HOME:-$HOME/.local/share}/wing-simulator}"
    copy_kit
fi
