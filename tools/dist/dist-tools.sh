#!/bin/bash
# Command-line tools bundle for platforms without the audio driver (Linux, Windows; also works on
# macOS). Run from the repo root, e.g. in the release workflow:
#
#   tools/dist/dist-tools.sh linux-x86_64        -> dist/wing-sim-tools-linux-x86_64.tar.gz
#   tools/dist/dist-tools.sh windows-x86_64      -> dist/wing-sim-tools-windows-x86_64.zip
#
# Contents: bin/ (wing-sim, wing-render, wing-diff, wing-record-setup), integrations/, docs/user/,
# README.md, VERSION. WINGSIM_VERSION overrides Cargo.toml's version. No signing.
set -euo pipefail

label="${1:?usage: dist-tools.sh <platform-label>}"
root="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$root"
version="${WINGSIM_VERSION:-$(sed -n 's/^version = "\(.*\)"/\1/p' Cargo.toml | head -n 1)}"
exe=""
case "$label" in windows-*) exe=".exe" ;; esac
name="wing-sim-tools-$label"
out="dist/$name"

cargo build --release -p wing-sim -p wing-render -p wing-diff
rm -rf "$out"
mkdir -p "$out/bin" "$out/docs"
for t in wing-sim wing-render wing-diff wing-record-setup; do
    cp "target/release/$t$exe" "$out/bin/"
done
cp -R integrations "$out/integrations"
cp -R docs/user "$out/docs/user"
cp README.md "$out/README.md"
echo "$version" > "$out/VERSION"

cd dist
if [ -n "$exe" ]; then
    rm -f "$name.zip"
    if command -v 7z >/dev/null; then
        7z a -tzip -bd "$name.zip" "$name" >/dev/null
    else
        powershell -NoProfile -Command "Compress-Archive -Path '$name' -DestinationPath '$name.zip'"
    fi
    echo "dist/$name.zip"
else
    tar -czf "$name.tar.gz" "$name"
    echo "dist/$name.tar.gz"
fi
