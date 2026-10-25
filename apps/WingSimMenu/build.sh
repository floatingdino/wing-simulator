#!/bin/sh
# Build WingSimMenu.app into apps/WingSimMenu/build/ (T064).
#
#   apps/WingSimMenu/build.sh            # release build
#   CONFIG=debug apps/WingSimMenu/build.sh
#
# `make dist` sets WINGSIM_REPO_ROOT="" so a distributed app never points at the build checkout.
# target/release/wing-sim, wing-render and wing-diff are copied into Contents/Resources/bin when they exist
# (`make build` or `cargo build --release -p wing-sim -p wing-diff`); the app falls back to
# <repo>/target/release (Info.plist WingSimRepoRoot) and $WINGSIM_BIN_DIR.
set -eu
HERE=$(cd "$(dirname "$0")" && pwd)
ROOT=$(cd "$HERE/../.." && pwd)
CONFIG=${CONFIG:-release}
VERSION=$(sed -n 's/^version = "\(.*\)"/\1/p' "$ROOT/Cargo.toml" | head -n 1)

cd "$HERE"
swift build -c "$CONFIG"
BIN=$(swift build -c "$CONFIG" --show-bin-path)

APP="$HERE/build/WingSimMenu.app"
rm -rf "$APP"
mkdir -p "$APP/Contents/MacOS" "$APP/Contents/Resources/bin"
cp "$BIN/WingSimMenu" "$APP/Contents/MacOS/WingSimMenu"
sed -e "s|@REPO_ROOT@|${WINGSIM_REPO_ROOT-$ROOT}|" -e "s|@VERSION@|${VERSION:-0.1.0}|" "$HERE/Info.plist.in" \
    > "$APP/Contents/Info.plist"
for tool in wing-sim wing-render wing-diff; do
    if [ -x "$ROOT/target/release/$tool" ]; then
        cp "$ROOT/target/release/$tool" "$APP/Contents/Resources/bin/$tool"
    else
        echo "build.sh: note: $ROOT/target/release/$tool not built; the app will look in target/release at run time" >&2
    fi
done
codesign --force --deep --sign - "$APP" >/dev/null 2>&1 || echo "build.sh: note: ad-hoc codesign failed" >&2
echo "built $APP"
