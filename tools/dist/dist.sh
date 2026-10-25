#!/bin/bash
# Assemble dist/WingSimulator-<version>/ (T073). Run via `make dist`.
#
# Needs: Rust toolchain, Xcode command-line tools (swift, codesign), cmake (driver; libASPL is fetched
# by CMake on the first driver build, so that needs network once). Never uses sudo, never installs
# anything, never touches a running daemon. Signing is ad-hoc only (`codesign -s -`): there is no Apple
# Developer ID, so nothing is notarised (see docs/user/install-checklist.md, "Distribution status").
set -euo pipefail

root="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$root"
version="$(sed -n 's/^version = "\(.*\)"/\1/p' Cargo.toml | head -n 1)"
out="$root/dist/WingSimulator-$version"

echo "== release binaries"
cargo build --release -p wing-sim -p wing-render -p wing-diff

echo "== driver"
if [ "${DIST_REBUILD_DRIVER:-0}" = 1 ] || [ ! -d driver/build/WingSimulator.driver ]; then
    driver/build.sh
else
    echo "   reusing driver/build/WingSimulator.driver (DIST_REBUILD_DRIVER=1 to rebuild)"
fi

echo "== menu-bar app (bundles wing-sim, wing-render, wing-diff in Contents/Resources/bin)"
WINGSIM_REPO_ROOT="" apps/WingSimMenu/build.sh

echo "== assembling $out"
rm -rf "$out"
mkdir -p "$out/bin" "$out/docs" "$out/integrations"
for t in wing-sim wing-render wing-diff; do
    cp "target/release/$t" "$out/bin/$t"
    codesign --force --sign - "$out/bin/$t"
done
cp -R driver/build/WingSimulator.driver "$out/WingSimulator.driver"
codesign --force --deep --sign - "$out/WingSimulator.driver"
cp -R apps/WingSimMenu/build/WingSimMenu.app "$out/WingSimMenu.app"
codesign --force --deep --sign - "$out/WingSimMenu.app"
cp -R integrations/reaper "$out/integrations/reaper"
cp -R docs/user "$out/docs/user"
cp README.md "$out/README.md"
cp tools/dist/install.sh tools/dist/uninstall.sh "$out/"
chmod +x "$out/install.sh" "$out/uninstall.sh"
echo "$version" > "$out/VERSION"

echo "== verifying"
for b in "$out"/bin/* "$out/WingSimulator.driver" "$out/WingSimMenu.app"; do
    codesign --verify --deep --strict "$b"
done
for t in wing-sim wing-render wing-diff; do
    test -x "$out/WingSimMenu.app/Contents/Resources/bin/$t" || { echo "missing $t inside the app" >&2; exit 1; }
done
bash -n "$out/install.sh"
bash -n "$out/uninstall.sh"
(cd "$out" && find . -type f ! -name SHA256SUMS -print0 | sort -z | xargs -0 shasum -a 256 > SHA256SUMS)

echo "== done: $out"
du -sh "$out"
echo "Zip for sharing: (cd dist && ditto -c -k --keepParent WingSimulator-$version WingSimulator-$version.zip)"
