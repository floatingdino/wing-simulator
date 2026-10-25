#!/bin/bash
# Build the Wing Simulator HAL driver and host-side tools (no root needed).
#   driver/build/WingSimulator.driver   - the AudioServerPlugIn bundle
#   driver/build/wing_shm_client        - daemon-side test client
#   driver/build/wing_shm_selftest      - transport self test (run via ctest)
#   driver/build/wing_driver_smoketest  - loads the bundle with a fake HAL host
#   driver/build/wing_hal_probe         - post-install CoreAudio e2e probe
# Usage: driver/build.sh [--test] [extra cmake -D args...]
set -euo pipefail

here="$(cd "$(dirname "$0")" && pwd)"
build="$here/build"

if ! command -v cmake >/dev/null 2>&1; then
    echo "cmake not found: brew install cmake" >&2
    exit 1
fi

run_tests=0
args=()
for a in "$@"; do
    if [ "$a" = "--test" ]; then run_tests=1; else args+=("$a"); fi
done

# -Wno-dev: silences CMake 4 policy warnings from libASPL's own CMakeLists.
cmake -Wno-dev -S "$here" -B "$build" -DCMAKE_BUILD_TYPE=Release ${args[@]+"${args[@]}"}
cmake --build "$build" --parallel

echo "-- built $build/WingSimulator.driver"
codesign --verify --verbose=1 "$build/WingSimulator.driver"
lipo -info "$build/WingSimulator.driver/Contents/MacOS/WingSimulator"

if [ "$run_tests" = 1 ]; then
    (cd "$build" && ctest --output-on-failure)
fi
