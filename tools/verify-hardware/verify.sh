#!/bin/sh
# Hardware verification kit driver (T072). Builds wing-verify on first use and forwards the arguments.
#
#   verify.sh gen     <kit-dir> [quick|all]
#   verify.sh compare <kit-dir> <recordings-dir> [report-dir] [--update-doc]
#   verify.sh warm-gen <kit-dir> [quick|all]                 (WARM calibration kit, T090)
#   verify.sh fit-warm <recordings-dir> [report-dir]        (fits + writes data/calibration/warm.json)
#
# See tools/verify-hardware/README.md.
set -eu
ROOT=$(cd "$(dirname "$0")/../.." && pwd)
cd "$ROOT"
cargo build --release -q -p wing-render --bin wing-verify
BIN="$ROOT/target/release/wing-verify"
cmd=${1:?usage: verify.sh gen|compare|warm-gen|fit-warm ...}
shift
case "$cmd" in
  gen)
    kit=${1:?kit dir}
    exec "$BIN" gen --out "$kit" --tier "${2:-quick}"
    ;;
  compare)
    kit=${1:?kit dir}
    rec=${2:?recordings dir}
    out=${3:-verify-report}
    extra=""
    if [ "${4:-}" = "--update-doc" ] || [ "$out" = "--update-doc" ]; then
      [ "$out" = "--update-doc" ] && out=verify-report
      extra="--update-doc $ROOT/docs/VERIFY-ON-HARDWARE.md"
    fi
    # shellcheck disable=SC2086
    exec "$BIN" compare --kit "$kit" --recordings "$rec" --out "$out" $extra
    ;;
  warm-gen)
    kit=${1:?kit dir}
    exec "$BIN" warm-gen --out "$kit" --tier "${2:-quick}"
    ;;
  fit-warm)
    rec=${1:?recordings dir}
    exec "$BIN" fit-warm "$rec" --out "${2:-warm-report}" --write "$ROOT/data/calibration/warm.json"
    ;;
  *)
    echo "usage: verify.sh gen|compare|warm-gen|fit-warm ..." >&2
    exit 2
    ;;
esac
