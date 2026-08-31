#!/bin/sh
# ===========================================================================
#  build.sh <target> [asan] [args...]        Linux / macOS
#
#  The POSIX twin of build.bat. Compiles every src/*.c plus the named app
#  (from apps/, or tests/ for test_all) and runs it.
#
#      ./build.sh test_all
#      ./build.sh ch_probe 30
#      ./build.sh link_sim 20 120
#      ./build.sh test_all asan
#
#  No dependencies beyond a C17 compiler and libm.
# ===========================================================================
set -e

ROOT=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
CC=${CC:-cc}

if [ $# -lt 1 ]; then
    echo "usage: ./build.sh <target> [asan] [args...]"
    echo "targets:"
    for f in "$ROOT"/apps/*.c "$ROOT"/tests/*.c; do
        [ -e "$f" ] && echo "  $(basename "$f" .c)"
    done
    exit 1
fi

APP=$1
shift

SRC_MAIN="$ROOT/apps/$APP.c"
[ -f "$SRC_MAIN" ] || SRC_MAIN="$ROOT/tests/$APP.c"
if [ ! -f "$SRC_MAIN" ]; then
    echo "error: no such target: $APP (looked in apps/ and tests/)"
    exit 1
fi

FLAGS="-std=c17 -O2 -Wall -Wextra -Wpedantic"
if [ "$1" = "asan" ]; then
    shift
    FLAGS="-std=c17 -O1 -g -fno-omit-frame-pointer -fsanitize=address,undefined"
fi

mkdir -p "$ROOT/out"
# shellcheck disable=SC2086
$CC $FLAGS -I"$ROOT/include" "$ROOT"/src/*.c "$SRC_MAIN" -o "$ROOT/out/$APP" -lm

echo "=== run ==="
"$ROOT/out/$APP" "$@"
