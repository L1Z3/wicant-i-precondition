#!/usr/bin/env bash
# Build firmware variant(s): ./build.sh {v300|proto|eb-fd|all}

set -euo pipefail
cd "$(dirname "$0")"

usage() {
    echo "usage: ${0##*/} {v300|proto|eb-fd|all}" >&2
    exit 2
}
[ $# -eq 1 ] || usage

if [ -z "${IDF_PATH:-}" ]; then
    echo "error: IDF_PATH not set. source esp-idf/export.sh first" >&2
    exit 1
fi

build_one() {
    local variant=$1
    echo "=== building $variant ==="
    idf.py -B "build.$variant" -DHARDWARE_VER_NAME="$variant" build
}

case "$1" in
    v300|proto|eb-fd)
        build_one "$1"
        ;;
    all)
        build_one v300
        build_one proto
        build_one eb-fd
        echo "=== all variants built OK ==="
        ;;
    *)
        usage
        ;;
esac
