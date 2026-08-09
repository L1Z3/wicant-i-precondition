#!/usr/bin/env bash
# Run clang-tidy on this repo's own sources (main/ + components/).
#
# clang-tidy analyzes against a *sanitized* copy of the build's compile
# database: the raw one records xtensa/riscv-elf-gcc invocations that clang
# cannot parse (unknown target triple, target-only flags, glibc header
# pollution). tools/clang-tidy/clean-compdb.py rewrites it into something
# clang-tidy can consume. See that script and .clang-tidy for the rationale.
#
# Needs clang-tidy on PATH and, only when build.<variant> has no compile
# database yet, an exported IDF environment (source esp-idf/export.sh).
#
# Usage: ./lint.sh [v300|custom]      (default: custom)
# The variants share all sources, so linting one variant is enough.

set -euo pipefail
cd "$(dirname "$0")"

variant=${1:-custom}
build_dir="build.$variant"

# Files clang-tidy must not analyze: whitespace-separated glob patterns matched
# against repo-relative paths (absolute paths match too). Skip everything for a
# single run with e.g.:  LINT_SKIP="main/*" ./lint.sh custom
# CI runs plain ./lint.sh custom, so anything that must be skipped there has to
# stay in LINT_SKIP_DEFAULT below.
#
# Default: every *.c/*.h under main/ this fork never touched (no changes since
# upstream commit e0b26b05), plus a few fork-touched but upstream-heavy files
# skipped deliberately to keep the lint gate focused on fork code (ble.c,
# gvret.c, slcan.c, elm327.c, mqtt.c, wifi_network.c). Regenerate the
# untouched part with:
#   comm -23 \
#     <(ls main/*.c main/*.h | xargs -n1 basename | sort) \
#     <(git diff --name-only e0b26b05 HEAD -- 'main/*.c' 'main/*.h' | xargs -n1 basename | sort) \
#     | sed 's|^|main/|'
LINT_SKIP_DEFAULT="main/autopid.c main/autopid.h main/ble.h main/comm_server.h main/dev_status.c main/dev_status.h main/elm327.h main/expression_parser.c main/expression_parser.h main/ftp.c main/ftp.h main/hw_config.c main/mqtt.h main/obd2_standard_pids.h main/realdash.c main/realdash.h main/slcan.h main/sleep_mode.h main/std_pid.h main/types.h main/wc_mdns.c main/wc_mdns.h main/wc_timer.c main/wc_timer.h main/wc_uart.c main/wc_uart.h main/wifi_network.h main/ble.c main/gvret.c main/slcan.c main/elm327.c main/mqtt.c main/wifi_network.c"
LINT_SKIP=${LINT_SKIP:-$LINT_SKIP_DEFAULT}

if ! command -v clang-tidy >/dev/null 2>&1; then
    echo "error: clang-tidy not found on PATH" >&2
    exit 1
fi

if [ ! -f "$build_dir/compile_commands.json" ]; then
    if [ -z "${IDF_PATH:-}" ]; then
        echo "error: $build_dir/compile_commands.json missing and IDF_PATH unset; source esp-idf/export.sh first" >&2
        exit 1
    fi
    echo "=== no compile database; generating one in $build_dir ==="
    idf.py -B "$build_dir" -DHARDWARE_VER_NAME="$variant" reconfigure
fi

mkdir -p "$build_dir/tidy"
python3 tools/clang-tidy/clean-compdb.py "$build_dir/compile_commands.json" "$build_dir/tidy/compile_commands.json"

declare -a files=() skipped=()
export LINT_SKIP
while IFS=$'\t' read -r tag path; do
    case $tag in
        KEEP) files+=("$path") ;;
        *) skipped+=("$path") ;;
    esac
done < <(python3 -c "
import json, os
from fnmatch import fnmatch

db = json.load(open('$build_dir/tidy/compile_commands.json'))
root = os.getcwd()
patterns = os.environ['LINT_SKIP'].split()

def excluded(path):
    return any(fnmatch(p, pat) for pat in patterns for p in (path, os.path.relpath(path, root)))

for entry in db:
    path = entry['file']
    print('%s\t%s' % ('SKIP' if excluded(path) else 'KEEP', path))
")

if ((${#skipped[@]})); then
    echo "=== skipping ${#skipped[@]} file(s) per LINT_SKIP ==="
    printf '  %s\n' "${skipped[@]}"
fi

if ((${#files[@]} == 0)); then
    echo "=== nothing left to lint after LINT_SKIP filtering ==="
    : > "$build_dir/tidy/report.txt"
    exit 0
fi

echo "=== clang-tidy on ${#files[@]} files ==="
clang-tidy --quiet -p "$build_dir/tidy" "${files[@]}" 2>&1 | tee "$build_dir/tidy/report.txt" | grep -E 'warning:|error:' || true

if grep -qE 'warning:|error:' "$build_dir/tidy/report.txt"; then
    echo
    echo "clang-tidy found issues; full output in $build_dir/tidy/report.txt" >&2
    exit 1
fi
echo "=== no clang-tidy findings ==="
