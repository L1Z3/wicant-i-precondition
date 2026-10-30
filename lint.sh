#!/usr/bin/env bash
# Run clang-tidy via idf.py clang-check (ESP-IDF's pyclang runner).
#
# This uses the clang toolchain (IDF_TOOLCHAIN=clang) and Espressif's
# esp-clang, which produces a clang-compatible compilation database directly.
#
# Needs IDF env (source esp-idf/export.sh) and esp-clang installed
# (idf_tools.py install esp-clang). The build dir is reconfigured with the
# clang toolchain on first run; subsequent runs reuse it.
#
# Usage: ./lint.sh [v300|custom]      (default: custom)
# The variants share all sources, so linting one variant is enough.

set -euo pipefail
cd "$(dirname "$0")"

variant=${1:-custom}
build_dir="build.$variant"
clang_build_dir="build.${variant}.clang"

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
export LINT_SKIP

# Parallelism hint for run-clang-tidy.py (pyclang uses os.cpu_count() by
# default, but we allow override for reproducible CI or low-mem hosts).
LINT_JOBS=${LINT_JOBS:-$(nproc)}

if [ -z "${IDF_PATH:-}" ]; then
    . ./esp-idf/export.sh
fi
# Ensure esp-clang bin is in PATH (export.sh may not add it until
# esp-clang is installed, and the toolchain needs 'clang' on PATH)
for d in "$HOME/.espressif/tools/esp-clang"/*/esp-clang/bin; do
    [ -d "$d" ] && export PATH="$d:$PATH"
done

# Ensure esp-clang is available (required for IDF_TOOLCHAIN=clang)
if [ ! -d "$HOME/.espressif/tools/esp-clang" ]; then
    echo "=== esp-clang not found; installing via idf_tools.py ==="
    ./esp-idf/tools/idf_tools.py install esp-clang
    for d in "$HOME/.espressif/tools/esp-clang"/*/esp-clang/bin; do
        [ -d "$d" ] && export PATH="$d:$PATH"
    done
    . ./esp-idf/export.sh
    for d in "$HOME/.espressif/tools/esp-clang"/*/esp-clang/bin; do
        [ -d "$d" ] && export PATH="$d:$PATH"
    done
fi

# Ensure the clang build dir exists and is configured for the variant.
# We use a separate build dir (build.$variant.clang) with IDF_TOOLCHAIN=clang
# so the compilation database is clang-native and needs no sanitization, and
# we don't clobber the gcc build in build.$variant. Reports are still written
# to build.$variant/tidy for CI compatibility.
if [ ! -f "$clang_build_dir/compile_commands.json" ]; then
    echo "=== no compile database; configuring $clang_build_dir with clang toolchain ==="
    IDF_TOOLCHAIN=clang idf.py -B "$clang_build_dir" -DHARDWARE_VER_NAME="$variant" reconfigure
fi

mkdir -p "$build_dir/tidy"
mkdir -p "$clang_build_dir"
# Clean previous pyclang output (it writes warnings.txt to project root)
rm -f warnings.txt
rm -f "$build_dir/tidy/warnings.txt"

# Build the --check-files-regex and --exclude-paths handling.
# pyclang's --exclude-paths is directory-based (checks parents), so per-file
# globs like main/autopid.c would not be excluded. Instead we let pyclang run
# on the project files and post-filter warnings.txt by LINT_SKIP globs, which
# preserves exact per-file semantics and keeps the cache-free path simple.
# For efficiency, we also pass a --check-files-regex that limits to the kept
# files, so run-clang-tidy doesn't waste time on skipped TUs. If all files
# are skipped, skip the clang-check entirely.

# Compute kept vs skipped using the current compile DB (like the old script)
declare -a kept_files=() skipped_files=()
# Use the compile DB's file list (already filtered to main/ + components/ by pyclang's filter,
# but we do our own per-file glob filtering for LINT_SKIP)
while IFS=$'\t' read -r tag path; do
    case $tag in
        KEEP) kept_files+=("$path") ;;
        *) skipped_files+=("$path") ;;
    esac
done < <(python3 -c "
import json, os
from fnmatch import fnmatch
import pathlib
db_path = '$clang_build_dir/compile_commands.json'
try:
    db = json.load(open(db_path))
except FileNotFoundError:
    db = []
root = os.getcwd()
patterns = os.environ.get('LINT_SKIP', '').split()
def excluded(path):
    return any(fnmatch(p, pat) for pat in patterns for p in (path, os.path.relpath(path, root)))
# Keep only entries that are in the project (main/ or components/) and not .S
for e in db:
    f = e.get('file', '')
    if not f.endswith('.c'):
        continue
    # pyclang already filters to project files, but be permissive here
    rel = os.path.relpath(f, root)
    if not (rel.startswith('main/') or rel.startswith('components/')):
        continue
    print('%s\t%s' % ('SKIP' if excluded(f) else 'KEEP', f))
" 2>&1)

if ((${#skipped_files[@]})); then
    echo "=== skipping ${#skipped_files[@]} file(s) per LINT_SKIP ==="
    printf '  %s\n' "${skipped_files[@]}"
fi

if ((${#kept_files[@]} == 0)); then
    echo "=== nothing left to lint after LINT_SKIP filtering ==="
    : > "$build_dir/tidy/report.txt"
    : > "$build_dir/tidy/warnings.txt"
    exit 0
fi

echo "=== idf.py clang-check on ${#kept_files[@]} file(s) (variant $variant, $LINT_JOBS jobs, build $clang_build_dir) ==="
# pyclang handles parallelism via run-clang-tidy.py -j; we pass -j via
# --run-clang-tidy-options. We limit the check to kept files by passing a
# single regex as the positional pattern (which becomes check_files_regex in
# pyclang). The regex is an OR of the kept files' relative paths.
# We do NOT use --exit-code so we can post-filter warnings.txt and decide
# the exit code based on the filtered report (like the old script).
# Convert kept_files (absolute) to project-relative and build a regex.
kept_regex=$(python3 -c "
import re, os
kept = '''${kept_files[*]}'''.split()
parts = [re.escape(os.path.relpath(p, os.getcwd())) for p in kept]
print('|'.join(parts))
")
if [ -z "$kept_regex" ]; then
    kept_regex=".*"
fi
set +e
IDF_TOOLCHAIN=clang idf.py -B "$clang_build_dir" clang-check \
    --run-clang-tidy-options "-j $LINT_JOBS" "$kept_regex" 2>&1 | tee "$build_dir/tidy/clang-check.log"
clang_check_rc=${PIPESTATUS[0]}
set -e

# pyclang writes warnings.txt to the project root (or output_path). Handle both.
if [ -f "warnings.txt" ]; then
    mv warnings.txt "$build_dir/tidy/warnings.txt"
elif [ -f "$build_dir/tidy/warnings.txt" ]; then
    : # already there
else
    # No warnings file generated (e.g., no files matched)
    : > "$build_dir/tidy/warnings.txt"
fi

# Post-filter warnings.txt by LINT_SKIP to produce report.txt
# This is a safety net: even though we limited via --check-files-regex,
# pyclang may still have run on extra files (e.g., headers). Filter again.
python3 -c "
import os, re
from fnmatch import fnmatch
root = os.getcwd()
patterns = os.environ.get('LINT_SKIP', '').split()
def excluded(path):
    return any(fnmatch(p, pat) for pat in patterns for p in (path, os.path.relpath(path, root)))
# warnings.txt lines are like: path:line:col: severity: msg [check]
# We filter out any warning whose path matches LINT_SKIP
import pathlib
inp = open('$build_dir/tidy/warnings.txt').read().splitlines() if os.path.exists('$build_dir/tidy/warnings.txt') else []
out = []
for line in inp:
    m = re.match(r'^([\w/.\- ]+):\d+:\d+:', line)
    if m:
        path = m.group(1).strip()
        # pyclang normalizes to relative paths
        abs_path = os.path.join(root, path) if not os.path.isabs(path) else path
        if excluded(abs_path) or excluded(path):
            continue
    out.append(line)
open('$build_dir/tidy/report.txt', 'w').write('\n'.join(out) + ('\n' if out else ''))
" || true

# Show filtered warnings
grep -E 'warning:|error:' "$build_dir/tidy/report.txt" || true

if grep -qE 'warning:|error:' "$build_dir/tidy/report.txt"; then
    echo
    echo "clang-tidy found issues; full output in $build_dir/tidy/report.txt (raw: $build_dir/tidy/warnings.txt, log: $build_dir/tidy/clang-check.log)" >&2
    exit 1
fi
echo "=== no clang-tidy findings ==="
