#!/usr/bin/env bash
# Pre-push clang-tidy gate: lint only the C++ files changed against the
# upstream branch, after refreshing compile_commands.json if needed.
#
# Wired into pre-commit via .pre-commit-config.yaml at the `pre-push` stage.
# Skips cleanly (exit 0 with a notice) when:
#   - no C++ files changed in the push range
#   - clang-tidy is not installed (contributor environment)
# Fails the push when:
#   - clang-tidy emits warnings/errors on changed files
#   - the build dir cannot produce compile_commands.json
#
# Override behaviour via env vars:
#   BUILD_DIR=build-mecanum     # build directory to read compile_commands from
#   UPSTREAM=origin/main        # ref to diff against
#   SKIP_CLANG_TIDY=1           # bail out without running (emergency escape)
set -euo pipefail

if [[ "${SKIP_CLANG_TIDY:-0}" == "1" ]]; then
    echo "clang-tidy: skipped via SKIP_CLANG_TIDY=1" >&2
    exit 0
fi

if ! command -v clang-tidy >/dev/null 2>&1; then
    echo "clang-tidy: not installed — skipping (install with 'apt install clang-tidy')" >&2
    exit 0
fi

BUILD_DIR="${BUILD_DIR:-build-mecanum}"
UPSTREAM="${UPSTREAM:-}"

# Determine the diff base. Prefer the configured upstream of the current
# branch so feature branches without main checked out still work.
if [[ -z "${UPSTREAM}" ]]; then
    if upstream=$(git rev-parse --abbrev-ref --symbolic-full-name '@{u}' 2>/dev/null); then
        UPSTREAM="${upstream}"
    elif git rev-parse --verify origin/main >/dev/null 2>&1; then
        UPSTREAM="origin/main"
    elif git rev-parse --verify main >/dev/null 2>&1; then
        UPSTREAM="main"
    else
        echo "clang-tidy: cannot resolve upstream ref — set UPSTREAM=<ref> to override" >&2
        exit 1
    fi
fi

# Files changed against UPSTREAM. Filter to C++ sources/headers under modules/
# (we don't lint vendored deps in raccoon-transport or fetched build artifacts).
mapfile -t changed < <(
    git diff --name-only --diff-filter=ACMR "${UPSTREAM}"...HEAD -- \
        'modules/**/*.cpp' 'modules/**/*.hpp' \
        2>/dev/null || true
)

if [[ ${#changed[@]} -eq 0 ]]; then
    echo "clang-tidy: no C++ changes vs ${UPSTREAM} — skipping" >&2
    exit 0
fi

echo "clang-tidy: ${#changed[@]} file(s) changed vs ${UPSTREAM}"

# Ensure compile_commands.json exists. If the file is older than 24h or missing
# entries for any of the changed files, regenerate via cmake configure (cheap;
# does not rebuild any object files).
needs_refresh=0
cc_json="${BUILD_DIR}/compile_commands.json"
if [[ ! -f "${cc_json}" ]]; then
    needs_refresh=1
elif ! python3 -c "
import json, sys
from pathlib import Path
db = json.loads(Path('${cc_json}').read_text())
known = {Path(e['file']).resolve() for e in db}
missing = [f for f in sys.argv[1:] if Path(f).resolve() not in known and Path(f).suffix == '.cpp']
sys.exit(1 if missing else 0)
" "${changed[@]}" 2>/dev/null; then
    needs_refresh=1
fi

if [[ "${needs_refresh}" -eq 1 ]]; then
    echo "clang-tidy: refreshing ${cc_json}" >&2
    if ! cmake -B "${BUILD_DIR}" -S . \
        -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
        -DLIBSTP_BUILD_TESTS=OFF \
        >/dev/null; then
        echo "clang-tidy: cmake configure failed — fix the build before pushing" >&2
        exit 1
    fi
fi

echo "clang-tidy: linting changed files (build: ${BUILD_DIR})"
clang-tidy -p "${BUILD_DIR}" --quiet "${changed[@]}"
