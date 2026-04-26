#!/usr/bin/env bash
# Run clang-tidy against the project's C++ sources using the existing
# .clang-tidy config. Requires a CMake build directory with
# CMAKE_EXPORT_COMPILE_COMMANDS=ON (the root CMakeLists.txt forces this).
#
# Usage:
#   bash scripts/run-clang-tidy.sh                 # build-mecanum (default)
#   BUILD_DIR=build bash scripts/run-clang-tidy.sh # explicit build dir
#   bash scripts/run-clang-tidy.sh path/to/file.cpp [other.cpp...]  # subset
#
# Why this isn't a pre-commit hook: clang-tidy needs compile_commands.json,
# which only exists after `cmake -B <dir>`. Running it from pre-commit when
# the build dir is missing or stale produces noisy false positives that
# train contributors to ignore the output. Run it explicitly here, then
# wire it into CI on a known-good build.

set -euo pipefail

BUILD_DIR="${BUILD_DIR:-build-mecanum}"

if [[ ! -f "${BUILD_DIR}/compile_commands.json" ]]; then
    echo "ERROR: ${BUILD_DIR}/compile_commands.json not found." >&2
    echo "  Run \`cmake -B ${BUILD_DIR}\` first (or set BUILD_DIR=...)." >&2
    exit 1
fi

if ! command -v clang-tidy >/dev/null 2>&1; then
    echo "ERROR: clang-tidy not on PATH. Install with:" >&2
    echo "  apt install clang-tidy   # Debian/Ubuntu" >&2
    echo "  brew install llvm        # macOS" >&2
    exit 1
fi

if [[ $# -gt 0 ]]; then
    files=("$@")
else
    # Default scope: project sources, excluding vendored deps and tests.
    mapfile -t files < <(
        find modules -type f \( -name '*.cpp' -o -name '*.hpp' \) \
            -not -path '*/build*' \
            -not -path '*/.cmake-cache*'
    )
fi

echo "Running clang-tidy on ${#files[@]} file(s) (build: ${BUILD_DIR})..."
clang-tidy -p "${BUILD_DIR}" --quiet "${files[@]}"
