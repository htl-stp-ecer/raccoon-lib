#!/usr/bin/env bash
# scripts/mutation_py.sh — run Python mutation testing (mutmut 2.4) on a single
# source file against a focused set of tests, and report the kill rate +
# surviving mutants.
#
#   bash scripts/mutation_py.sh <source.py> <pytest-target> [pytest-target...]
#
# Example:
#   bash scripts/mutation_py.sh \
#       modules/libstp-motion/python/raccoon/step/motion/wall_align.py \
#       tests/python/test_wall_align_core.py
#
# REQUIREMENTS (built by the test-harness setup, see scripts/setup_test_venv.sh):
#   - .venv-test : a venv with raccoon installed via DRIVER_BUNDLE=mock +
#     LIBSTP_RUNTIME_PLATFORM=OFF (so the mock sim works and the full suite runs)
#   - the .py files under .venv-test/.../site-packages/raccoon are SYMLINKS to
#     the source tree, so mutating a source file is seen by normal imports AND
#     by tests that load source via importlib.spec_from_file_location.
#
# mutmut mutates the SOURCE file in place, runs the given tests, and restores it.
# A surviving mutant means the tests did not detect that change → a real gap.
# Serialize calls to this script: mutmut mutates shared source files in place,
# so two concurrent runs corrupt each other.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

VENV="${TEST_VENV:-$REPO_ROOT/.venv-test}"
PY="$VENV/bin/python"
MUTMUT="$VENV/bin/mutmut"

if [[ ! -x "$MUTMUT" ]]; then
  echo "error: $MUTMUT not found — run scripts/setup_test_venv.sh first" >&2
  exit 2
fi

SRC="$1"; shift
if [[ $# -eq 0 ]]; then
  echo "error: need at least one pytest target" >&2
  exit 2
fi
PYTEST_TARGETS=("$@")

# mutmut 2.4 keeps a fixed-name .mutmut-cache in CWD; clear it so each run is fresh.
rm -rf "$REPO_ROOT/.mutmut-cache"

RUNNER="$PY -m pytest ${PYTEST_TARGETS[*]} -x -q -o addopts= -o filterwarnings= -p no:cacheprovider"

echo "==> mutating $SRC"
echo "    tests: ${PYTEST_TARGETS[*]}"
set +e
"$MUTMUT" run --paths-to-mutate "$SRC" --runner "$RUNNER" --no-progress
set -e

# Summarise from the sqlite cache.
"$PY" - "$SRC" <<'PY'
import sqlite3, sys
db = sqlite3.connect(".mutmut-cache")
counts = dict(db.execute("select status, count(*) from mutant group by status").fetchall())
# A timeout means the mutant caused a hang/inf-loop — the suite WOULD fail it in
# CI, so it counts as killed (mutmut labels it bad_timeout).
killed = counts.get("ok_killed", 0) + counts.get("timeout", 0) + counts.get("bad_timeout", 0)
survived = counts.get("bad_survived", 0)
total = sum(counts.values())
rate = 100.0 * killed / total if total else 0.0
print()
print(f"  Mutation score: {killed}/{total} killed ({rate:.1f}%)  |  survived: {survived}")
print(f"  raw: {counts}")
if survived:
    print("\n  Surviving mutants (gaps — strengthen tests to kill these):")
    print("    inspect with:  .venv-test/bin/mutmut show <id>")
PY

# List surviving mutant ids for convenience.
"$MUTMUT" results 2>/dev/null | sed -n '/Survived/,$p' | head -40 || true
