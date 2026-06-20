#!/usr/bin/env bash
# scripts/mutation_cpp_mull.sh — real C++ mutation testing with mull (LLVM).
#
#   scripts/mutation_cpp_mull.sh <test_binary> [--source <src.cpp>] [--gtest <filter>] [--threshold N]
#
# Requires:
#   - mull-20 package installed (mull-runner-20; LLVM 20.1.x).
#   - a build produced by scripts/setup_mull_build.sh (clang-20 + LIBSTP_MUTATION=ON),
#     so the test binary has mull mutations embedded by the IR frontend plugin.
#
# mull-runner reads ./mull.yml. We rewrite it each run to exclude test/dep code
# and (optionally) scope mutations to a single source file via includePaths.
# Survivors = mutations the tests did not catch (real gaps). Serialize runs.
set -euo pipefail
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

BIN=""; SOURCE=""; GTEST=""; THRESH=0
while [[ $# -gt 0 ]]; do
  case "$1" in
    --source) SOURCE="$2"; shift 2;;
    --gtest) GTEST="$2"; shift 2;;
    --threshold) THRESH="$2"; shift 2;;
    *) BIN="$1"; shift;;
  esac
done
[[ -x "$BIN" ]] || { echo "error: test binary not found/executable: $BIN" >&2; exit 2; }
command -v mull-runner-20 >/dev/null || { echo "error: mull-runner-20 not installed" >&2; exit 2; }

# (Re)generate mull.yml: never mutate tests/deps; optionally scope to one file.
{
  echo "excludePaths:"
  echo "  - .*/tests/.*"
  echo "  - .*/_deps/.*"
  echo "  - .*/.cmake-cache.*"
  echo "  - .*/build-.*/.*"
  if [[ -n "$SOURCE" ]]; then
    base="$(basename "$SOURCE")"
    echo "includePaths:"
    echo "  - .*/${base//./\\.}\$"
  fi
} > mull.yml

RUNNER_ARGS=()
[[ -n "$GTEST" ]] && RUNNER_ARGS=(-- --gtest_filter="$GTEST")

echo "==> mull-runner-20 on $BIN ${SOURCE:+(scoped to $SOURCE)} ${GTEST:+[gtest: $GTEST]}"
OUT=/tmp/mull-run-$$.txt
set +e
mull-runner-20 --reporters IDE --ide-reporter-show-killed --allow-surviving \
  "$BIN" "${RUNNER_ARGS[@]}" 2>&1 | grep -E "warning: (Killed|Survived)" > "$OUT"
set -e

# Optionally narrow the report to the source of interest.
VIEW="$OUT"
if [[ -n "$SOURCE" ]]; then
  base="$(basename "$SOURCE")"
  grep -E "/$base:" "$OUT" > "$OUT.f" || true
  VIEW="$OUT.f"
fi

killed=$(grep -cE "warning: Killed" "$VIEW" || true)
survived=$(grep -cE "warning: Survived" "$VIEW" || true)
total=$((killed + survived))
score=0
[[ $total -gt 0 ]] && score=$(( 100 * killed / total ))

echo
echo "  Mutation score: ${killed}/${total} killed (${score}%)  |  survived: ${survived}"
if [[ $survived -gt 0 ]]; then
  echo
  echo "  Surviving mutants (gaps — add assertions to kill these):"
  grep -E "warning: Survived" "$VIEW" | sed -E 's#^.*/(modules/[^ ]+) warning: Survived: (.*)$#   - \1  | \2#' | head -60
fi
rm -f "$OUT" "$OUT.f"

if [[ "$THRESH" != "0" && $score -lt $THRESH ]]; then
  echo "  FAIL: mull score ${score}% < required ${THRESH}%" >&2
  exit 1
fi
