#!/usr/bin/env bash
# scripts/coverage.sh — measure raccoon test coverage (Python + C++).
#
# This is the single entry point for evaluating how much of the library the
# test suite exercises, in both languages, and for *gating* on it later:
#
#   bash scripts/coverage.sh                       # both languages, report only
#   bash scripts/coverage.sh --python-only         # fast: just the Python suite
#   bash scripts/coverage.sh --cpp-only            # just the C++ (ctest) suite
#   bash scripts/coverage.sh --fail-under 60       # gate Python at 60 % lines
#   bash scripts/coverage.sh --cpp-fail-under 50   # gate C++ at 50 % lines
#   bash scripts/coverage.sh -- tests/python/test_line_follow.py   # subset
#
# Python coverage is measured against the installed `raccoon` package and then
# re-mapped onto the split source tree (modules/libstp-*/python + python/) by
# scripts/coverage_report.py. C++ coverage instruments a throwaway build dir
# (-DLIBSTP_COVERAGE=ON) and harvests gcov data with gcovr.
#
# Artifacts land under build/coverage/ (Python) and the C++ build dir.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$REPO_ROOT"

# ---- defaults / arg parsing ----
RUN_PYTHON=1
RUN_CPP=1
FAIL_UNDER=0
FAIL_UNDER_MODULE=0
CPP_FAIL_UNDER=0
CPP_BUILD_DIR="${CPP_BUILD_DIR:-build-coverage}"
CPP_NO_BUILD=0
SHOW_FILES=0
PYTEST_EXTRA=()

usage() { sed -n '2,30p' "$0"; exit "${1:-0}"; }

while [[ $# -gt 0 ]]; do
  case "$1" in
    --python-only) RUN_CPP=0; shift;;
    --cpp-only)    RUN_PYTHON=0; shift;;
    --fail-under)  FAIL_UNDER="$2"; shift 2;;
    --fail-under=*) FAIL_UNDER="${1#*=}"; shift;;
    --fail-under-module) FAIL_UNDER_MODULE="$2"; shift 2;;
    --cpp-fail-under) CPP_FAIL_UNDER="$2"; shift 2;;
    --cpp-fail-under=*) CPP_FAIL_UNDER="${1#*=}"; shift;;
    --build-dir)   CPP_BUILD_DIR="$2"; shift 2;;
    --no-build)    CPP_NO_BUILD=1; shift;;
    --show-files)  SHOW_FILES=1; shift;;
    -h|--help)     usage 0;;
    --)            shift; PYTEST_EXTRA=("$@"); break;;
    *)             echo "unknown option: $1" >&2; usage 1;;
  esac
done

# ---- pick interpreter (prefer repo venv) ----
if [[ -n "${PYTHON:-}" ]]; then
  :
elif [[ -x "$REPO_ROOT/.venv/bin/python" ]]; then
  PYTHON="$REPO_ROOT/.venv/bin/python"
else
  PYTHON="$(command -v python3)"
fi

OUT_DIR="$REPO_ROOT/build/coverage"
mkdir -p "$OUT_DIR"

PY_RC=0
CPP_RC=0

# =====================================================================
# Python coverage
# =====================================================================
if [[ "$RUN_PYTHON" -eq 1 ]]; then
  echo "==> Python coverage"

  # Ensure measurement deps are present (idempotent).
  if ! "$PYTHON" -c "import pytest_cov, coverage" >/dev/null 2>&1; then
    echo "    installing coverage + pytest-cov ..."
    "$PYTHON" -m pip install -q 'coverage[toml]' pytest-cov
  fi

  # Local dev wheels are built without the mock sim bundle, so the in-house
  # `raccoon_testing` pytest plugin and several async tests cannot load. Detect
  # that and fall back to a "local mode" that disables the plugin and drops the
  # files that fail to even collect — so we still get a coverage number from the
  # pure-Python tests. CI (mock bundle present) runs the full suite.
  LOCAL_MODE=0
  "$PYTHON" -c "import raccoon.testing.sim" >/dev/null 2>&1 || LOCAL_MODE=1

  PYTEST_TARGETS=()
  if [[ ${#PYTEST_EXTRA[@]} -gt 0 ]]; then
    PYTEST_TARGETS=("${PYTEST_EXTRA[@]}")
  else
    PYTEST_TARGETS=(tests)
  fi

  # pytest-cov marks the already-imported editable shim for assertion rewrite,
  # which raises PytestAssertRewriteWarning at config time — and the project's
  # `filterwarnings = error` turns that into a hard abort before any test runs
  # (a command-line `-W ignore` is applied too late to help). Override the ini
  # option so that one meta-warning is ignored while real warnings still error.
  PYTEST_FLAGS=(
    -o addopts=""
    -o filterwarnings=$'error\nignore::pytest.PytestAssertRewriteWarning'
    -p no:cacheprovider
  )
  if [[ "$LOCAL_MODE" -eq 1 ]]; then
    echo "    local mode: mock sim bundle absent — disabling raccoon_testing plugin"
    PYTEST_FLAGS+=(-p no:raccoon_testing)
    # Probe collection and drop any file that errors (needs mock sim or the
    # asyncio plugin). --continue-on-collection-errors lets the probe finish.
    mapfile -t BROKEN < <(
      "$PYTHON" -m pytest "${PYTEST_TARGETS[@]}" --collect-only -q \
        "${PYTEST_FLAGS[@]}" --continue-on-collection-errors 2>&1 \
        | sed -n 's/^ERROR \([^ ]*\).*/\1/p' | sort -u)
    if [[ ${#BROKEN[@]} -gt 0 ]]; then
      echo "    skipping ${#BROKEN[@]} uncollectable file(s) (need mock sim / asyncio):"
      for f in "${BROKEN[@]}"; do
        echo "      - $f"
        PYTEST_FLAGS+=("--ignore=$f")
      done
    fi
  fi

  # Run the suite under coverage. Don't abort the script if tests fail — we
  # still want the coverage artifacts and report (the gate is enforced below).
  set +e
  "$PYTHON" -m pytest "${PYTEST_TARGETS[@]}" "${PYTEST_FLAGS[@]}" \
    --cov=raccoon --cov-branch \
    --cov-config="$REPO_ROOT/.coveragerc" \
    --cov-report="term-missing:skip-covered" \
    --cov-report="html:$OUT_DIR/html" \
    --cov-report="xml:$OUT_DIR/coverage.xml" \
    --cov-report="json:$OUT_DIR/coverage.json"
  PYTEST_STATUS=$?
  set -e
  if [[ "$PYTEST_STATUS" -ne 0 ]]; then
    echo "    (note: pytest exited $PYTEST_STATUS — some tests failed/skipped; coverage still recorded)"
  fi

  REPORT_ARGS=(--json "$OUT_DIR/coverage.json"
    --fail-under "$FAIL_UNDER" --fail-under-module "$FAIL_UNDER_MODULE")
  [[ "$SHOW_FILES" -eq 1 ]] && REPORT_ARGS+=(--show-files)
  set +e
  "$PYTHON" "$SCRIPT_DIR/coverage_report.py" "${REPORT_ARGS[@]}"
  PY_RC=$?
  set -e
  echo "    HTML: $OUT_DIR/html/index.html   XML: $OUT_DIR/coverage.xml"
fi

# =====================================================================
# C++ coverage
# =====================================================================
if [[ "$RUN_CPP" -eq 1 ]]; then
  echo ""
  echo "==> C++ coverage"

  if ! "$PYTHON" -c "import gcovr" >/dev/null 2>&1; then
    echo "    installing gcovr ..."
    "$PYTHON" -m pip install -q gcovr
  fi

  if [[ "$CPP_NO_BUILD" -eq 0 ]]; then
    echo "    configuring instrumented build in $CPP_BUILD_DIR/ (this is slow on first run) ..."
    GEN=()
    command -v ninja >/dev/null 2>&1 && GEN=(-G Ninja)
    cmake -S "$REPO_ROOT" -B "$CPP_BUILD_DIR" "${GEN[@]}" \
      -DLIBSTP_BUILD_TESTS=ON \
      -DLIBSTP_COVERAGE=ON \
      -DLIBSTP_RUN_MYPY=OFF \
      -DLIBSTP_RUN_RUFF=OFF \
      -DCMAKE_BUILD_TYPE=Debug \
      -DFETCHCONTENT_BASE_DIR="$REPO_ROOT/.cmake-cache"
    cmake --build "$CPP_BUILD_DIR" -j "$(nproc 2>/dev/null || echo 4)"
  fi

  echo "    running C++ tests (ctest) ..."
  set +e
  ctest --test-dir "$CPP_BUILD_DIR" --output-on-failure -E MockSimIntegration
  CTEST_STATUS=$?
  set -e
  [[ "$CTEST_STATUS" -ne 0 ]] && echo "    (note: ctest exited $CTEST_STATUS; coverage still harvested)"

  echo "    harvesting coverage with gcovr ..."
  # Measure only our own C++ sources: modules/*/{src,include} + the hal/platform
  # C++. Exclude fetched deps (gtest/eigen/pybind), generated code and tests.
  GCOVR_COMMON=(
    --root "$REPO_ROOT"
    --gcov-ignore-parse-errors
    --exclude-unreachable-branches
    --filter "$REPO_ROOT/modules/"
    --exclude ".*/tests/.*"
    --exclude ".*/_deps/.*"
    --exclude-directories ".*/_deps/.*"
  )
  set +e
  "$PYTHON" -m gcovr "${GCOVR_COMMON[@]}" \
    --xml-pretty -o "$OUT_DIR/cpp-coverage.xml" \
    --html-details "$OUT_DIR/cpp-html/index.html" \
    --print-summary "$CPP_BUILD_DIR"
  GCOVR_STATUS=$?
  set -e

  if [[ "$GCOVR_STATUS" -eq 0 && "$CPP_FAIL_UNDER" != "0" ]]; then
    set +e
    "$PYTHON" -m gcovr "${GCOVR_COMMON[@]}" \
      --fail-under-line "$CPP_FAIL_UNDER" "$CPP_BUILD_DIR" >/dev/null
    CPP_RC=$?
    set -e
    if [[ "$CPP_RC" -ne 0 ]]; then
      echo "    FAIL: C++ line coverage below required ${CPP_FAIL_UNDER}%" >&2
    else
      echo "    OK: C++ coverage gate passed."
    fi
  fi
  echo "    HTML: $OUT_DIR/cpp-html/index.html   XML: $OUT_DIR/cpp-coverage.xml"
fi

# =====================================================================
echo ""
if [[ "$PY_RC" -ne 0 || "$CPP_RC" -ne 0 ]]; then
  echo "Coverage gate FAILED."
  exit 1
fi
echo "Coverage evaluation complete."
