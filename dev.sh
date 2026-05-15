#!/usr/bin/env bash
# dev.sh — fast dev loop for raccoon-lib
#
# 1. Rebuilds the local editable install (mock driver, incremental)
# 2. Cross-compiles an ARM64 wheel via Docker and deploys to the wombat
#
# Usage:
#   bash dev.sh                         # local only (no deploy)
#   RPI_HOST=192.168.x.x bash dev.sh    # local + wombat deploy
#
# Env vars:
#   RPI_HOST   — wombat IP (required for deploy)
#   RPI_USER   — SSH user (default: pi)
#   BUILD_JOBS — parallel compile jobs (default: all CPUs)

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
JOBS=$(nproc 2>/dev/null || sysctl -n hw.logicalcpu 2>/dev/null || echo 4)

find_python() {
  if command -v python3 >/dev/null 2>&1; then
    command -v python3
    return 0
  fi
  if command -v python3.13 >/dev/null 2>&1; then
    command -v python3.13
    return 0
  fi
  if command -v python >/dev/null 2>&1; then
    command -v python
    return 0
  fi
  echo "error: no Python interpreter found (tried python3, python3.13, python)" >&2
  exit 1
}

ensure_global_deps() {
  local python_cmd=$1
  "$python_cmd" -m pip install -q -U pip setuptools wheel \
    scikit-build-core pybind11 \
    --break-system-packages
}

PYTHON_CMD="$(find_python)"

# ── 1. Local editable install (mock driver, incremental) ──────────────────
ensure_global_deps "$PYTHON_CMD"
echo "▶ Rebuilding local install (mock)..."
CMAKE_BUILD_PARALLEL_LEVEL=$JOBS "$PYTHON_CMD" -m pip install -e "$SCRIPT_DIR" \
  --no-build-isolation \
  --config-settings=cmake.define.DRIVER_BUNDLE=mock \
  --config-settings=cmake.define.LIBSTP_RUN_RUFF=OFF \
  --config-settings=cmake.define.LIBSTP_RUN_MYPY=OFF \
  --break-system-packages \
  -q
echo "✓ Local install updated"

# ── 2. Wombat deploy (optional) ───────────────────────────────────────────
if [[ -z "${RPI_HOST:-}" ]]; then
  echo "ℹ  RPI_HOST not set — skipping wombat deploy"
  exit 0
fi

echo "▶ Cross-compiling ARM64 wheel (SKIP_TESTS=1)..."
SKIP_TESTS=1 BUILD_JOBS=$JOBS bash "$SCRIPT_DIR/build.sh"

echo "▶ Deploying to ${RPI_USER:-pi}@$RPI_HOST..."
RPI_HOST=$RPI_HOST RPI_USER=${RPI_USER:-pi} python3 "$SCRIPT_DIR/install.py"
