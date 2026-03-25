#!/usr/bin/env bash
# build-stubs.sh — Generate a stubs-only wheel for IDE support / codegen.
#
# Runs INSIDE the ARM64 Docker container where libstp is already installed.
# Usage: bash scripts/build-stubs.sh <version>
#
# Output: /src/build-docker-stubs/libstp_stubs-<version>-py3-none-any.whl

set -euo pipefail

VERSION="${1:?Usage: build-stubs.sh <version>}"
STAGING=/src/stubs-staging
PY_STUBS_TMP=/tmp/py-stubs

rm -rf "$STAGING" "$PY_STUBS_TMP"
mkdir -p "$STAGING/libstp"

# Find the site-packages directory reliably
SITE_PACKAGES="$(python -c 'import sysconfig; print(sysconfig.get_path("purelib"))')"
echo "Site-packages: $SITE_PACKAGES"

# --- 0) Ensure LIBSTP_STUBGEN is exported for all child processes ---
# This env var disables import side effects in __init__.py (atexit hooks,
# signal handlers, logging init) and skips LcmReader background thread.
export LIBSTP_STUBGEN=1

# Stub out the compiled 'raccoon' LCM types module (not installed in container)
if ! python -c "import raccoon" 2>/dev/null; then
  echo "Creating stub 'raccoon' module for import resolution"
  mkdir -p "$SITE_PACKAGES/raccoon"
  touch "$SITE_PACKAGES/raccoon/__init__.py"
fi

# --- 1) C++ binding stubs (pybind11-stubgen) ---
# Run per-submodule for isolation (a segfault in one doesn't kill the rest),
# but pre-import all dependencies so cross-module types are registered.
echo "--- Running pybind11-stubgen per C++ submodule ---"
CPP_MODULES=(
  _core foundation hal drive motion odometry odometry_fused odometry_stm32
  kinematics kinematics_differential kinematics_mecanum
  button sensor_ir sensor_et calibration_store kmeans cam async
)
# Python snippet that pre-imports all C++ modules, then runs stubgen
# on a single target module passed as the first CLI argument.
STUBGEN_RUNNER='
import sys, importlib
target = sys.argv[1]
# Pre-import modules in dependency order so pybind11 cross-module types
# (base classes, default argument types) are registered before introspection.
for mod in [
    "_core", "foundation", "hal", "drive", "motion", "odometry",
    "odometry_fused", "kinematics", "button", "sensor_ir", "sensor_et",
    "calibration_store", "kmeans", "cam",
]:
    try:
        importlib.import_module(f"libstp.{mod}")
    except Exception:
        pass
# Now run pybind11-stubgen on the target module
sys.argv = ["pybind11-stubgen", f"libstp.{target}", "-o", sys.argv[2], "--ignore-all-errors"]
from pybind11_stubgen import main
main()
'
for mod in "${CPP_MODULES[@]}"; do
  echo -n "  libstp.$mod ... "
  if (cd /tmp && python -c "$STUBGEN_RUNNER" "$mod" "$STAGING") 2>&1; then
    echo "OK"
  else
    echo "FAILED (exit $?)"
  fi
done

# --- 2) Python source stubs (mypy stubgen --no-import) ---
# --no-import parses source files without importing, avoiding side effects.
echo "--- Running stubgen --no-import ---"
stubgen --no-import -p libstp --search-path "$SITE_PACKAGES" -o "$PY_STUBS_TMP" || true
if [ -d "$PY_STUBS_TMP/libstp" ]; then
  find "$PY_STUBS_TMP/libstp" -name '*.pyi' | while read -r pyi; do
    REL="${pyi#$PY_STUBS_TMP/libstp/}"
    DEST="$STAGING/libstp/$REL"
    # pybind11-stubgen output takes priority for C++ modules
    if [ ! -f "$DEST" ]; then
      mkdir -p "$(dirname "$DEST")"
      cp "$pyi" "$DEST"
    fi
  done
fi

# --- 3) Strip all runtime code — keep only .pyi + py.typed ---
find "$STAGING/libstp" -name '*.py' ! -name '__init__.py' -delete
# Empty __init__.py files (needed for package structure, no logic)
find "$STAGING/libstp" -name '__init__.py' -exec sh -c 'echo "" > "$1"' _ {} \;

# Ensure every directory has an __init__.pyi for proper resolution
find "$STAGING/libstp" -type d | while read -r dir; do
  if [ ! -f "$dir/__init__.pyi" ] && [ ! -f "$dir/__init__.py" ]; then
    touch "$dir/__init__.pyi"
  fi
done

touch "$STAGING/libstp/py.typed"

# --- 4) Diagnostics ---
echo "=== Stubs package contents ==="
find "$STAGING/libstp" -type f | sort
PYI_COUNT=$(find "$STAGING/libstp" -name '*.pyi' | wc -l)
echo "Total .pyi files: $PYI_COUNT"
if [ "$PYI_COUNT" -eq 0 ]; then
  echo "ERROR: No .pyi stubs were generated!" >&2
  exit 1
fi

# --- 5) Write pyproject.toml ---
cat > "$STAGING/pyproject.toml" << EOF
[build-system]
requires = ["setuptools"]
build-backend = "setuptools.build_meta"

[project]
name = "libstp-stubs"
version = "$VERSION"
description = "Type stubs for libstp (IDE support, type hinting, codegen — no runtime code)"
requires-python = ">=3.11"

[tool.setuptools.packages.find]
include = ["libstp*"]

[tool.setuptools.package-data]
libstp = ["*.pyi", "py.typed", "**/*.pyi"]
EOF

# --- 6) Build the universal wheel ---
pip install --quiet build
cd "$STAGING"
python -m build --wheel --outdir /src/build-docker-stubs
