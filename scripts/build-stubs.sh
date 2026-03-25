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

# --- 1) C++ binding stubs (pybind11-stubgen) ---
pybind11-stubgen libstp -o "$STAGING" --ignore-all-errors 2>/dev/null || true

# --- 2) Python source stubs (mypy stubgen) ---
stubgen -p libstp -o "$PY_STUBS_TMP" 2>/dev/null || true
if [ -d "$PY_STUBS_TMP/libstp" ]; then
  find "$PY_STUBS_TMP/libstp" -name '*.pyi' | while read -r pyi; do
    REL="${pyi#$PY_STUBS_TMP/libstp/}"
    DEST="$STAGING/libstp/$REL"
    # pybind11-stubgen output takes priority
    if [ ! -f "$DEST" ]; then
      mkdir -p "$(dirname "$DEST")"
      cp "$pyi" "$DEST"
    fi
  done
fi

# --- 3) Hand-written stubs from repo (highest priority) ---
if [ -d /src/python/stubs/libstp ]; then
  cp -R /src/python/stubs/libstp/. "$STAGING/libstp/"
fi

# --- 4) Strip all runtime code — keep only .pyi + py.typed ---
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
