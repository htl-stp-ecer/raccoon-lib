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

# Find the installed libstp package location
SITE_LIBSTP="$(pip show libstp | grep '^Location:' | cut -d' ' -f2)/libstp"

# --- 1) C++ binding stubs (pybind11-stubgen) ---
# Run from /tmp to avoid repo's python/ shadowing the installed package.
# pybind11-stubgen must import the compiled .so modules to introspect them.
(cd /tmp && pybind11-stubgen libstp -o "$STAGING" --ignore-all-errors) || true

# --- 2) Python source stubs (mypy stubgen --no-import) ---
# --no-import parses source files without importing, avoiding side effects
# (libstp.__init__ registers signal handlers and calls initialize_logging).
stubgen --no-import -p libstp --search-path "$SITE_LIBSTP/.." -o "$PY_STUBS_TMP" || true
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

# --- 5) Diagnostics ---
echo "=== Stubs package contents ==="
find "$STAGING/libstp" -type f | sort
PYI_COUNT=$(find "$STAGING/libstp" -name '*.pyi' | wc -l)
echo "Total .pyi files: $PYI_COUNT"
if [ "$PYI_COUNT" -eq 0 ]; then
  echo "ERROR: No .pyi stubs were generated!" >&2
  exit 1
fi

# --- 6) Write pyproject.toml ---
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

# --- 7) Build the universal wheel ---
pip install --quiet build
cd "$STAGING"
python -m build --wheel --outdir /src/build-docker-stubs
