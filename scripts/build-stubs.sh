#!/usr/bin/env bash
# build-stubs.sh — Generate a stubs-only wheel for IDE support / codegen.
#
# Runs INSIDE the ARM64 Docker container where a mock-platform libstp
# wheel is already installed (no hardware deps, no background threads).
#
# Usage: bash scripts/build-stubs.sh <version>
# Output: /src/build-docker-stubs/libstp_stubs-<version>-py3-none-any.whl

set -euo pipefail

VERSION="${1:?Usage: build-stubs.sh <version>}"
STAGING=/src/stubs-staging
PY_STUBS_TMP=/tmp/py-stubs

rm -rf "$STAGING" "$PY_STUBS_TMP"
mkdir -p "$STAGING/libstp"

SITE_PACKAGES="$(python -c 'import sysconfig; print(sysconfig.get_path("platlib"))')"
echo "Site-packages: $SITE_PACKAGES"

# Stub out the compiled 'raccoon' LCM types module (not in container)
if ! python -c "import raccoon" 2>/dev/null; then
  echo "Creating stub 'raccoon' module for import resolution"
  mkdir -p "$SITE_PACKAGES/raccoon"
  touch "$SITE_PACKAGES/raccoon/__init__.py"
fi

# --- 1) C++ binding stubs (pybind11-stubgen) ---
echo "--- Running pybind11-stubgen ---"
(cd /tmp && pybind11-stubgen libstp -o "$STAGING" --ignore-all-errors) || true

# --- 2) Python source stubs (mypy stubgen --inspect-mode) ---
# --inspect-mode imports modules at runtime, producing cleaner stubs that
# preserve docstrings, full signatures, and proper types compared to
# --no-import which only does static analysis and loses most of this.
echo "--- Running stubgen --inspect-mode ---"
stubgen --inspect-mode -p libstp --search-path "$SITE_PACKAGES" -o "$PY_STUBS_TMP" || true
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
find "$STAGING/libstp" -name '__init__.py' -exec sh -c 'echo "" > "$1"' _ {} \;

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
