#!/usr/bin/env bash
# build-stubs.sh — Generate type stubs for raccoon's pybind11 bindings
# plus its pure-Python modules.
#
# Two modes:
#
# 1. CI / wheel build (default): runs inside the ARM64 Docker image with
#    a mock raccoon already installed; emits a stubs-only wheel ready for
#    PyPI.
#       bash scripts/build-stubs.sh <version>
#
# 2. Local dev: skip the wheel step and dump stubs into a staging dir
#    pointed at by the CMake mypy target. Requires raccoon importable in
#    the active environment (``pip install -e .[dev]`` is enough).
#       STAGING=$PWD/build-mecanum/python-staging \
#       SKIP_WHEEL=1 \
#       bash scripts/build-stubs.sh dev
#
# Env knobs:
#   STAGING       Output staging dir (default: /src/stubs-staging)
#   STUBS_OUTPUT  Wheel output dir   (default: /src/build-docker-stubs)
#   SKIP_WHEEL    If set, skip the final wheel build (local-dev mode)

set -euo pipefail

VERSION="${1:?Usage: build-stubs.sh <version>}"
STAGING="${STAGING:-/src/stubs-staging}"
STUBS_OUTPUT="${STUBS_OUTPUT:-/src/build-docker-stubs}"
PY_STUBS_TMP="${PY_STUBS_TMP:-/tmp/py-stubs}"

# Resolve binaries: pybind11-stubgen and mypy stubgen must run in the
# same Python interpreter where raccoon is importable (they introspect
# the live module). uvx isolation breaks that, so we prefer
# ``python -m`` against the active interpreter and only fall back to
# system entry points. The pinned versions match pyproject.toml's dev
# extras so CI and local stub generation produce identical output.
PYTHON="${PYTHON:-python3}"

if "$PYTHON" -c "import pybind11_stubgen" 2>/dev/null; then
  PYBIND11_STUBGEN=("$PYTHON" -m pybind11_stubgen)
elif command -v pybind11-stubgen >/dev/null 2>&1; then
  PYBIND11_STUBGEN=(pybind11-stubgen)
else
  echo "ERROR: pybind11-stubgen not importable in $PYTHON. Install via:" >&2
  echo "  $PYTHON -m pip install -e .[dev]" >&2
  exit 1
fi

if "$PYTHON" -c "import mypy.stubgen" 2>/dev/null; then
  STUBGEN=("$PYTHON" -m mypy.stubgen)
elif command -v stubgen >/dev/null 2>&1; then
  STUBGEN=(stubgen)
else
  echo "ERROR: mypy stubgen not importable in $PYTHON. Install via:" >&2
  echo "  $PYTHON -m pip install -e .[dev]" >&2
  exit 1
fi

rm -rf "$STAGING" "$PY_STUBS_TMP"
mkdir -p "$STAGING/raccoon"

PYTHON="${PYTHON:-python3}"
SITE_PACKAGES="$("$PYTHON" -c 'import sysconfig; print(sysconfig.get_path("platlib"))')"
echo "Site-packages: $SITE_PACKAGES"

# Stub out the raccoon_transport LCM types module if not present in container
if ! "$PYTHON" -c "import raccoon_transport" 2>/dev/null; then
  echo "Creating stub 'raccoon_transport' module for import resolution"
  mkdir -p "$SITE_PACKAGES/raccoon_transport"
  touch "$SITE_PACKAGES/raccoon_transport/__init__.py"
fi

# --- 1) C++ binding stubs (pybind11-stubgen) ---
echo "--- Running pybind11-stubgen ---"
(cd /tmp && "${PYBIND11_STUBGEN[@]}" raccoon -o "$STAGING" --ignore-all-errors) || true

# --- 2) Python source stubs (mypy stubgen --inspect-mode) ---
# --inspect-mode imports modules at runtime, producing cleaner stubs that
# preserve docstrings, full signatures, and proper types compared to
# --no-import which only does static analysis and loses most of this.
echo "--- Running stubgen --inspect-mode ---"
"${STUBGEN[@]}" --inspect-mode -p raccoon --search-path "$SITE_PACKAGES" -o "$PY_STUBS_TMP" || true
if [ -d "$PY_STUBS_TMP/raccoon" ]; then
  find "$PY_STUBS_TMP/raccoon" -name '*.pyi' | while read -r pyi; do
    REL="${pyi#$PY_STUBS_TMP/raccoon/}"
    DEST="$STAGING/raccoon/$REL"
    # pybind11-stubgen output takes priority for C++ modules
    if [ ! -f "$DEST" ]; then
      mkdir -p "$(dirname "$DEST")"
      cp "$pyi" "$DEST"
    fi
  done
fi

# --- 3) Strip all runtime code — keep only .pyi + py.typed ---
find "$STAGING/raccoon" -name '*.py' ! -name '__init__.py' -delete
find "$STAGING/raccoon" -name '__init__.py' -exec sh -c 'echo "" > "$1"' _ {} \;

find "$STAGING/raccoon" -type d | while read -r dir; do
  if [ ! -f "$dir/__init__.pyi" ] && [ ! -f "$dir/__init__.py" ]; then
    touch "$dir/__init__.pyi"
  fi
done

touch "$STAGING/raccoon/py.typed"

# --- 4) Diagnostics ---
echo "=== Stubs package contents ==="
find "$STAGING/raccoon" -type f | sort
PYI_COUNT=$(find "$STAGING/raccoon" -name '*.pyi' | wc -l)
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
name = "raccoon-stubs"
version = "$VERSION"
description = "Type stubs for raccoon (IDE support, type hinting, codegen — no runtime code)"
requires-python = ">=3.11"

[tool.setuptools.packages.find]
include = ["raccoon*"]

[tool.setuptools.package-data]
raccoon = ["*.pyi", "py.typed", "**/*.pyi"]
EOF

# --- 6) Build the universal wheel (skip in local-dev mode) ---
if [[ -n "${SKIP_WHEEL:-}" ]]; then
  echo "SKIP_WHEEL set — staging only at $STAGING; not building wheel."
  exit 0
fi

"$PYTHON" -m pip install --quiet build
cd "$STAGING"
"$PYTHON" -m build --wheel --outdir "$STUBS_OUTPUT"
