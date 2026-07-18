#!/usr/bin/env bash
# build-stubs.sh — Generate type stubs for raccoon's pybind11 bindings
# plus its pure-Python modules.
#
# The produced wheel IS the default ``raccoon-library`` distribution: a
# platform-independent (py3-none-any) .pyi-only header package. The compiled
# runtime + simulator ships separately as ``raccoon-sim`` and is pulled in via
# the ``raccoon-library[sim]`` extra.
#
# Two modes:
#
# 1. CI / wheel build (default): runs with a mock raccoon already installed;
#    emits the ``raccoon-library`` header wheel ready for PyPI.
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

# --- 3) Strip ALL runtime code — keep only .pyi + py.typed ---
# The default ``raccoon-library`` wheel must ship *exclusively* .pyi stubs so it
# can coexist in the same ``raccoon/`` namespace as the compiled ``raccoon-sim``
# wheel (``pip install raccoon-library[sim]``) without RECORD file conflicts.
# raccoon-sim owns every ``.py`` / ``.so``; this package owns every ``.pyi``.
# So we delete *all* ``.py`` — including ``__init__.py`` — and substitute an
# ``__init__.pyi`` in each package dir so type checkers still see each subpackage.
find "$STAGING/raccoon" -name '*.py' -delete

find "$STAGING/raccoon" -type d | while read -r dir; do
  if [ ! -f "$dir/__init__.pyi" ]; then
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
# This is the DEFAULT ``raccoon-library`` distribution: a platform-independent
# (py3-none-any) header package of .pyi stubs. ``raccoon-library[sim]`` pulls in
# the compiled runtime + simulator from the sibling ``raccoon-sim`` distribution
# (built by scikit-build/cibuildwheel), pinned to the exact same version so the
# stubs and native bindings can never drift apart.
#
# Package discovery: with no ``__init__.py`` left on disk, setuptools' package
# *finder* can't detect the tree, so we enumerate every ``raccoon`` subdirectory
# as an explicit package. ``package-data`` then ships each package's ``.pyi``
# (``__init__.pyi`` matches ``*.pyi``) plus the ``py.typed`` marker.
PACKAGES="$(
  cd "$STAGING" && find raccoon -type d | sed 's#/#.#g' | sort |
    sed 's/^/    "/; s/$/",/'
)"

cat > "$STAGING/pyproject.toml" << EOF
[build-system]
requires = ["setuptools"]
build-backend = "setuptools.build_meta"

[project]
name = "raccoon-library"
version = "$VERSION"
description = "Raccoon robotics library — platform-independent Python/.pyi headers (IDE support, type hinting, toolchain codegen). Install the [sim] extra for the compiled runtime + simulator."
requires-python = ">=3.11"

[project.optional-dependencies]
# The compiled runtime + simulator lives in the sibling ``raccoon-sim``
# distribution. Pinned exact so headers and native bindings stay in lockstep.
sim = ["raccoon-sim==$VERSION"]

[tool.setuptools]
packages = [
$PACKAGES
]

[tool.setuptools.package-data]
"*" = ["*.pyi", "py.typed"]
EOF

# --- 6) Build the universal wheel (skip in local-dev mode) ---
if [[ -n "${SKIP_WHEEL:-}" ]]; then
  echo "SKIP_WHEEL set — staging only at $STAGING; not building wheel."
  exit 0
fi

"$PYTHON" -m pip install --quiet build
cd "$STAGING"
"$PYTHON" -m build --wheel --outdir "$STUBS_OUTPUT"
