#!/usr/bin/env bash
# build-cross.sh — Native cross-compilation of the raccoon ARM64 wheel.
#
# Builds directly on the host using aarch64-linux-gnu-g++ — no Docker, no QEMU.
# Incremental builds are typically 5–10× faster than the QEMU Docker build.
#
# First-time setup (run once):
#   bash scripts/setup-cross-toolchain.sh
#
# Usage:
#   bash build-cross.sh                     # build wheel
#   SKIP_DSL_GEN=1 bash build-cross.sh      # skip generate_step_builders.py
#   FORCE_REBUILD=1 bash build-cross.sh     # clean + rebuild
set -euo pipefail

# -------- Config (env overridable) --------
PROJECT_NAME="${PROJECT_NAME:-raccoon}"
BUILD_DIR="${BUILD_DIR:-build-cross}"
BUILD_TYPE="${BUILD_TYPE:-Release}"
BUILD_NUMBER="${BUILD_NUMBER:-0}"
SKBUILD_DIR="${SKBUILD_DIR:-_skbuild-cross}"
FETCHCONTENT_DIR="${FETCHCONTENT_DIR:-.cmake-cache-cross}"
LIBSTP_RUN_MYPY="${LIBSTP_RUN_MYPY:-OFF}"
SKIP_DSL_GEN="${SKIP_DSL_GEN:-0}"
FORCE_REBUILD="${FORCE_REBUILD:-0}"

TOOLCHAIN_FILE="$(pwd)/cmake/toolchains/aarch64-linux-gnu.cmake"

if command -v nproc >/dev/null 2>&1; then
  BUILD_JOBS="$(nproc)"
else
  BUILD_JOBS="$(getconf _NPROCESSORS_ONLN 2>/dev/null || echo 4)"
fi
BUILD_JOBS="${BUILD_JOBS:-4}"

# -------- Preflight: cross-toolchain installed? --------
if ! command -v aarch64-linux-gnu-g++ >/dev/null 2>&1; then
  echo "✖ aarch64-linux-gnu-g++ not found."
  echo "  Run: bash scripts/setup-cross-toolchain.sh"
  exit 1
fi

if [[ ! -f /usr/include/aarch64-linux-gnu/python3.13/pyconfig.h ]]; then
  echo "✖ ARM64 Python 3.13 headers not found at"
  echo "  /usr/include/aarch64-linux-gnu/python3.13/pyconfig.h"
  echo "  Run: bash scripts/setup-cross-toolchain.sh"
  exit 1
fi

# -------- Version patching (same logic as build.sh) --------
PYPROJECT="pyproject.toml"
RACCOON_PYPROJECT="raccoon-transport/python/pyproject.toml"
ORIGINAL_VERSION=""
RACCOON_ORIGINAL_VERSION=""

patch_version_file() {
  local file="$1" version="$2"
  if [[ ! "$version" =~ ^[A-Za-z0-9.+_-]+$ ]]; then
    echo "ERROR: refusing to patch version with non-PEP-440 value: $version" >&2
    exit 1
  fi
  sed "s|^version = .*|version = \"$version\"|" "$file" > "$file.tmp"
  mv "$file.tmp" "$file"
}

restore_version() {
  if [[ -n "$ORIGINAL_VERSION" ]]; then
    patch_version_file "$PYPROJECT" "$ORIGINAL_VERSION"
  fi
  if [[ -n "$RACCOON_ORIGINAL_VERSION" ]]; then
    patch_version_file "$RACCOON_PYPROJECT" "$RACCOON_ORIGINAL_VERSION"
  fi
}

if [[ "$BUILD_NUMBER" != "0" ]]; then
  ORIGINAL_VERSION=$(sed -n 's/^version = "\([^"]*\)"/\1/p' "$PYPROJECT")
  BASE_VERSION="${ORIGINAL_VERSION%.*}"
  NEW_VERSION="${BASE_VERSION}.${BUILD_NUMBER}"
  echo "▶ Patching raccoon version: ${ORIGINAL_VERSION} → ${NEW_VERSION}"
  patch_version_file "$PYPROJECT" "$NEW_VERSION"

  RACCOON_ORIGINAL_VERSION=$(sed -n 's/^version = "\([^"]*\)"/\1/p' "$RACCOON_PYPROJECT")
  RACCOON_BASE="${RACCOON_ORIGINAL_VERSION%.*}"
  RACCOON_NEW="${RACCOON_BASE}.${BUILD_NUMBER}"
  echo "▶ Patching raccoon-transport version: ${RACCOON_ORIGINAL_VERSION} → ${RACCOON_NEW}"
  patch_version_file "$RACCOON_PYPROJECT" "$RACCOON_NEW"

  trap restore_version EXIT
fi

echo "▶ Cross-building $PROJECT_NAME (aarch64) using native toolchain"

# -------- Python build tools: sanity-check --------
if ! python3.13 -c "import build, scikit_build_core, pybind11" 2>/dev/null; then
  echo "✖ Missing Python build tools (build, scikit-build-core, pybind11)."
  echo "  Install via uv:  uv pip install build 'scikit-build-core>=0.10' pybind11 'cmake>=3.27'"
  echo "  Or system-wide:  pip install build 'scikit-build-core>=0.10' pybind11 --break-system-packages"
  exit 1
fi

if [[ "$FORCE_REBUILD" == "1" ]]; then
  echo "• Force rebuild: cleaning $SKBUILD_DIR"
  rm -rf "$SKBUILD_DIR"
fi

mkdir -p "$BUILD_DIR"

# -------- DSL code generation (host Python, no ARM64 needed) --------
if [[ "$SKIP_DSL_GEN" != "1" ]]; then
  echo "• Generating step builder DSL code..."
  python3.13 tools/generate_step_builders.py
fi

# -------- Build ARM64 wheel --------
# _PYTHON_HOST_PLATFORM overrides sysconfig.get_platform() so the produced
# wheel gets the correct linux_aarch64 platform tag instead of linux_x86_64.
CMAKE_ARGS="\
-DCMAKE_TOOLCHAIN_FILE=${TOOLCHAIN_FILE};\
-DFETCHCONTENT_BASE_DIR=$(pwd)/${FETCHCONTENT_DIR};\
-DCMAKE_CXX_COMPILER_LAUNCHER=ccache;\
-DCMAKE_C_COMPILER_LAUNCHER=ccache;\
-DLIBSTP_BUILD_TESTS=OFF;\
-DLIBSTP_RUN_MYPY=${LIBSTP_RUN_MYPY};\
-DCMAKE_BUILD_TYPE=${BUILD_TYPE}"

# -------- Patch LCM sources (cross-build cache only) --------
# LCM's CMakeLists.txt unconditionally adds lcmgen/ which builds lcm-gen as an
# ARM64 binary. That binary can't run on the x86_64 host, so we need to skip it.
# We patch the downloaded sources in the cross-specific FetchContent cache so
# only this build is affected (Docker builds use a separate .cmake-cache-docker/).
LCM_CMAKELISTS="${FETCHCONTENT_DIR}/lcm-src/CMakeLists.txt"
LCM_PATCH_STAMP="${FETCHCONTENT_DIR}/lcm-src/.cross-patched"
if [[ -f "$LCM_CMAKELISTS" && ! -f "$LCM_PATCH_STAMP" ]]; then
  echo "• Patching LCM sources: wrapping add_subdirectory(lcmgen) for cross-compile..."
  sed -i 's|^add_subdirectory(lcmgen)|if(NOT CMAKE_CROSSCOMPILING)\nadd_subdirectory(lcmgen)\nendif() # patched by build-cross.sh|' \
    "$LCM_CMAKELISTS"
  touch "$LCM_PATCH_STAMP"
  echo "  Patched: $LCM_CMAKELISTS"
fi

echo "• Building ARM64 wheel with scikit-build-core ($BUILD_JOBS jobs)..."
_PYTHON_HOST_PLATFORM=linux-aarch64 \
  SKBUILD_BUILD_DIR="$(pwd)/$SKBUILD_DIR" \
  CMAKE_BUILD_PARALLEL_LEVEL="$BUILD_JOBS" \
  python3.13 -m build --wheel --outdir "$BUILD_DIR" --no-isolation \
    -C "cmake.args=${CMAKE_ARGS}"

WHEEL_FILE=$(find "$BUILD_DIR" -name "*.whl" -type f -exec ls -t {} + | head -1)
if [[ ! -f "$WHEEL_FILE" ]]; then
  echo "✖ Build completed but wheel not found in $BUILD_DIR"
  exit 1
fi

echo "✓ Built ARM64 wheel → $WHEEL_FILE"
ls -la "$WHEEL_FILE"
