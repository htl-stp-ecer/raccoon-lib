#!/usr/bin/env bash
# build-cross-docker.sh — Cross-compile the raccoon ARM64 wheel inside Docker.
#
# Runs natively on any x86_64 machine — no ARM64 emulation, no QEMU.
# The Docker image contains the aarch64-linux-gnu-g++ cross-toolchain;
# the container itself is always x86_64.
#
# First run builds the Docker image (~2 min). Subsequent runs reuse it and
# skip straight to the wheel build, with ccache keeping incremental builds fast.
#
# Usage:
#   bash build-cross-docker.sh
#   FORCE_REBUILD=1 bash build-cross-docker.sh
#   REBUILD_IMAGE=1 bash build-cross-docker.sh   # force rebuild Docker image
#   BUILD_NUMBER=42 bash build-cross-docker.sh   # CI version stamp
set -euo pipefail

# -------- Config (env overridable) --------
PROJECT_NAME="${PROJECT_NAME:-raccoon}"
BUILD_DIR="${BUILD_DIR:-build-cross}"
BUILD_TYPE="${BUILD_TYPE:-Release}"
BUILD_NUMBER="${BUILD_NUMBER:-0}"
IMAGE_NAME="${IMAGE_NAME:-raccoon-cross-builder:latest}"
CCACHE_VOL="${CCACHE_VOL:-raccoon-ccache-cross}"
REBUILD_IMAGE="${REBUILD_IMAGE:-0}"
CCACHE_MAXSIZE="${CCACHE_MAXSIZE:-3G}"
FORCE_REBUILD="${FORCE_REBUILD:-0}"
SKIP_DSL_GEN="${SKIP_DSL_GEN:-0}"
LIBSTP_RUN_MYPY="${LIBSTP_RUN_MYPY:-OFF}"

SKBUILD_DIR="${SKBUILD_DIR:-_skbuild-cross}"
FETCHCONTENT_DIR="${FETCHCONTENT_DIR:-.cmake-cache-cross}"

if command -v nproc >/dev/null 2>&1; then
  BUILD_JOBS="$(nproc)"
else
  BUILD_JOBS="$(getconf _NPROCESSORS_ONLN 2>/dev/null || echo 4)"
fi
BUILD_JOBS="${BUILD_JOBS:-4}"

echo "▶ Cross-building $PROJECT_NAME (ARM64) via Docker — native x86_64, no QEMU"

# -------- Build Docker image if needed --------
if ! docker image inspect "$IMAGE_NAME" >/dev/null 2>&1 || [[ "$REBUILD_IMAGE" == "1" ]]; then
  echo "• Building cross-builder image: $IMAGE_NAME"
  docker build -t "$IMAGE_NAME" -f Dockerfile.cross .
fi

# -------- Docker volume for ccache --------
docker volume create "$CCACHE_VOL" >/dev/null

mkdir -p "$BUILD_DIR"

# -------- Passthrough env to build-cross.sh inside the container --------
docker run --rm \
  -v "$PWD":/src \
  -v "$CCACHE_VOL":/ccache \
  -e CCACHE_DIR=/ccache \
  -e CCACHE_MAXSIZE="$CCACHE_MAXSIZE" \
  -e CCACHE_COMPRESS=1 \
  -e CCACHE_BASEDIR=/src \
  -e CCACHE_NOHASHDIR=1 \
  -e CCACHE_COMPILERCHECK=content \
  -e BUILD_DIR="$BUILD_DIR" \
  -e BUILD_TYPE="$BUILD_TYPE" \
  -e BUILD_NUMBER="$BUILD_NUMBER" \
  -e BUILD_JOBS="$BUILD_JOBS" \
  -e SKBUILD_DIR="$SKBUILD_DIR" \
  -e FETCHCONTENT_DIR="$FETCHCONTENT_DIR" \
  -e FORCE_REBUILD="$FORCE_REBUILD" \
  -e SKIP_DSL_GEN="$SKIP_DSL_GEN" \
  -e LIBSTP_RUN_MYPY="$LIBSTP_RUN_MYPY" \
  -e PYTHONUNBUFFERED=1 \
  -e PYTHONDONTWRITEBYTECODE=1 \
  -e GIT_CONFIG_COUNT=1 \
  -e GIT_CONFIG_KEY_0=safe.directory \
  -e GIT_CONFIG_VALUE_0='*' \
  --cpus="$BUILD_JOBS" \
  -w /src \
  "$IMAGE_NAME" \
  bash build-cross.sh

WHEEL_FILE=$(find "$BUILD_DIR" -name "*.whl" -type f -exec ls -t {} + | head -1)
echo "✓ ARM64 wheel → $WHEEL_FILE"
ls -la "$WHEEL_FILE"
