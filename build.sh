#!/usr/bin/env bash
set -euo pipefail

# -------- Config (env overridable) --------
PROJECT_NAME="${PROJECT_NAME:-libstp}"
BUILD_DIR="${BUILD_DIR:-build-docker}"
PLATFORM="${PLATFORM:-linux/arm64/v8}"
IMAGE_NAME="${IMAGE_NAME:-libstp-dev:arm64}"
CCACHE_VOL="${CCACHE_VOL:-libstp-ccache}"
PIP_CACHE_VOL="${PIP_CACHE_VOL:-libstp-pip-cache}"
BUILD_TYPE="${BUILD_TYPE:-Release}"
# Portable CPU count default
if command -v nproc >/dev/null 2>&1; then
  _cpu="$(nproc)"
else
  _cpu="$(getconf _NPROCESSORS_ONLN 2>/dev/null || echo 4)"
fi
BUILD_JOBS="${BUILD_JOBS:-${_cpu}}"
FORCE_REBUILD="${FORCE_REBUILD:-0}"
REBUILD_IMAGE="${REBUILD_IMAGE:-0}"
CCACHE_MAXSIZE="${CCACHE_MAXSIZE:-3G}"

echo "▶ Building Python library $PROJECT_NAME for $PLATFORM using $IMAGE_NAME"
mkdir -p "$BUILD_DIR"

need_builder() {
  if ! docker buildx inspect >/dev/null 2>&1; then
    echo "• No buildx builder found; creating 'cross'..."
    docker buildx create --name cross --use >/dev/null
    docker buildx inspect --bootstrap >/dev/null
  fi
}

ensure_binfmt() {
  # If Docker Desktop provides emulation, skip installing binfmt
  if docker info 2>/dev/null | grep -qi 'docker desktop'; then
    return
  fi
  # Quick probe: can we run uname on target platform?
  if ! docker run --rm --platform="$PLATFORM" alpine:3.20 uname -m >/dev/null 2>&1; then
    echo "• QEMU binfmt not working — installing emulators (requires privileged)..."
    docker run --rm --privileged tonistiigi/binfmt --install all
    docker run --rm --platform="$PLATFORM" alpine:3.20 uname -m >/dev/null
  fi
}

ensure_image() {
  if ! docker image inspect "$IMAGE_NAME" >/dev/null 2>&1 || [[ "$REBUILD_IMAGE" == "1" ]]; then
    echo "• Building builder image: $IMAGE_NAME (for $PLATFORM)"
    # Important: --platform matches FROM --platform in Dockerfile (use $TARGETPLATFORM inside Dockerfile)
    docker buildx build --platform "$PLATFORM" -t "$IMAGE_NAME" --load .
  fi
}

docker_exec() {
  docker run --rm --platform="$PLATFORM" \
    -v "$PWD":/src \
    -v "$CCACHE_VOL":/ccache \
    -v "$PIP_CACHE_VOL":/root/.cache/pip \
    -e CCACHE_DIR=/ccache \
    -e CCACHE_MAXSIZE="$CCACHE_MAXSIZE" \
    -e CCACHE_COMPRESS=1 \
    -e PIP_CACHE_DIR=/root/.cache/pip \
    -e PYTHONUNBUFFERED=1 \
    -e PYTHONDONTWRITEBYTECODE=1 \
    -e CMAKE_BUILD_PARALLEL_LEVEL="$BUILD_JOBS" \
    -e MAKEFLAGS="-j$BUILD_JOBS" \
    -e SKBUILD_BUILD_DIR=/src/_skbuild-docker \
    -e FETCHCONTENT_BASE_DIR=/src/.cmake-cache-docker \
    --cpus="$(nproc)" \
    -w /src \
    "$IMAGE_NAME" \
    bash -lc "$*"
}

need_builder
ensure_binfmt
docker volume create "$CCACHE_VOL" >/dev/null
docker volume create "$PIP_CACHE_VOL" >/dev/null
ensure_image

if [[ "$FORCE_REBUILD" == "1" ]]; then
  echo "• Force rebuild requested, cleaning build directory"
  docker_exec "rm -rf /src/$BUILD_DIR"
fi

docker_exec g++ --version

echo "• Installing build dependencies first..."
docker_exec "pip install --disable-pip-version-check -U 'scikit-build-core>=0.10' pybind11 'cmake>=3.27'"
echo "• Building Python wheel with scikit-build-core (using all $BUILD_JOBS CPUs)"
docker_exec "CMAKE_BUILD_PARALLEL_LEVEL=$BUILD_JOBS python -m build --wheel --outdir /src/$BUILD_DIR --no-isolation -C cmake.args=-DFETCHCONTENT_BASE_DIR=/src/.cmake-cache-docker"

WHEEL_FILE=$(find "$BUILD_DIR" -name "*.whl" -type f | head -1)
if [[ ! -f "$WHEEL_FILE" ]]; then
  echo "✖ Build completed but wheel not found in $BUILD_DIR"
  exit 1
fi

echo "✓ Built Python wheel → $WHEEL_FILE"
ls -la "$WHEEL_FILE"
