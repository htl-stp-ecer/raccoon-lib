#!/usr/bin/env bash
# scripts/setup_mull_build.sh — configure + build the clang/mull instrumented
# C++ test build so `scripts/mutation_cpp_mull.sh` can run real mutation testing.
#
# Prereq: the mull-20 package (provides mull-runner-20 + /usr/lib/mull-ir-frontend-20).
# Install once with:
#   curl -sSL -o /tmp/mull20.deb \
#     https://github.com/mull-project/mull/releases/download/0.34.0/Mull-20-0.34.0-LLVM-20.1.2-ubuntu-amd64-24.04.deb
#   sudo apt-get install -y /tmp/mull20.deb
#
# The build needs several clang-specific workarounds (the normal build uses gcc):
#   - CMAKE_CXX_SCAN_FOR_MODULES=OFF : the project has no C++20 modules; without
#     this CMake demands clang-scan-deps and every try_compile fails.
#   - FMT_CONSTEVAL= (empty)         : spdlog 1.14's bundled fmt uses consteval
#     format-string checks that clang-20 rejects; this forces the constexpr path.
#   - a SEPARATE FetchContent cache (.cmake-cache-mull) so clang doesn't collide
#     with the gcc-populated .cmake-cache.
set -euo pipefail
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

command -v clang++-20 >/dev/null || { echo "need clang++-20" >&2; exit 2; }
[[ -f /usr/lib/mull-ir-frontend-20 ]] || { echo "need mull-20 package (mull-ir-frontend-20)" >&2; exit 2; }

BUILD_DIR="${MULL_BUILD_DIR:-build-mull}"
echo "==> configuring $BUILD_DIR (clang-20 + LIBSTP_MUTATION)"
cmake -S . -B "$BUILD_DIR" -G Ninja \
  -DCMAKE_C_COMPILER=clang-20 -DCMAKE_CXX_COMPILER=clang++-20 \
  -DCMAKE_CXX_SCAN_FOR_MODULES=OFF \
  -DCMAKE_CXX_FLAGS="-DFMT_CONSTEVAL=" \
  -DLIBSTP_BUILD_TESTS=ON -DLIBSTP_MUTATION=ON \
  -DLIBSTP_RUN_MYPY=OFF -DLIBSTP_RUN_RUFF=OFF \
  -DCMAKE_BUILD_TYPE=Debug \
  -DFETCHCONTENT_BASE_DIR="$REPO_ROOT/.cmake-cache-mull"

TARGETS=("${@:-test_foundation test_kinematics test_motion}")
echo "==> building targets: ${TARGETS[*]}"
cmake --build "$BUILD_DIR" --target ${TARGETS[*]} -j "$(nproc)"
echo "==> done. Run mutation with: scripts/mutation_cpp_mull.sh $BUILD_DIR/tests/cpp/<m>/<target> --source <src> --gtest '<Suite>.*'"
