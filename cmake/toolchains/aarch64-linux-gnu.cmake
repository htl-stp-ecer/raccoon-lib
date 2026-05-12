# Cross-compilation toolchain: x86_64 host → aarch64 Raspberry Pi 3
# Prerequisites: run scripts/setup-cross-toolchain.sh once before using this.
#
# Usage (direct CMake): cmake -B build-cross
# -DCMAKE_TOOLCHAIN_FILE=cmake/toolchains/aarch64-linux-gnu.cmake
#
# Usage (via build-cross.sh): bash build-cross.sh

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

# ---- Compilers ----
set(CMAKE_C_COMPILER aarch64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER aarch64-linux-gnu-g++)
set(CMAKE_STRIP aarch64-linux-gnu-strip)

# ---- Sysroot / find-root ----
# Debian/Ubuntu multiarch installs arm64 libs and headers under these paths.
# /usr/include/python3.13 is architecture-independent and lives on the host, but
# we include it here so FindPython's validation pass doesn't filter it out when
# CMAKE_FIND_ROOT_PATH_MODE_INCLUDE is set to ONLY.
list(APPEND CMAKE_FIND_ROOT_PATH /usr/aarch64-linux-gnu
     /usr/lib/aarch64-linux-gnu /usr/include/aarch64-linux-gnu
     /usr/include/python3.13)

# Programs (ninja, python, etc.) must run on the host, not inside the target
# sysroot
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
# Libraries must come from the target sysroot only
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
# Headers: BOTH so arch-independent headers (glib-2.0, python3.13) are found on
# the host while the cross-compiler's default search paths still take precedence
# for arch-specific headers like pyconfig.h and glib glibconfig.h.
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE BOTH)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

# ---- Python ----
# Two separate concerns: 1. Interpreter   → HOST python3 (runs syntax checks,
# ruff, stubs at build time) 2. Dev headers   → TARGET python3.13-dev:arm64
# (compiled into _core.so)
#
# Find the host interpreter explicitly, bypassing CMAKE_FIND_ROOT_PATH:
find_program(
  _python_host python3.13
  PATHS /usr/bin /usr/local/bin
  NO_CMAKE_FIND_ROOT_PATH REQUIRED)
set(Python_EXECUTABLE
    "${_python_host}"
    CACHE FILEPATH "Host Python interpreter" FORCE)

# The architecture-independent Python.h lives in /usr/include/python3.13
# (installed by the host python3.13-dev package). The cross-compiler
# automatically searches /usr/include/aarch64-linux-gnu for the arch-specific
# pyconfig.h, so pointing Python_INCLUDE_DIR at the main include dir is
# sufficient.
set(Python_INCLUDE_DIR
    /usr/include/python3.13
    CACHE PATH "Python headers" FORCE)

# Do NOT set Python_LIBRARY: pybind11 extension modules do not link against
# libpython at build time (by design), and setting it to an empty string can
# cause FindPython to report "library not found".
