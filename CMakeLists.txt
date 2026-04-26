set(CMAKE_POLICY_VERSION_MINIMUM 3.5)

option(ENABLE_SAFETY_CHECKS
       "Enable runtime safety checks across the whole project" ON)
# Default OFF in production builds: trace statements run in 100 Hz motion-update
# loops (e.g. linear_motion::update prints 8+ floats per cycle). Even with the
# `if constexpr` short-circuit in LIBSTP_LOG_CALL, leaving them compiled in
# means runtime filter checks fire 800+ times/sec on the Pi for nothing. Turn ON
# during development; flip OFF for wheels going to a robot.
option(LIBSTP_TRACE_LOGGING "Compile SPDLOG trace statements into all targets"
       OFF)

cmake_minimum_required(VERSION 3.15...3.27)

set(FETCHCONTENT_BASE_DIR
    "${CMAKE_SOURCE_DIR}/.cmake-cache"
    CACHE PATH "")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
# Required for clang-tidy / clangd / scripts/run-clang-tidy.sh to find sources.
set(CMAKE_EXPORT_COMPILE_COMMANDS
    ON
    CACHE BOOL "Emit compile_commands.json")

# Set C++20 as the standard for coroutine support
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Include FetchContent module
include(FetchContent)

if(ENABLE_SAFETY_CHECKS)
  add_compile_definitions(SAFETY_CHECKS_ENABLED)
endif()

if(LIBSTP_TRACE_LOGGING)
  message(STATUS "Enabling trace logging in foundation macros (level: TRACE)")
  message(
    STATUS
      "  -> All log statements compiled in; use runtime filtering to control output"
  )
  add_compile_definitions(FOUNDATION_LOG_ACTIVE_LEVEL=0)
else()
  message(STATUS "Setting logging level to INFO")
  add_compile_definitions(FOUNDATION_LOG_ACTIVE_LEVEL=2)
endif()

# Scikit-build-core sets these values for you, or you can just hard-code the
# name and version.
project(
  ${SKBUILD_PROJECT_NAME}
  VERSION ${SKBUILD_PROJECT_VERSION}
  LANGUAGES CXX)

# Find the module development requirements (requires FindPython from 3.17 or
# scikit-build-core's built-in backport)
find_package(Python REQUIRED COMPONENTS Interpreter Development.Module)

# Use FetchContent to auto-download pybind11
set(PYBIND11_DETAILED_ERROR_MESSAGES
    ON
    CACHE BOOL "" FORCE)
FetchContent_Declare(
  pybind11
  GIT_REPOSITORY https://github.com/pybind/pybind11.git
  GIT_TAG v2.13.6)
FetchContent_MakeAvailable(pybind11)

# Google Test for C++ unit testing - must be declared before LCM to avoid
# conflicts
option(LIBSTP_BUILD_TESTS "Build unit tests" OFF)

if(LIBSTP_BUILD_TESTS)
  FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.14.0)
  # Prevent overriding parent project's compiler/linker settings on Windows
  set(gtest_force_shared_crt
      ON
      CACHE BOOL "" FORCE)
  # Don't install gtest
  set(INSTALL_GTEST
      OFF
      CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(googletest)

  enable_testing()
  include(GoogleTest)
endif()

set(EIGEN_BUILD_DOC
    OFF
    CACHE BOOL "" FORCE)
set(EIGEN_BUILD_TESTING
    OFF
    CACHE BOOL "" FORCE)
set(BUILD_TESTING
    OFF
    CACHE BOOL "" FORCE)
FetchContent_Declare(
  eigen
  GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
  GIT_TAG 3.4.0)

FetchContent_MakeAvailable(eigen)
# Eigen3::Eigen is exported by the FetchContent project and carries its own
# include directories — modules pick it up transitively via foundation (which
# links it PUBLIC). No need for a global include_directories().

# All vendored deps build STATIC so the produced wheel is self-contained and
# doesn't depend on a system liblcm.so / libspdlog.so / libyaml-cpp.so.
# Subprojects that respect their own *_BUILD_SHARED variable still have it set
# explicitly below; raccoon-transport's LCM only honours BUILD_SHARED_LIBS.
# Module shared/static (libfoundation.so, libbutton.so) is decided per-target in
# modules/CMakeLists.txt with explicit STATIC/SHARED, not via this flag.
set(BUILD_SHARED_LIBS
    OFF
    CACHE BOOL "Build vendored libraries as shared" FORCE)

# ---------- raccoon-transport (brings LCM + message types) ----------
FetchContent_Declare(raccoon_transport
                     SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/raccoon-transport)
FetchContent_MakeAvailable(raccoon_transport)

set(SPDLOG_BUILD_SHARED
    OFF
    CACHE BOOL "" FORCE)
FetchContent_Declare(
  spdlog
  GIT_REPOSITORY https://github.com/gabime/spdlog.git
  GIT_TAG v1.14.1)
FetchContent_MakeAvailable(spdlog)

set(YAML_BUILD_SHARED_LIBS
    OFF
    CACHE BOOL "" FORCE)
set(YAML_CPP_BUILD_TESTS
    OFF
    CACHE BOOL "" FORCE)
set(YAML_CPP_BUILD_TOOLS
    OFF
    CACHE BOOL "" FORCE)
set(YAML_CPP_BUILD_CONTRIB
    OFF
    CACHE BOOL "" FORCE)
FetchContent_Declare(
  yaml-cpp
  GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
  GIT_TAG yaml-cpp-0.7.0)

FetchContent_MakeAvailable(yaml-cpp)

python_add_library(_core MODULE main.cpp WITH_SOABI)

target_link_libraries(_core PRIVATE pybind11::headers spdlog::spdlog
                                    Eigen3::Eigen yaml-cpp::yaml-cpp)

add_subdirectory(modules)

target_compile_features(_core PRIVATE cxx_std_23)
target_compile_definitions(_core PRIVATE VERSION_INFO=${PROJECT_VERSION})

install(TARGETS _core DESTINATION raccoon)

# ---------- Python syntax gate ----------
#
# Before we install any .py file into the wheel, make sure every shipped Python
# source actually parses. A SyntaxError in a leaf step file would otherwise lurk
# in the wheel and only surface on the Pi at import time — painful to diagnose
# remotely. tools/check_python_syntax.py uses the bytecode compiler in-memory
# (no __pycache__ pollution in the source tree); the stamp-file dependency means
# CMake re-runs the check only when at least one source has changed.
file(
  GLOB_RECURSE
  _RACCOON_PYTHON_SOURCES
  CONFIGURE_DEPENDS
  "${CMAKE_SOURCE_DIR}/python/raccoon/*.py"
  "${CMAKE_SOURCE_DIR}/python/libstp_compat/libstp/*.py"
  "${CMAKE_SOURCE_DIR}/modules/*/python/raccoon/*.py")

set(_RACCOON_PYTHON_SYNTAX_STAMP
    "${CMAKE_BINARY_DIR}/python_syntax_check.stamp")
add_custom_command(
  OUTPUT "${_RACCOON_PYTHON_SYNTAX_STAMP}"
  COMMAND
    "${Python_EXECUTABLE}" "${CMAKE_SOURCE_DIR}/tools/check_python_syntax.py"
    ${_RACCOON_PYTHON_SOURCES}
  COMMAND "${CMAKE_COMMAND}" -E touch "${_RACCOON_PYTHON_SYNTAX_STAMP}"
  DEPENDS ${_RACCOON_PYTHON_SOURCES}
          "${CMAKE_SOURCE_DIR}/tools/check_python_syntax.py"
  COMMENT "Checking Python syntax across raccoon-lib"
  VERBATIM)
add_custom_target(raccoon_python_syntax_check ALL
                  DEPENDS "${_RACCOON_PYTHON_SYNTAX_STAMP}")
# Tie the wheel's native module to the syntax gate so a build that produces
# _core.so also implicitly proved the Python tree compiles.
add_dependencies(_core raccoon_python_syntax_check)

# ---------- Ruff lint + format ----------
#
# Run ruff against every Python source the wheel will ship. Two stamps so the
# faster lint pass and the slower format-check can fail independently and so
# CMake can re-run only what changed. Both opt-out via -DLIBSTP_RUN_RUFF=OFF for
# the rare case where a contributor wants to bypass (e.g. exploring a
# half-broken refactor); the pre-push hook still gates the eventual push.
option(LIBSTP_RUN_RUFF "Run ruff lint + format checks during the build" ON)

if(LIBSTP_RUN_RUFF)
  # scripts/run-ruff.sh resolves the right ruff (system → python -m ruff → uvx
  # <pinned>) so contributors don't have to install anything beyond uv. The
  # pinned version inside the script matches the one pinned in
  # .pre-commit-config.yaml, keeping all three quality gates in lockstep.
  set(_RACCOON_RUFF_WRAPPER "${CMAKE_SOURCE_DIR}/scripts/run-ruff.sh")

  # Run ruff with cwd at the source root so its ``--force-exclude`` and
  # ``extend-exclude`` glob patterns resolve against the same tree the patterns
  # were written for. Inside the build dir ruff cannot tell that
  # ``modules/.../foo_dsl.py`` is in scope, so generated files leak into the
  # lint set.
  set(_RACCOON_RUFF_LINT_STAMP "${CMAKE_BINARY_DIR}/python_ruff_lint.stamp")
  add_custom_command(
    OUTPUT "${_RACCOON_RUFF_LINT_STAMP}"
    COMMAND
      bash "${_RACCOON_RUFF_WRAPPER}" check --config
      "${CMAKE_SOURCE_DIR}/pyproject.toml" --force-exclude
      ${_RACCOON_PYTHON_SOURCES}
    COMMAND "${CMAKE_COMMAND}" -E touch "${_RACCOON_RUFF_LINT_STAMP}"
    WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
    DEPENDS ${_RACCOON_PYTHON_SOURCES} "${CMAKE_SOURCE_DIR}/pyproject.toml"
            "${_RACCOON_RUFF_WRAPPER}"
    COMMENT "Linting Python sources with ruff"
    VERBATIM)
  add_custom_target(raccoon_python_ruff_lint ALL
                    DEPENDS "${_RACCOON_RUFF_LINT_STAMP}")

  set(_RACCOON_RUFF_FORMAT_STAMP "${CMAKE_BINARY_DIR}/python_ruff_format.stamp")
  add_custom_command(
    OUTPUT "${_RACCOON_RUFF_FORMAT_STAMP}"
    COMMAND
      bash "${_RACCOON_RUFF_WRAPPER}" format --check --config
      "${CMAKE_SOURCE_DIR}/pyproject.toml" --force-exclude
      ${_RACCOON_PYTHON_SOURCES}
    COMMAND "${CMAKE_COMMAND}" -E touch "${_RACCOON_RUFF_FORMAT_STAMP}"
    WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
    DEPENDS ${_RACCOON_PYTHON_SOURCES} "${CMAKE_SOURCE_DIR}/pyproject.toml"
            "${_RACCOON_RUFF_WRAPPER}"
    COMMENT "Checking Python formatting with ruff"
    VERBATIM)
  add_custom_target(raccoon_python_ruff_format ALL
                    DEPENDS "${_RACCOON_RUFF_FORMAT_STAMP}")

  add_dependencies(_core raccoon_python_ruff_lint raccoon_python_ruff_format)
endif()

install(
  DIRECTORY "${CMAKE_SOURCE_DIR}/python/raccoon/"
  DESTINATION raccoon
  FILES_MATCHING
  PATTERN "*.py")

# Install bundled test scenes alongside the raccoon package so raccoon.testing
# can resolve scene names like "empty_table.ftmap" via importlib.resources.
install(
  DIRECTORY "${CMAKE_SOURCE_DIR}/scenes/"
  DESTINATION raccoon/scenes
  FILES_MATCHING
  PATTERN "*.ftmap")

# Compatibility shim: install python/libstp_compat/libstp -> top-level 'libstp'
# package that emits a DeprecationWarning and aliases libstp.* to raccoon.*
install(
  DIRECTORY "${CMAKE_SOURCE_DIR}/python/libstp_compat/libstp/"
  DESTINATION libstp
  FILES_MATCHING
  PATTERN "*.py")

# Add test directory after modules are defined
if(LIBSTP_BUILD_TESTS)
  add_subdirectory(tests/cpp)
endif()
