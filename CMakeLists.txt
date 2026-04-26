set(CMAKE_POLICY_VERSION_MINIMUM 3.5)

option(ENABLE_SAFETY_CHECKS "Enable runtime safety checks across the whole project" ON)
# Default OFF in production builds: trace statements run in 100 Hz motion-update
# loops (e.g. linear_motion::update prints 8+ floats per cycle). Even with the
# `if constexpr` short-circuit in LIBSTP_LOG_CALL, leaving them compiled in
# means runtime filter checks fire 800+ times/sec on the Pi for nothing.
# Turn ON during development; flip OFF for wheels going to a robot.
option(LIBSTP_TRACE_LOGGING "Compile SPDLOG trace statements into all targets" OFF)

cmake_minimum_required(VERSION 3.15...3.27)

set(FETCHCONTENT_BASE_DIR "${CMAKE_SOURCE_DIR}/.cmake-cache" CACHE PATH "")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

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
    message(STATUS "  -> All log statements compiled in; use runtime filtering to control output")
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
set(PYBIND11_DETAILED_ERROR_MESSAGES ON CACHE BOOL "" FORCE)
FetchContent_Declare(
        pybind11
        GIT_REPOSITORY https://github.com/pybind/pybind11.git
        GIT_TAG v2.13.6
)
FetchContent_MakeAvailable(pybind11)

# Google Test for C++ unit testing - must be declared before LCM to avoid conflicts
option(LIBSTP_BUILD_TESTS "Build unit tests" OFF)

if(LIBSTP_BUILD_TESTS)
    FetchContent_Declare(
        googletest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG v1.14.0
    )
    # Prevent overriding parent project's compiler/linker settings on Windows
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    # Don't install gtest
    set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)

    enable_testing()
    include(GoogleTest)
endif()

set(EIGEN_BUILD_DOC OFF CACHE BOOL "" FORCE)
set(EIGEN_BUILD_TESTING OFF CACHE BOOL "" FORCE)
set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
FetchContent_Declare(
        eigen
        GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
        GIT_TAG 3.4.0
)

FetchContent_MakeAvailable(eigen)
# Eigen3::Eigen is exported by the FetchContent project and carries its own
# include directories — modules pick it up transitively via foundation
# (which links it PUBLIC). No need for a global include_directories().

# All vendored deps build STATIC so the produced wheel is self-contained
# and doesn't depend on a system liblcm.so / libspdlog.so / libyaml-cpp.so.
# Subprojects that respect their own *_BUILD_SHARED variable still have it
# set explicitly below; raccoon-transport's LCM only honours BUILD_SHARED_LIBS.
# Module shared/static (libfoundation.so, libbutton.so) is decided per-target
# in modules/CMakeLists.txt with explicit STATIC/SHARED, not via this flag.
set(BUILD_SHARED_LIBS OFF CACHE BOOL "Build vendored libraries as shared" FORCE)

# ---------- raccoon-transport (brings LCM + message types) ----------
FetchContent_Declare(
        raccoon_transport
        SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/raccoon-transport
)
FetchContent_MakeAvailable(raccoon_transport)

set(SPDLOG_BUILD_SHARED OFF CACHE BOOL "" FORCE)
FetchContent_Declare(
        spdlog
        GIT_REPOSITORY https://github.com/gabime/spdlog.git
        GIT_TAG v1.14.1
)
FetchContent_MakeAvailable(spdlog)

set(YAML_BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
set(YAML_CPP_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(YAML_CPP_BUILD_TOOLS OFF CACHE BOOL "" FORCE)
set(YAML_CPP_BUILD_CONTRIB OFF CACHE BOOL "" FORCE)
FetchContent_Declare(
        yaml-cpp
        GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
        GIT_TAG yaml-cpp-0.7.0
)

FetchContent_MakeAvailable(yaml-cpp)


python_add_library(_core MODULE main.cpp WITH_SOABI)

target_link_libraries(_core PRIVATE
        pybind11::headers
        spdlog::spdlog
        Eigen3::Eigen
        yaml-cpp::yaml-cpp
)

add_subdirectory(modules)

target_compile_features(_core PRIVATE cxx_std_23)
target_compile_definitions(_core PRIVATE VERSION_INFO=${PROJECT_VERSION})

install(TARGETS _core DESTINATION raccoon)

install(DIRECTORY "${CMAKE_SOURCE_DIR}/python/raccoon/"
        DESTINATION raccoon
        FILES_MATCHING PATTERN "*.py")

# Install bundled test scenes alongside the raccoon package so
# raccoon.testing can resolve scene names like "empty_table.ftmap" via
# importlib.resources.
install(DIRECTORY "${CMAKE_SOURCE_DIR}/scenes/"
        DESTINATION raccoon/scenes
        FILES_MATCHING PATTERN "*.ftmap")

# Compatibility shim: install python/libstp_compat/libstp -> top-level 'libstp'
# package that emits a DeprecationWarning and aliases libstp.* to raccoon.*
install(DIRECTORY "${CMAKE_SOURCE_DIR}/python/libstp_compat/libstp/"
        DESTINATION libstp
        FILES_MATCHING PATTERN "*.py")

# Add test directory after modules are defined
if(LIBSTP_BUILD_TESTS)
    add_subdirectory(tests/cpp)
endif()
