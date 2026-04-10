set(CMAKE_POLICY_VERSION_MINIMUM 3.5)

option(ENABLE_SAFETY_CHECKS "Enable runtime safety checks across the whole project" ON)
option(LIBSTP_TRACE_LOGGING "Compile SPDLOG trace statements into all targets" ON)

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

# Add include directory for Eigen
include_directories(${eigen_SOURCE_DIR})

# ---------- raccoon-transport (brings LCM + message types) ----------
# Force static LCM so the wheel is self-contained (no external liblcm.so needed).
set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
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
set(BUILD_SHARED_LIBS ON CACHE BOOL "" FORCE)


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
target_include_directories(_core PRIVATE ${eigen_SOURCE_DIR})
target_compile_definitions(_core PRIVATE VERSION_INFO=${PROJECT_VERSION})

install(TARGETS _core DESTINATION raccoon)

install(DIRECTORY "${CMAKE_SOURCE_DIR}/python/raccoon/"
        DESTINATION raccoon
        FILES_MATCHING PATTERN "*.py")

# Compatibility shim: install python/libstp_compat/libstp -> top-level 'libstp'
# package that emits a DeprecationWarning and aliases libstp.* to raccoon.*
install(DIRECTORY "${CMAKE_SOURCE_DIR}/python/libstp_compat/libstp/"
        DESTINATION libstp
        FILES_MATCHING PATTERN "*.py")

# Add test directory after modules are defined
if(LIBSTP_BUILD_TESTS)
    add_subdirectory(tests/cpp)
endif()
