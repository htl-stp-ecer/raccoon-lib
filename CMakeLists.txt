option(ENABLE_SAFETY_CHECKS "Enable runtime safety checks across the whole project" ON)

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
FetchContent_Declare(
        pybind11
        GIT_REPOSITORY https://github.com/pybind/pybind11.git
        GIT_TAG v2.13.6
)
FetchContent_MakeAvailable(pybind11)

FetchContent_Declare(
        eigen
        GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
        GIT_TAG 3.4.0
)

FetchContent_MakeAvailable(eigen)

# Add include directory for Eigen
include_directories(${eigen_SOURCE_DIR})

add_subdirectory(modules)
# Add a library using FindPython's tooling (pybind11 also provides a helper like
# this)
python_add_library(_core MODULE main.cpp WITH_SOABI)
target_link_libraries(_core PRIVATE pybind11::headers)

# Add Eigen as a dependency to your library
target_include_directories(_core PRIVATE ${eigen_SOURCE_DIR})

# Add coroutine support to the _core target
target_compile_features(_core PRIVATE cxx_std_20)

# This is passing in the version as a define just as an example
target_compile_definitions(_core PRIVATE VERSION_INFO=${PROJECT_VERSION})

# The install directory is the output (wheel) directory
install(TARGETS _core DESTINATION libstp)
