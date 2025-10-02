set(CMAKE_POLICY_VERSION_MINIMUM 3.5)

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
set(PYBIND11_DETAILED_ERROR_MESSAGES ON CACHE BOOL "" FORCE)
FetchContent_Declare(
        pybind11
        GIT_REPOSITORY https://github.com/pybind/pybind11.git
        GIT_TAG v2.13.6
)
FetchContent_MakeAvailable(pybind11)

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

# Add LCM
set(LCM_ENABLE_JAVA OFF CACHE BOOL "" FORCE)
set(LCM_ENABLE_PYTHON OFF CACHE BOOL "" FORCE)
set(LCM_ENABLE_DOCS OFF CACHE BOOL "" FORCE)
FetchContent_Declare(
        lcm
        GIT_REPOSITORY https://github.com/lcm-proj/lcm.git
        GIT_TAG v1.5.0
)
FetchContent_MakeAvailable(lcm)

add_subdirectory(lcm-messages)

set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
set(SPDLOG_BUILD_SHARED OFF CACHE BOOL "" FORCE)
FetchContent_Declare(
        spdlog
        GIT_REPOSITORY https://github.com/gabime/spdlog.git
        GIT_TAG v1.14.1
)
FetchContent_MakeAvailable(spdlog)
set(BUILD_SHARED_LIBS ON CACHE BOOL "" FORCE)

python_add_library(_core MODULE main.cpp WITH_SOABI)

target_link_libraries(_core PRIVATE
        pybind11::headers
        spdlog::spdlog
        Eigen3::Eigen
)

add_subdirectory(modules)

target_compile_features(_core PRIVATE cxx_std_23)
target_include_directories(_core PRIVATE ${eigen_SOURCE_DIR})
target_compile_definitions(_core PRIVATE VERSION_INFO=${PROJECT_VERSION})

install(TARGETS _core DESTINATION libstp)

