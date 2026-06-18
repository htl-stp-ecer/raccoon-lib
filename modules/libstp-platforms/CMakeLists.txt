function(register_driver DRIVER_NAME)
  set(DRIVER_SRC_DIR "${CMAKE_CURRENT_LIST_DIR}/src")
  file(GLOB DRIVER_SRCS "${DRIVER_SRC_DIR}/*.cpp")
  if(NOT DRIVER_SRCS)
    message(FATAL_ERROR "No .cpp files found in ${DRIVER_SRC_DIR}")
  endif()
  set(DRIVER_TARGET ${DRIVER_NAME})
  add_library(${DRIVER_TARGET} STATIC ${DRIVER_SRCS})
  target_include_directories(${DRIVER_TARGET}
                             PUBLIC "${CMAKE_CURRENT_LIST_DIR}/include")
  target_link_libraries(${DRIVER_TARGET} PUBLIC hal platform_core)

  get_property(_drivers GLOBAL PROPERTY PLATFORM_DRIVERS)
  if(NOT _drivers)
    set(_drivers "")
  endif()
  list(APPEND _drivers ${DRIVER_TARGET})
  set_property(GLOBAL PROPERTY PLATFORM_DRIVERS "${_drivers}")

  message(STATUS "Registered driver: ${DRIVER_TARGET}")
endfunction()

function(register_platform PLATFORM_NAME)
  get_property(_drivers GLOBAL PROPERTY PLATFORM_DRIVERS)
  add_library(platform_${PLATFORM_NAME} INTERFACE)
  if(_drivers)
    target_link_libraries(platform_${PLATFORM_NAME} INTERFACE ${_drivers})
  endif()
  add_library(platform ALIAS platform_${PLATFORM_NAME})
  message(STATUS "Linking platform_${PLATFORM_NAME} with drivers: ${_drivers}")
endfunction()

set(DRIVER_BUNDLE
    "wombat"
    CACHE STRING "Which driver bundle to use (e.g. wombat, esp32, mock)")
set_property(CACHE DRIVER_BUNDLE PROPERTY STRINGS wombat esp32 mock)

# ----------------------------------------------------------------------------
# Runtime-switchable bundles
#
# Build each bundle as a self-contained SHARED library that exports the hal
# device symbols. Every source under <bundle>/*/src is glob-compiled into one
# .so (drivers + platform_core + heartbeat_publish). The library links `hal`
# (the bundle-agnostic glue half of hal::Motor etc.) plus the bundle's own
# dependencies, so it resolves with no undefined symbols and can be dlopen()ed
# with RTLD_NOW|RTLD_GLOBAL. The pybind11 extensions are built with the device
# symbols left undefined (see register_libstp_module) and bind against whichever
# bundle raccoon/__init__.py preloads.
# ----------------------------------------------------------------------------
function(register_runtime_bundle BUNDLE_NAME)
  file(GLOB_RECURSE BUNDLE_SRCS
       "${CMAKE_CURRENT_LIST_DIR}/${BUNDLE_NAME}/*/src/*.cpp")
  if(NOT BUNDLE_SRCS)
    message(FATAL_ERROR "No bundle sources for ${BUNDLE_NAME}")
  endif()
  set(_tgt raccoon_platform_${BUNDLE_NAME})
  add_library(${_tgt} SHARED ${BUNDLE_SRCS})
  target_include_directories(
    ${_tgt} PRIVATE "${CMAKE_CURRENT_LIST_DIR}/${BUNDLE_NAME}/core/include"
                    "${CMAKE_SOURCE_DIR}/modules/libstp-screen/include")
  # hal carries the bundle-agnostic glue (Motor::~Motor, port registry,
  # failsafe); foundation/kinematics are common deps for every driver.
  target_link_libraries(${_tgt} PRIVATE hal foundation kinematics)
  set_target_properties(${_tgt} PROPERTIES BUILD_RPATH "$ORIGIN" INSTALL_RPATH
                                                                 "$ORIGIN")
  install(TARGETS ${_tgt} DESTINATION raccoon)
  message(STATUS "Built runtime platform bundle: lib${_tgt}.so")
endfunction()

if(LIBSTP_RUNTIME_PLATFORM)
  register_runtime_bundle(mock)
  target_link_libraries(raccoon_platform_mock PRIVATE sim)

  register_runtime_bundle(wombat)
  target_link_libraries(raccoon_platform_wombat PRIVATE transport_core
                                                        raccoon::transport)
else()
  add_subdirectory(${DRIVER_BUNDLE})
endif()
