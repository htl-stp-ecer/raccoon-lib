register_libstp_module(foundation "FALSE" "")

target_precompile_headers(foundation PUBLIC include/foundation/pch.hpp)

target_link_libraries(foundation PUBLIC Eigen3::Eigen spdlog::spdlog)

# Transport bindings live inside the foundation Python module so users can do
# `from raccoon.foundation import get_transport, Subscription`. The
# SharedTransport implementation itself stays in transport_core (a separate
# static lib) to avoid a link cycle between foundation and threading. We pull
# transport_core in only for the pybind11 module (libstp_foundation), and only
# when it actually exists — mock bundles skip the transport binding entirely.
if(NOT DRIVER_BUNDLE STREQUAL "mock")
  # transport_core is added later in modules/CMakeLists.txt but CMake resolves
  # target link references at generate time, so a forward reference is fine.
  target_link_libraries(libstp_foundation PRIVATE transport_core)
  target_compile_definitions(libstp_foundation PRIVATE LIBSTP_HAS_TRANSPORT=1)
endif()
