register_libstp_module(foundation "FALSE" "")

# Skip PCH on Apple: universal2 builds emit -Xarch_<arch> -include <pch> pairs
# that sccache can't parse, forcing every TU to fall back to direct compile.
# Losing PCH costs ~10-20% on macOS but unlocks sccache (>5x net win on repeated
# builds). Linux/Windows keep PCH.
if(NOT APPLE)
  target_precompile_headers(foundation PUBLIC include/foundation/pch.hpp)
endif()

target_link_libraries(foundation PUBLIC Eigen3::Eigen spdlog::spdlog)
