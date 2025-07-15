register_libstp_module(foundation "FALSE" "")

target_precompile_headers(foundation PUBLIC include/foundation/pch.hpp)

target_link_libraries(foundation PUBLIC Eigen3::Eigen)