register_libstp_module(cam "FALSE" "calibration_store;foundation")

# Link raccoon::transport for LCM subscribe support.
target_link_libraries(cam PUBLIC raccoon::transport)

# The pybind11 extension also needs the transport headers.
if (TARGET libstp_cam)
    target_link_libraries(libstp_cam PRIVATE raccoon::transport)
endif ()
