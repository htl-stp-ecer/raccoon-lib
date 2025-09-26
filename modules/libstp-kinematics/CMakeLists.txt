register_libstp_module(kinematics "FALSE" "foundation")

add_subdirectory(models)

# Link the differential and mecanum kinematics libraries to the main kinematics module
target_link_libraries(kinematics PUBLIC kinematics-differential kinematics-mecanum)