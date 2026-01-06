register_libstp_test(test_odometry
    SOURCES
        test_fused_odometry.cpp
    LIBS
        odometry_fused
        odometry
        kinematics
)
