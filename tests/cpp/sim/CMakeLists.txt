register_libstp_test(test_sim
    SOURCES
        test_pose2d.cpp
        test_worldmap.cpp
        test_collision.cpp
        test_simworld.cpp
        test_sensors.cpp
    LIBS
        sim
)

# Integration tests that exercise the mock HAL path. Only meaningful when
# DRIVER_BUNDLE=mock (the default is wombat, which uses different drivers).
if(DRIVER_BUNDLE STREQUAL "mock")
    register_libstp_test(test_sim_mock_integration
        SOURCES
            test_mock_integration.cpp
            test_autotick.cpp
        LIBS
            sim
            platform_core
            motor
            odometry_bridge_driver
    )
endif()
