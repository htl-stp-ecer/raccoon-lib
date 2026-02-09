# Test helper function for module tests
function(register_libstp_test test_name)
    set(options "")
    set(oneValueArgs "")
    set(multiValueArgs SOURCES LIBS)
    cmake_parse_arguments(TEST "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    add_executable(${test_name} ${TEST_SOURCES})
    target_link_libraries(${test_name} PRIVATE
        ${TEST_LIBS}
        libstp_test_support
        GTest::gtest_main
        GTest::gmock
    )
    target_include_directories(${test_name} PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    )
    gtest_discover_tests(${test_name})
endfunction()

# Test support library (mocks and fixtures)
add_subdirectory(support)

# Module tests
add_subdirectory(foundation)
add_subdirectory(motion)
add_subdirectory(kinematics)
add_subdirectory(odometry)
add_subdirectory(drive)
add_subdirectory(kmeans)
add_subdirectory(platform/wombat/lcm)