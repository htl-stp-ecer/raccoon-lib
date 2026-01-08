# Test support library - mocks and fixtures
add_library(libstp_test_support STATIC
    mocks.cpp
)

target_include_directories(libstp_test_support PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
)

target_link_libraries(libstp_test_support PUBLIC
    GTest::gtest
    GTest::gmock
    foundation
    kinematics
    odometry
    Eigen3::Eigen
)
