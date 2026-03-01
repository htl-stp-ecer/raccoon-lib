#include <gtest/gtest.h>
#include "foundation/types.hpp"
#include "test_support/test_fixtures.hpp"

using namespace libstp::foundation;
using namespace libstp::test;

class TypesTest : public AlgorithmTestFixture {};

TEST_F(TypesTest, ChassisVelocityDefaultConstruction) {
    ChassisVelocity cv{};
    EXPECT_NEAR(cv.vx, 0.0, kTolerance);
    EXPECT_NEAR(cv.vy, 0.0, kTolerance);
    EXPECT_NEAR(cv.wz, 0.0, kTolerance);
}

TEST_F(TypesTest, ChassisVelocityInitialization) {
    ChassisVelocity cv{1.0, 2.0, 3.0};
    EXPECT_NEAR(cv.vx, 1.0, kTolerance);
    EXPECT_NEAR(cv.vy, 2.0, kTolerance);
    EXPECT_NEAR(cv.wz, 3.0, kTolerance);
}

TEST_F(TypesTest, PoseDefaultInitialization) {
    Pose pose{};
    EXPECT_NEAR(pose.position.x(), 0.0f, 1e-6f);
    EXPECT_NEAR(pose.position.y(), 0.0f, 1e-6f);
    EXPECT_NEAR(pose.position.z(), 0.0f, 1e-6f);
    EXPECT_NEAR(pose.heading, 0.0f, 1e-6f);
}

// TODO: Add more tests for Meters, Radians, etc. once their interfaces are confirmed
