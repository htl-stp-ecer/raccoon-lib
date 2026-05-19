#include <gtest/gtest.h>
#include "foundation/types.hpp"
#include "test_support/test_fixtures.hpp"

using namespace libstp::foundation;
using namespace libstp::foundation::literals;
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

TEST_F(TypesTest, MetersConstructionAndConversion) {
    Meters m1{1.0};
    EXPECT_DOUBLE_EQ(m1.value, 1.0);

    auto m2 = Meters::from_cm(100.0);
    EXPECT_DOUBLE_EQ(m2.value, 1.0);

    auto m3 = Meters::from_mm(1000.0);
    EXPECT_DOUBLE_EQ(m3.value, 1.0);

    EXPECT_DOUBLE_EQ(m2.to_cm(), 100.0);
    EXPECT_DOUBLE_EQ(m3.to_mm(), 1000.0);
}

TEST_F(TypesTest, MetersOperators) {
    Meters a{1.5}, b{0.5};
    EXPECT_DOUBLE_EQ((a + b).value, 2.0);
    EXPECT_DOUBLE_EQ((a - b).value, 1.0);
    EXPECT_DOUBLE_EQ((-b).value, -0.5);
    EXPECT_DOUBLE_EQ((a * 2.0).value, 3.0);
    EXPECT_DOUBLE_EQ((a / 3.0).value, 0.5);
}

TEST_F(TypesTest, MetersLiterals) {
    auto m = 1.5_m;
    EXPECT_DOUBLE_EQ(m.value, 1.5);
    EXPECT_DOUBLE_EQ((100.0_cm).value, 1.0);
    EXPECT_DOUBLE_EQ((1000.0_mm).value, 1.0);
}

TEST_F(TypesTest, MetersPerSecondConstructionAndConversion) {
    MetersPerSecond v1{2.0};
    EXPECT_DOUBLE_EQ(v1.value, 2.0);

    auto v2 = MetersPerSecond::from_cm_per_sec(200.0);
    EXPECT_DOUBLE_EQ(v2.value, 2.0);
}
