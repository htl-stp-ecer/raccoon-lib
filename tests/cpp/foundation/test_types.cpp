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

TEST_F(TypesTest, PoseExplicitInitialization) {
    Pose pose{Eigen::Vector3f::Zero(), Eigen::Quaternionf::Identity()};
    EXPECT_NEAR(pose.position.x(), 0.0f, 1e-6f);
    EXPECT_NEAR(pose.position.y(), 0.0f, 1e-6f);
    EXPECT_NEAR(pose.position.z(), 0.0f, 1e-6f);
    EXPECT_NEAR(pose.orientation.w(), 1.0f, 1e-6f);
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


TEST_F(TypesTest, MetersPerSecondConstructionAndConversion) {
    MetersPerSecond v1{2.0};
    EXPECT_DOUBLE_EQ(v1.value, 2.0);

    auto v2 = MetersPerSecond::from_cm_per_sec(200.0);
    EXPECT_DOUBLE_EQ(v2.value, 2.0);
}

TEST_F(TypesTest, MetersPerSecondOperators) {
    MetersPerSecond a{1.5};
    EXPECT_DOUBLE_EQ((-a).value, -1.5);
    EXPECT_DOUBLE_EQ((a * 2.0).value, 3.0);
}


TEST_F(TypesTest, RadiansConstructionAndConversion) {
    Radians r1{3.14159265358979323846};
    EXPECT_DOUBLE_EQ(r1.value, M_PI);

    auto r2 = Radians::from_deg(180.0);
    EXPECT_NEAR(r2.value, M_PI, 1e-12);

    EXPECT_NEAR(r2.to_deg(), 180.0, 1e-12);
}

TEST_F(TypesTest, RadiansOperators) {
    Radians a{1.0}, b{2.0};
    EXPECT_DOUBLE_EQ((a + b).value, 3.0);
    EXPECT_DOUBLE_EQ((b - a).value, 1.0);
    EXPECT_DOUBLE_EQ((-a).value, -1.0);
}


TEST_F(TypesTest, RadiansPerSecondConstructionAndConversion) {
    RadiansPerSecond w1{2.0};
    EXPECT_DOUBLE_EQ(w1.value, 2.0);

    auto w2 = RadiansPerSecond::from_deg_per_sec(180.0);
    EXPECT_NEAR(w2.value, M_PI, 1e-12);
}

TEST_F(TypesTest, RadiansPerSecondOperators) {
    RadiansPerSecond a{1.5};
    EXPECT_DOUBLE_EQ((-a).value, -1.5);
    EXPECT_DOUBLE_EQ((a * 2.0).value, 3.0);
}


TEST_F(TypesTest, LiteralsConstruction) {
    auto m = 1.0_m;
    EXPECT_DOUBLE_EQ(m.value, 1.0);

    auto cm = 100.0_cm;
    EXPECT_DOUBLE_EQ(cm.value, 1.0);

    auto mm = 1000.0_mm;
    EXPECT_DOUBLE_EQ(mm.value, 1.0);

    auto v = 2.0_mps;
    EXPECT_DOUBLE_EQ(v.value, 2.0);

    auto rad = 3.14159265358979323846_rad;
    EXPECT_DOUBLE_EQ(rad.value, M_PI);

    auto deg = 180.0_deg;
    EXPECT_NEAR(deg.value, M_PI, 1e-12);

    auto w = 180.0_rad_s;
    EXPECT_DOUBLE_EQ(w.value, 180.0);
}