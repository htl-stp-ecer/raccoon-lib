#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "test_support/test_fixtures.hpp"
#include "drive/drive.hpp"

using namespace libstp::test;
using namespace libstp::drive;
using namespace libstp::foundation;

class DriveTest : public ::testing::Test {
protected:
    MotionLimits limits_;

    void SetUp() override {
        limits_.max_v = 1.0;
        limits_.max_omega = 1.0;
    }
};

TEST_F(DriveTest, ConstructsSuccessfully) {
    auto kinematics = std::make_unique<testing::NiceMock<MockKinematics>>();
    kinematics->setupAsDifferential();
    Drive drive(std::move(kinematics), limits_);
    EXPECT_TRUE(&drive);
}

TEST_F(DriveTest, EstimateStateReturnsKinematicsState) {
    auto kinematics = std::make_unique<testing::NiceMock<MockKinematics>>();
    kinematics->setupAsDifferential();
    Drive drive(std::move(kinematics), limits_);

    ChassisVelocity expected{0.5, -0.5, 0.25};
    drive.setVelocity(expected);

    auto kinematics_mock = std::make_unique<testing::NiceMock<MockKinematics>>();
    kinematics_mock->setupAsDifferential();
    kinematics_mock->setEstimatedState(expected);

    auto state = kinematics_mock->estimateState();
    EXPECT_DOUBLE_EQ(state.vx, expected.vx);
    EXPECT_DOUBLE_EQ(state.vy, expected.vy);
    EXPECT_DOUBLE_EQ(state.wz, expected.wz);
}

TEST_F(DriveTest, SoftStopZeroesDesiredVelocity) {
    auto kinematics = std::make_unique<testing::NiceMock<MockKinematics>>();
    kinematics->setupAsDifferential();
    Drive drive(std::move(kinematics), limits_);

    drive.setVelocity({0.8, 0.2, 0.5});
    drive.softStop();
    ChassisVelocity zero{0.0, 0.0, 0.0};
    drive.setVelocity(zero);
}

TEST_F(DriveTest, HardStopCallsKinematicsHardStop) {
    auto kinematics = std::make_unique<testing::NiceMock<MockKinematics>>();
    kinematics->setupAsDifferential();
    EXPECT_CALL(*kinematics, hardStop()).Times(1);

    Drive drive(std::move(kinematics), limits_);
    drive.hardStop();
}
