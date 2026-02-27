#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "test_support/test_fixtures.hpp"
#include "drive/drive.hpp"

using namespace libstp::test;
using namespace libstp::drive;
using namespace libstp::foundation;

class DriveTest : public ::testing::Test {
protected:
    ChassisVelocityControlConfig config{};
    testing::NiceMock<MockIMU> imu_;
};

TEST_F(DriveTest, ConstructsSuccessfully) {
    auto kinematics = std::make_unique<testing::NiceMock<MockKinematics>>();
    kinematics->setupAsDifferential();
    Drive drive(std::move(kinematics), config, imu_);
    EXPECT_TRUE(&drive);
}

TEST_F(DriveTest, SoftStopZeroesDesiredVelocity) {
    auto kinematics = std::make_unique<testing::NiceMock<MockKinematics>>();
    kinematics->setupAsDifferential();
    Drive drive(std::move(kinematics), config, imu_);

    drive.setVelocity({0.8, 0.2, 0.5});
    drive.softStop();
}

TEST_F(DriveTest, HardStopCallsKinematicsHardStop) {
    auto kinematics = std::make_unique<testing::NiceMock<MockKinematics>>();
    kinematics->setupAsDifferential();
    EXPECT_CALL(*kinematics, hardStop()).Times(1);

    Drive drive(std::move(kinematics), config, imu_);
    drive.hardStop();
}