#pragma once

#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <memory>
#include "test_support/mock_kinematics.hpp"
#include "test_support/mock_odometry.hpp"
#include "test_support/mock_motor.hpp"
#include "test_support/mock_imu.hpp"

namespace libstp::test
{
    /**
     * Fixture for testing motion controllers
     */
    class MotionTestFixture : public ::testing::Test
    {
    protected:
        void SetUp() override {
            mock_kinematics_ = std::make_shared<testing::NiceMock<MockKinematics>>();
            mock_odometry_ = std::make_shared<testing::NiceMock<MockOdometry>>();

            // Setup default behaviors
            mock_kinematics_->setupAsDifferential();
            mock_odometry_->setupDefaults();
        }

        std::shared_ptr<testing::NiceMock<MockKinematics>> mock_kinematics_;
        std::shared_ptr<testing::NiceMock<MockOdometry>> mock_odometry_;
    };

    /**
     * Fixture for testing kinematics with mock motors
     */
    class KinematicsTestFixture : public ::testing::Test
    {
    protected:
        void SetUp() override {
            // Create mock motors with default calibration
            for (int i = 0; i < 4; ++i) {
                motors_[i] = std::make_unique<testing::NiceMock<MockMotor>>(i, false);
                foundation::MotorCalibration cal{};
                motors_[i]->setCalibration(cal);
                motors_[i]->simulatePosition(0);
            }
        }

        std::unique_ptr<testing::NiceMock<MockMotor>> motors_[4];
    };

    /**
     * Fixture for testing odometry with mock IMU and kinematics
     */
    class OdometryTestFixture : public ::testing::Test
    {
    protected:
        void SetUp() override {
            mock_imu_ = std::make_shared<testing::NiceMock<MockIMU>>();
            mock_kinematics_ = std::make_shared<testing::NiceMock<MockKinematics>>();

            mock_imu_->setupDefaults();
            mock_kinematics_->setupAsDifferential();
        }

        std::shared_ptr<testing::NiceMock<MockIMU>> mock_imu_;
        std::shared_ptr<testing::NiceMock<MockKinematics>> mock_kinematics_;
    };

    /**
     * Fixture for pure algorithm tests (no mocks needed)
     */
    class AlgorithmTestFixture : public ::testing::Test
    {
    protected:
        static constexpr double kDefaultDt = 0.01;  // 100Hz
        static constexpr double kTolerance = 1e-6;
    };
}
