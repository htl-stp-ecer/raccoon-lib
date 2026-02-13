#pragma once

#include <gmock/gmock.h>
#include "hal/IIMU.hpp"

namespace libstp::test
{
    class MockIMU : public hal::imu::IIMU
    {
    public:
        MOCK_METHOD(void, read, (float* accel, float* gyro, float* magneto), (override));
        MOCK_METHOD(void, getAngularVelocity, (float* gyro), (override));
        MOCK_METHOD(void, calibrate, (), (override));
        MOCK_METHOD(Eigen::Quaternionf, getOrientation, (), (override));
        MOCK_METHOD(bool, waitForReady, (int timeout_ms), (override));

        // Simulation helpers
        void setOrientation(const Eigen::Quaternionf& q) {
            ON_CALL(*this, getOrientation()).WillByDefault(testing::Return(q));
        }

        void setOrientationFromYaw(double yaw_rad) {
            Eigen::Quaternionf q(Eigen::AngleAxisf(static_cast<float>(yaw_rad), Eigen::Vector3f::UnitZ()));
            setOrientation(q);
        }

        void setReady(bool ready) {
            ON_CALL(*this, waitForReady(testing::_)).WillByDefault(testing::Return(ready));
        }

        // Default setup for common test scenarios
        void setupDefaults() {
            setReady(true);
            setOrientationFromYaw(0.0);
        }
    };
}
