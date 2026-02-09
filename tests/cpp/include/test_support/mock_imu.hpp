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
        MOCK_METHOD(float, getHeading, (), (override));
        MOCK_METHOD(void, getLinearAcceleration, (float*), (override));
        MOCK_METHOD(void, getIntegratedVelocity, (float*), (override));
        MOCK_METHOD(void, resetIntegratedVelocity, (), (override));
        MOCK_METHOD(void, getLinearAcceleration, (float* linear_accel), (override));
        MOCK_METHOD(bool, waitForReady, (int timeout_ms), (override));

        // Simulation helpers
        void setHeading(float heading_rad) {
            ON_CALL(*this, getHeading()).WillByDefault(testing::Return(heading_rad));
        }

        void setReady(bool ready) {
            ON_CALL(*this, waitForReady(testing::_)).WillByDefault(testing::Return(ready));
        }

        // Default setup for common test scenarios
        void setupDefaults() {
            setReady(true);
            setHeading(0.0f);
        }
    };
}
