#pragma once

#include <gmock/gmock.h>
#include "hal/IMotor.hpp"

namespace libstp::test
{
    class MockMotor : public hal::motor::IMotor
    {
    public:
        MockMotor(int port = 0, bool inverted = false)
            : port_(port), inverted_(inverted) {}

        MOCK_METHOD(void, setSpeed, (int percent), (override));
        MOCK_METHOD(void, setVelocity, (int velocity), (override));
        MOCK_METHOD(void, moveToPosition, (int velocity, int goalPosition), (override));
        MOCK_METHOD(void, moveRelative, (int velocity, int deltaPosition), (override));
        MOCK_METHOD(int, getPosition, (), (const, override));
        MOCK_METHOD(bool, isDone, (), (const, override));
        MOCK_METHOD(void, brake, (), (override));
        MOCK_METHOD(void, off, (), (override));
        MOCK_METHOD(const foundation::MotorCalibration&, getCalibration, (), (const, override));
        MOCK_METHOD(int, getBemf, (), (const, override));

        [[nodiscard]] int getPort() const override { return port_; }
        [[nodiscard]] bool isInverted() const override { return inverted_; }

        // Simulation helpers
        void simulatePosition(int pos) {
            current_position_ = pos;
            ON_CALL(*this, getPosition()).WillByDefault(testing::Return(pos));
        }

        void advancePosition(int delta) {
            current_position_ += delta;
            ON_CALL(*this, getPosition()).WillByDefault(testing::Return(current_position_));
        }

        void setCalibration(const foundation::MotorCalibration& cal) {
            calibration_ = cal;
            ON_CALL(*this, getCalibration()).WillByDefault(testing::ReturnRef(calibration_));
        }

    private:
        int port_;
        bool inverted_;
        int current_position_{0};
        foundation::MotorCalibration calibration_{};
    };
}
