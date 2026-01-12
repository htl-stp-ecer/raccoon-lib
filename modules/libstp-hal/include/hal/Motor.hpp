//
// Created by tobias on 6/1/25.
//
#pragma once

#ifdef SAFETY_CHECKS_ENABLED
#include <set>
#endif
#include "foundation/motor.hpp"
#include "hal/IMotor.hpp"

namespace libstp::hal::motor
{
    class Motor : public IMotor
    {
#ifdef SAFETY_CHECKS_ENABLED
        static inline std::set<int> used_motor_ports{};

        static void registerMotorPort(int port);

        static void unregisterMotorPort(int port);
#endif

    public:
        explicit Motor(int port, bool inverted, const foundation::MotorCalibration& calibration = {});

        ~Motor() override;

        void setSpeed(int percent) override;
        [[nodiscard]] int getPosition() const override;

        void brake() override;

        [[nodiscard]] const foundation::MotorCalibration& getCalibration() const override;
        [[nodiscard]] int getPort() const override { return port_; }
        [[nodiscard]] bool isInverted() const override { return inverted_; }

        static void disableAll();

    private:
        int port_;
        bool inverted_;
        foundation::MotorCalibration calibration_;
    };
}
