//
// Created by tobias on 9/5/25.
//

#pragma once
#include <memory>
#include <vector>

#include "rate_limiter.hpp"
#include "velocity_controller.hpp"
#include "motor_adapter.hpp"
#include "kinematics/kinematics.hpp"
#include "foundation/types.hpp"
#include "drive/limits.hpp"

namespace libstp::drive
{
    struct Achieved
    {
        foundation::ChassisVel body{};
        bool saturated_any{false};
        std::uint32_t kinematic_sat_mask{0};
        std::uint32_t actuator_sat_mask{0};
    };

    class Drive final
    {
    public:
        Drive(std::unique_ptr<kinematics::IKinematics> kinematics,
              const std::vector<hal::motor::Motor*>& motors);

        void setChassisLimits(const MotionLimits& lim);
        void setWheelLimits(const WheelLimits& lim);

        void setVelocity(const foundation::ChassisVel& v_body);
        Achieved update(double dt);

        [[nodiscard]] foundation::ChassisState estimateState() const;
        [[nodiscard]] std::size_t wheelCount() const;

        void stop(bool hard = false);

    private:
        struct Wheel
        {
            MotorAdapter adapter;
            RateLimiter limiter{0.0};
            double target_w{0.0};
        };

        std::unique_ptr<kinematics::IKinematics> kinematics_;
        std::vector<Wheel> wheels_;

        MotionLimits chassis_lim_{};
        WheelLimits wheel_lim_{};

        foundation::ChassisVel desired_{};
    };
}
