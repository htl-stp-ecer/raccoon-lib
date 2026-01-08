//
// Created by tobias on 9/5/25.
//

#pragma once
#include <memory>

#include "kinematics/kinematics.hpp"
#include "foundation/types.hpp"
#include "drive/limits.hpp"
#include "foundation/config.hpp"

namespace libstp::drive
{
    class Drive final
    {
    public:
        Drive(std::unique_ptr<kinematics::IKinematics> kinematics,
              const MotionLimits& chassis_lim);

        void setVelocity(const foundation::ChassisVelocity& v_body);
        [[nodiscard]] kinematics::MotorCommands update(double dt) const;

        [[nodiscard]] foundation::ChassisVelocity estimateState() const;
        [[nodiscard]] std::size_t wheelCount() const;

        void softStop();
        void hardStop();

        [[nodiscard]] kinematics::IKinematics& getKinematics() { return *kinematics_; }
        [[nodiscard]] const kinematics::IKinematics& getKinematics() const { return *kinematics_; }

    private:
        std::unique_ptr<kinematics::IKinematics> kinematics_;

        MotionLimits chassis_lim_{};

        foundation::ChassisVelocity desired_{};
    };
}
