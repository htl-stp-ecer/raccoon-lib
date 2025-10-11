//
// Created by tobias on 9/5/25.
//

#pragma once
#include <memory>

#include "kinematics/kinematics.hpp"
#include "foundation/types.hpp"
#include "drive/limits.hpp"

namespace libstp::drive
{
    class Drive final
    {
    public:
        Drive(std::unique_ptr<kinematics::IKinematics> kinematics,
              const MotionLimits& chassis_lim);

        void setVelocity(const foundation::ChassisVel& v_body);
        void update(double dt) const;

        [[nodiscard]] foundation::ChassisState estimateState() const;
        [[nodiscard]] std::size_t wheelCount() const;

        void hardStop();

    private:
        std::unique_ptr<kinematics::IKinematics> kinematics_;

        MotionLimits chassis_lim_{};

        foundation::ChassisVel desired_{};
    };
}
