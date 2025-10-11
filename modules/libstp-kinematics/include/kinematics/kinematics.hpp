//
// Created by tobias on 9/8/25.
//

#pragma once
#include <cstddef>
#include <vector>

#include "foundation/types.hpp"

namespace libstp::kinematics
{
    struct MotorCommands
    {
        std::vector<double> wheel_velocities{};
        bool saturated_any{false};
        std::uint32_t saturation_mask{0};
    };

    struct IKinematics
    {
        virtual ~IKinematics() = default;
        [[nodiscard]] virtual std::size_t wheelCount() const = 0;

        virtual void applyCommand(const foundation::ChassisCmd& cmd, double dt) = 0;

        [[nodiscard]] virtual foundation::ChassisState estimateState() const = 0;

        virtual void hardStop() = 0;
    };
}
