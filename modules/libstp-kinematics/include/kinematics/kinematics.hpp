//
// Created by tobias on 9/8/25.
//

#pragma once
#include <cstddef>
#include <vector>

#include "foundation/types.hpp"

namespace libstp::kinematics
{
    struct IKinematics
    {
        virtual ~IKinematics() = default;
        [[nodiscard]] virtual std::size_t wheelCount() const = 0;

        [[nodiscard]] virtual std::vector<double> inverse(const foundation::ChassisCmd& cmd) const = 0;

        [[nodiscard]] virtual foundation::ChassisState forward(const std::vector<double>& w) const = 0;
    };
}
