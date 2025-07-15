//
// Created by tobias on 1/12/25.
//

#pragma once

#include "datatype/conditions.hpp"

namespace libstp::motion
{
    struct DifferentialDriveState;
}

namespace libstp::device::omni_wheeled::datatype
{
    class ForwardDistanceConditionalResult final : public libstp::datatype::DistanceConditionalResult
    {
    public:
        explicit ForwardDistanceConditionalResult(const float targetCm)
            : DistanceConditionalResult(targetCm)
        {
        }

        void update(motion::DifferentialDriveState& state) override;
    };

    class SideDistanceConditionalResult final : public libstp::datatype::DistanceConditionalResult
    {
    public:
        explicit SideDistanceConditionalResult(const float targetCm)
            : DistanceConditionalResult(targetCm)
        {
        }

        void update(motion::DifferentialDriveState& state) override;
    };
}
