//
// Created by tobias on 9/8/25.
//

#include "kinematics/mecanum/mecanum.hpp"

#include "foundation/types.hpp"

namespace libstp::kinematics::mecanum
{
    MecanumKinematics::MecanumKinematics(double wheelbase, double trackWidth, double wheelRadius)
        : m_wheelbase(wheelbase), m_trackWidth(trackWidth), m_wheelRadius(wheelRadius)
    {
    }

    std::size_t MecanumKinematics::wheelCount() const
    {
        return 4;
    }

    std::vector<double> MecanumKinematics::inverse(const foundation::ChassisCmd& cmd) const
    {
        // Dummy implementation
        return {0.0, 0.0, 0.0, 0.0};
    }

    foundation::ChassisState MecanumKinematics::forward(const std::vector<double>& w) const
    {
        // Dummy implementation
        return {};
    }
}
