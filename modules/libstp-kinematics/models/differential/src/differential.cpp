//
// Created by tobias on 9/8/25.
//

#include "kinematics/differential/differential.hpp"

#include "foundation/types.hpp"


namespace libstp::kinematics::differential
{
    DifferentialKinematics::DifferentialKinematics(double wheelbase, double wheelRadius)
        : m_wheelbase(wheelbase), m_wheelRadius(wheelRadius)
    {
    }

    std::size_t DifferentialKinematics::wheelCount() const
    {
        return 2;
    }

    std::vector<double> DifferentialKinematics::inverse(const foundation::ChassisCmd& cmd) const
    {
        // Dummy implementation
        return {0.0, 0.0};
    }

    foundation::ChassisState DifferentialKinematics::forward(const std::vector<double>& w) const
    {
        // Dummy implementation
        return {};
    }
}
