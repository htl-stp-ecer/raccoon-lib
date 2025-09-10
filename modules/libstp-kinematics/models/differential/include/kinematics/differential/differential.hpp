//
// Created by tobias on 9/8/25.
//

#pragma once
#include "kinematics/kinematics.hpp"

namespace libstp::kinematics::differential
{
    class DifferentialKinematics : public IKinematics
    {
    private:
        double m_wheelbase;
        double m_wheelRadius;

    public:
        DifferentialKinematics(double wheelbase, double wheelRadius);
        ~DifferentialKinematics() override = default;

        [[nodiscard]] std::size_t wheelCount() const override;
        [[nodiscard]] std::vector<double> inverse(const foundation::ChassisCmd& cmd) const override;
        [[nodiscard]] foundation::ChassisState forward(const std::vector<double>& w) const override;
    };
}