//
// Created by tobias on 9/8/25.
//

#pragma once
#include "kinematics/kinematics.hpp"

namespace libstp::kinematics::mecanum
{
    class MecanumKinematics : public IKinematics
    {
    private:
        double m_wheelbase;
        double m_trackWidth;
        double m_wheelRadius;

    public:
        MecanumKinematics(double wheelbase, double trackWidth, double wheelRadius);
        ~MecanumKinematics() override = default;

        [[nodiscard]] std::size_t wheelCount() const override;
        [[nodiscard]] std::vector<double> inverse(const foundation::ChassisCmd& cmd) const override;
        [[nodiscard]] foundation::ChassisState forward(const std::vector<double>& w) const override;
    };
}