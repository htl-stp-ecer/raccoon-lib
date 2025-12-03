#pragma once

#include <vector>

namespace libstp::calibration::data
{
    struct VelocityDataPoint
    {
        double time;
        double velocity;
        double command;
    };

    struct VelocityProfile
    {
        std::vector<VelocityDataPoint> data;
        double mean_velocity{0.0};
        double std_dev{0.0};
    };
}
