#pragma once

#include <vector>
#include <cmath>

namespace libstp::calibration::utils
{
    double getMeanValue(const std::vector<double>& values);

    double getStdDev(const std::vector<double>& values, double mean);

    double clamp(double value, double min_val, double max_val);

    struct LinearRegressionResult
    {
        double slope{0.0};
        double intercept{0.0};
        double r_squared{0.0};
    };

    LinearRegressionResult linearFit(
        const std::vector<double>& x,
        const std::vector<double>& y
    );
}
