#include "calibration/motor/utils/math_utils.hpp"
#include <numeric>
#include <algorithm>

namespace libstp::calibration::utils
{
    double getMeanValue(const std::vector<double>& values)
    {
        if (values.empty()) return 0.0;
        return std::accumulate(values.begin(), values.end(), 0.0) / values.size();
    }

    double getStdDev(const std::vector<double>& values, double mean)
    {
        if (values.size() < 2) return 0.0;
        double sum_sq_diff = 0.0;
        for (double v : values) {
            double diff = v - mean;
            sum_sq_diff += diff * diff;
        }
        return std::sqrt(sum_sq_diff / (values.size() - 1));
    }

    double clamp(double value, double min_val, double max_val)
    {
        return std::max(min_val, std::min(value, max_val));
    }

    LinearRegressionResult linearFit(
        const std::vector<double>& x,
        const std::vector<double>& y)
    {
        LinearRegressionResult result{0.0, 0.0, 0.0};

        if (x.size() != y.size() || x.size() < 2) {
            return result;
        }

        const size_t n = x.size();
        double sum_x = 0.0, sum_y = 0.0, sum_xy = 0.0, sum_xx = 0.0;

        for (size_t i = 0; i < n; ++i) {
            sum_x += x[i];
            sum_y += y[i];
            sum_xy += x[i] * y[i];
            sum_xx += x[i] * x[i];
        }

        double mean_x = sum_x / n;
        double mean_y = sum_y / n;

        double numerator = sum_xy - n * mean_x * mean_y;
        double denominator = sum_xx - n * mean_x * mean_x;

        if (std::abs(denominator) < 1e-10) {
            return result;
        }

        result.slope = numerator / denominator;
        result.intercept = mean_y - result.slope * mean_x;

        // Calculate R²
        double ss_tot = 0.0, ss_res = 0.0;
        for (size_t i = 0; i < n; ++i) {
            double y_pred = result.slope * x[i] + result.intercept;
            ss_res += (y[i] - y_pred) * (y[i] - y_pred);
            ss_tot += (y[i] - mean_y) * (y[i] - mean_y);
        }

        result.r_squared = (ss_tot > 1e-10) ? 1.0 - (ss_res / ss_tot) : 0.0;

        return result;
    }
}
