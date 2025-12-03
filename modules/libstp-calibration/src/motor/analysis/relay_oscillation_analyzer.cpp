#include "calibration/motor/analysis/relay_oscillation_analyzer.hpp"
#include "calibration/motor/utils/math_utils.hpp"
#include <algorithm>
#include <cmath>

namespace libstp::calibration::analysis
{
    RelayParams analyzeOscillations(
        const data::VelocityProfile& profile,
        double relay_amplitude)
    {
        RelayParams params{0.0, 0.0, 0.0, 0};

        if (profile.data.size() < 10) {
            return params;
        }

        // Find zero crossings
        std::vector<double> crossing_times;
        double last_velocity = profile.data[0].velocity;

        for (size_t i = 1; i < profile.data.size(); ++i) {
            double v = profile.data[i].velocity;
            if ((last_velocity > 0 && v < 0) || (last_velocity < 0 && v > 0)) {
                crossing_times.push_back(profile.data[i].time);
            }
            last_velocity = v;
        }

        if (crossing_times.size() < 4) {
            return params;
        }

        // Calculate period (average time between alternate crossings)
        std::vector<double> periods;
        for (size_t i = 2; i < crossing_times.size(); ++i) {
            periods.push_back(crossing_times[i] - crossing_times[i-2]);
        }
        params.Tu = utils::getMeanValue(periods);
        params.cycles = crossing_times.size() / 2;

        // Calculate amplitude (peak-to-peak velocity)
        double max_v = profile.data[0].velocity;
        double min_v = profile.data[0].velocity;
        for (const auto& point : profile.data) {
            max_v = std::max(max_v, point.velocity);
            min_v = std::min(min_v, point.velocity);
        }
        params.amplitude = (max_v - min_v) / 2.0;

        // Calculate ultimate gain Ku = 4*relay_amplitude / (π * oscillation_amplitude)
        params.Ku = (4.0 * relay_amplitude) / (3.14159265359 * params.amplitude);

        return params;
    }
}
