#pragma once

#include "calibration/motor/calibration_config.hpp"

namespace libstp::calibration::validation
{
    class SafetyMonitor
    {
    public:
        explicit SafetyMonitor(const CalibrationConfig& config);

        bool checkSafetyLimits(double total_distance_moved);
        bool checkTimeout(double start_time, double calibration_start_time);

    private:
        const CalibrationConfig& config_;
    };
}
