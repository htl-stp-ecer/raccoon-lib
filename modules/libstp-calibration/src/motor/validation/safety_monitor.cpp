#include "calibration/motor/validation/safety_monitor.hpp"
#include "calibration/motor/utils/time_utils.hpp"
#include "foundation/logging.hpp"

namespace libstp::calibration::validation
{
    SafetyMonitor::SafetyMonitor(const CalibrationConfig& config)
        : config_(config)
    {
    }

    bool SafetyMonitor::checkSafetyLimits(double total_distance_moved)
    {
        if (total_distance_moved > config_.max_test_distance_m) {
            LIBSTP_LOG_ERROR("Safety limit exceeded: distance={:.3f}m > max={:.3f}m",
                        total_distance_moved, config_.max_test_distance_m);
            return false;
        }
        return true;
    }

    bool SafetyMonitor::checkTimeout(double start_time, double calibration_start_time)
    {
        double elapsed = utils::getCurrentTime() - start_time;

        if (elapsed > config_.max_single_test_duration) {
            LIBSTP_LOG_ERROR("Individual test timeout: {:.2f}s > {:.2f}s",
                        elapsed, config_.max_single_test_duration);
            return true;
        }

        double total_elapsed = utils::getCurrentTime() - calibration_start_time;
        if (total_elapsed > config_.max_calibration_duration) {
            LIBSTP_LOG_ERROR("Total calibration timeout: {:.2f}s > {:.2f}s",
                        total_elapsed, config_.max_calibration_duration);
            return true;
        }

        return false;
    }
}
