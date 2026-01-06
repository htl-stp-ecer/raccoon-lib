#pragma once

#include <string>

#include "drive/drive.hpp"
#include "odometry/odometry.hpp"

namespace libstp::calibration
{
    struct MotionCalibrationConfig
    {
        double control_rate_hz{100.0};
        double relay_yaw_rate{0.5};           // rad/s for relay test
        double forward_speed_mps{0.2};        // m/s forward speed during test
        int min_heading_cycles{3};            // minimum oscillation cycles required
        int settle_skip_cycles{1};            // initial cycles to skip
        double max_heading_autotune_time{30.0}; // seconds
        double min_peak_separation{0.1};      // seconds between detected peaks
        double min_heading_amplitude{0.01};   // rad, minimum oscillation amplitude
    };

    struct DriveStraightAutotuneResult
    {
        bool success{false};
        double kp{0.0};
        double ki{0.0};
        double kd{0.0};
        double Ku{0.0};
        double Tu{0.0};
        double amplitude{0.0};
        double relay_output{0.0};
        int cycles{0};
        std::string error;
    };

    /**
     * @brief Relay-feedback autotuner for the heading loop while driving straight.
     *
     * Implements an Åström–Hägglund style relay test: replaces the heading PID with
     * an on/off yaw command, measures the resulting oscillation amplitude/period,
     * and computes PID gains from Ku/Tu using conservative Ziegler–Nichols factors.
     */
    class DriveStraightAutotuner
    {
    public:
        DriveStraightAutotuner(drive::Drive& drive,
                               odometry::IOdometry& odom,
                               MotionCalibrationConfig cfg);

        DriveStraightAutotuneResult autotuneHeading();

    private:
        drive::Drive& drive_;
        odometry::IOdometry& odom_;
        MotionCalibrationConfig cfg_;
    };
}
