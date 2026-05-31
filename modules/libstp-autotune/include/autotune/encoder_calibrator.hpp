#pragma once

#include "autotune/types.hpp"
#include "drive/drive.hpp"
#include "hal/odometry.hpp"

namespace libstp::autotune
{
    /**
     * @brief Calibrate per-motor `ticks_to_rad` using the IMU heading as
     *        ground truth.
     *
     * The calibrator commands a slow in-place rotation, accumulates the IMU
     * heading change vs. the odometry-integrated heading change, and rescales
     * `MotorCalibration::ticks_to_rad` by `imu_angle / odom_angle` for every
     * motor.
     *
     * Because the same scale factor is applied to all motors, this only fixes
     * the systematic component of the ticks_to_rad assumption (e.g. wrong
     * gear-ratio constant). Individual per-motor variation requires a richer
     * kinematics model and is out of scope here.
     *
     * @note The caller is responsible for re-publishing the kinematics config
     *       to the STM32 (`TransportWriter::sendKinematicsConfig`) after a
     *       successful calibration — the calibrator only mutates the in-process
     *       `MotorCalibration` instances.
     */
    class EncoderCalibrator
    {
    public:
        EncoderCalibrator(drive::Drive& drive, odometry::IOdometry& odometry);

        [[nodiscard]] EncoderCalResult calibrate(const EncoderCalConfig& cfg) const;

    private:
        drive::Drive&        drive_;
        odometry::IOdometry& odometry_;
    };

} // namespace libstp::autotune
