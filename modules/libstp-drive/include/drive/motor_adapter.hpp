//
// Created by tobias on 9/8/25.
//

#pragma once
#include "hal/IMotor.hpp"
#include "calibration/motor/calibration_result.hpp"

namespace libstp::calibration
{
    struct CalibrationConfig;
}

namespace libstp::drive
{
    class MotorAdapter
    {
    public:
        explicit MotorAdapter(hal::motor::IMotor* motor);

        void setVelocity(double w_ref, double dt, bool* out_saturated = nullptr);

        [[nodiscard]] double getVelocity() const;

        void brake();

        /**
         * @brief Reset encoder position tracking to prevent stale deltas
         *
         * Call this when resetting odometry to invalidate the previous encoder position.
         * This prevents incorrect velocity calculations on the next update.
         */
        void resetEncoderTracking();

        hal::motor::IMotor& motor();
        [[nodiscard]] const hal::motor::IMotor& motor() const;

        void updateEncoderVelocity(double dt);

        // Calibration methods
        calibration::CalibrationResult calibrate(const calibration::CalibrationConfig& config);
        calibration::CalibrationResult calibrate(); // Overload with default config

    private:
        hal::motor::IMotor* motor_{nullptr};

        mutable double w_meas_filt_{0.0};
        long long pos_prev_{0};
        bool pos_prev_init_{false};
    };
}
