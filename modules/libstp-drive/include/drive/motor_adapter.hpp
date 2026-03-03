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
    /**
     * Adapter from `IMotor` to drive-module velocity conventions.
     *
     * The adapter translates wheel angular velocity targets into the firmware's
     * BEMF units and maintains a filtered encoder-based velocity estimate for
     * higher-level kinematics.
     */
    class MotorAdapter
    {
    public:
        /** @param motor Non-owning pointer to the wrapped motor interface. */
        explicit MotorAdapter(hal::motor::IMotor* motor);

        /**
         * Send a wheel angular velocity target in rad/s.
         *
         * @param w_ref Desired wheel speed in rad/s.
         * @param dt Control-loop time step used to refresh encoder velocity.
         * @param out_saturated Optional saturation flag. The current adapter
         *                      never reports saturation and always writes false.
         */
        void setVelocity(double w_ref, double dt, bool* out_saturated = nullptr);

        /** Return the low-pass filtered encoder velocity estimate in rad/s. */
        [[nodiscard]] double getVelocity() const;

        /** Command an immediate motor brake. */
        void brake();

        /**
         * @brief Reset encoder position tracking to prevent stale deltas
         *
         * Call this when resetting odometry to invalidate the previous encoder position.
         * This prevents incorrect velocity calculations on the next update.
         */
        void resetEncoderTracking();

        /** Access the wrapped motor interface. */
        hal::motor::IMotor& motor();
        [[nodiscard]] const hal::motor::IMotor& motor() const;

        /** Refresh the filtered encoder-velocity estimate from the latest position delta. */
        void updateEncoderVelocity(double dt);

        /** Run motor calibration with an explicit config. */
        calibration::CalibrationResult calibrate(const calibration::CalibrationConfig& config);

        /** Run motor calibration with the default config. */
        calibration::CalibrationResult calibrate(); // Overload with default config

    private:
        hal::motor::IMotor* motor_{nullptr};

        mutable double w_meas_filt_{0.0};
        long long pos_prev_{0};
        bool pos_prev_init_{false};
    };
}
