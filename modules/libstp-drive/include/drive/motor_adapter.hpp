//
// Created by tobias on 9/8/25.
//

#pragma once
#include "hal/Motor.hpp"
#include "velocity_controller.hpp"
#include "calibration/calibration_result.hpp"

namespace libstp::calibration
{
    struct CalibrationConfig;
}

namespace libstp::drive
{
    class MotorAdapter
    {
    public:
        explicit MotorAdapter(hal::motor::Motor* motor);

        void setVelocityWithAccel(double w_ref, double a_ref, double dt, bool* out_saturated);

        void setVelocity(double w_ref, double dt);

        void setPercent(double percent);

        [[nodiscard]] double getVelocity() const;

        [[nodiscard]] int getRawPercent() const;

        void resetController();
        void brake();

        /**
         * @brief Reset encoder position tracking to prevent stale deltas
         *
         * Call this when resetting odometry to invalidate the previous encoder position.
         * This prevents incorrect velocity calculations on the next update.
         */
        void resetEncoderTracking();

        hal::motor::Motor& motor();
        [[nodiscard]] const hal::motor::Motor& motor() const;

        void updateEncoderVelocity(double dt);

        // Calibration methods
        calibration::CalibrationResult calibrate(const calibration::CalibrationConfig& config);
        calibration::CalibrationResult calibrate(); // Overload with default config
        void updateCalibration(const foundation::MotorCalibration& cal);

        // Direct access to controller for calibration
        VelocityController& getController() { return controller_; }
        [[nodiscard]] const VelocityController& getController() const { return controller_; }

    private:
        hal::motor::Motor* motor_{nullptr};
        VelocityController controller_;

        mutable double w_meas_filt_{0.0};
        long long pos_prev_{0};
        bool pos_prev_init_{false};

        double last_u_cmd_{0.0};
    };
}
