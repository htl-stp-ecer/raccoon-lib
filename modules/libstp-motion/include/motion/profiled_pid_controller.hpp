#pragma once

#include <algorithm>
#include <cmath>

#include "motion/trapezoidal_profile.hpp"

namespace libstp::motion
{
    /**
     * PID controller whose setpoint is constrained by a trapezoidal motion profile.
     *
     * Ported from FRC WPILib's ProfiledPIDController. Each update cycle:
     * 1. The profile advances the setpoint toward the goal
     * 2. PID computes tracking error (setpoint - measurement)
     * 3. Output = velocity_ff * setpoint.velocity + PID(tracking_error)
     *
     * The velocity feedforward provides the baseline command from the profile,
     * while the PID corrects tracking errors. This is essential for velocity-
     * commanding systems (like ours) where the PID output is a velocity, not
     * a motor voltage.
     */
    class ProfiledPIDController
    {
    public:
        using State = TrapezoidalProfile::State;
        using Constraints = TrapezoidalProfile::Constraints;

        struct Config
        {
            double kp{2.0};
            double ki{0.0};
            double kd{0.3};
            double velocity_ff{1.0};           // Feedforward gain on profile velocity (1.0 = full FF)
            double derivative_lpf_alpha{0.3};  // Low-pass filter on derivative
            double integral_max{10.0};         // Anti-windup clamp
            double integral_deadband{0.01};    // Don't integrate below this error
        };

        ProfiledPIDController(Config pid_config, Constraints profile_constraints)
            : cfg_(pid_config)
            , constraints_(profile_constraints)
        {
        }

        void setGoal(double position) { goal_ = {position, 0.0}; }
        void setGoal(State goal) { goal_ = goal; }
        [[nodiscard]] State getGoal() const { return goal_; }
        [[nodiscard]] State getSetpoint() const { return setpoint_; }
        [[nodiscard]] const Constraints& getConstraints() const { return constraints_; }

        void setConstraints(Constraints constraints) { constraints_ = constraints; }

        void enableContinuousInput(double min_input, double max_input)
        {
            continuous_input_ = true;
            min_input_ = min_input;
            max_input_ = max_input;
        }

        void disableContinuousInput() { continuous_input_ = false; }

        void reset(double position, double velocity = 0.0)
        {
            setpoint_ = {position, velocity};
            integral_ = 0.0;
            prev_error_ = 0.0;
            filtered_derivative_ = 0.0;
            first_update_ = true;
        }

        void reset(State state) { reset(state.position, state.velocity); }

        /**
         * Compute the next control output.
         *
         * @param measurement Current measured position
         * @param dt Time step (seconds)
         * @return Velocity command = velocity_ff * setpoint.velocity + PID(tracking_error)
         */
        double calculate(double measurement, double dt)
        {
            if (dt <= 0.0) return 0.0;

            // Handle continuous input wrapping (e.g., angles)
            if (continuous_input_)
            {
                const double error_bound = (max_input_ - min_input_) / 2.0;
                const double goal_min = inputModulus(
                    goal_.position - measurement, -error_bound, error_bound);
                const double setpoint_min = inputModulus(
                    setpoint_.position - measurement, -error_bound, error_bound);

                goal_.position = goal_min + measurement;
                setpoint_.position = setpoint_min + measurement;
            }

            // Advance profile: setpoint moves toward goal
            setpoint_ = TrapezoidalProfile::calculate(dt, setpoint_, goal_, constraints_);

            // Tracking error: where profile says we should be vs where we are
            double error = setpoint_.position - measurement;

            // Wrap tracking error for continuous input
            if (continuous_input_)
            {
                const double error_bound = (max_input_ - min_input_) / 2.0;
                error = inputModulus(error, -error_bound, error_bound);
            }

            // Derivative (filtered)
            double d_term = 0.0;
            if (!first_update_)
            {
                const double raw_deriv = (error - prev_error_) / dt;
                filtered_derivative_ = cfg_.derivative_lpf_alpha * raw_deriv
                    + (1.0 - cfg_.derivative_lpf_alpha) * filtered_derivative_;
                d_term = cfg_.kd * filtered_derivative_;
            }
            else
            {
                first_update_ = false;
            }

            // Integral with anti-windup
            if (std::abs(error) > cfg_.integral_deadband)
            {
                integral_ += error * dt;
                integral_ = std::clamp(integral_, -cfg_.integral_max, cfg_.integral_max);
            }

            const double pid_output = cfg_.kp * error + cfg_.ki * integral_ + d_term;
            prev_error_ = error;

            // Feedforward from profile velocity + PID correction
            return cfg_.velocity_ff * setpoint_.velocity + pid_output;
        }

        /**
         * Check if the profile's setpoint has reached the goal.
         * Note: this does NOT mean the robot has reached the goal —
         * use separate settling detection for that.
         */
        [[nodiscard]] bool profileComplete() const
        {
            return std::abs(goal_.position - setpoint_.position) < 1e-6
                && std::abs(goal_.velocity - setpoint_.velocity) < 1e-6;
        }

    private:
        static double inputModulus(double input, double min, double max)
        {
            const double range = max - min;
            double mod = std::fmod(input - min, range);
            if (mod < 0.0) mod += range;
            return mod + min;
        }

        Config cfg_;
        Constraints constraints_;
        State goal_{};
        State setpoint_{};

        bool continuous_input_{false};
        double min_input_{0.0};
        double max_input_{0.0};

        // PID state
        double integral_{0.0};
        double prev_error_{0.0};
        double filtered_derivative_{0.0};
        bool first_update_{true};
    };
}
