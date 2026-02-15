#pragma once

#include <algorithm>
#include <cmath>

#include "foundation/pid.hpp"
#include "motion/trapezoidal_profile.hpp"

namespace libstp::motion
{
    /**
     * PID controller whose setpoint is constrained by a trapezoidal motion profile.
     *
     * Each update cycle:
     * 1. The profile advances the setpoint toward the goal
     * 2. PID computes tracking error (setpoint - measurement)
     * 3. Output = velocity_ff * setpoint.velocity + PID(tracking_error)
     */
    class ProfiledPIDController
    {
    public:
        using State = TrapezoidalProfile::State;
        using Constraints = TrapezoidalProfile::Constraints;

        struct Config
        {
            foundation::PidConfig pid{};
            double velocity_ff{1.0};
        };

        ProfiledPIDController(Config config, Constraints profile_constraints)
            : cfg_(config)
            , constraints_(profile_constraints)
            , pid_(config.pid)
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
            pid_.reset();
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

            const double pid_output = pid_.update(error, dt);

            // Feedforward from profile velocity + PID correction
            return cfg_.velocity_ff * setpoint_.velocity + pid_output;
        }

        /**
         * Check if the profile's setpoint has reached the goal.
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
        foundation::PidController pid_;
        State goal_{};
        State setpoint_{};

        bool continuous_input_{false};
        double min_input_{0.0};
        double max_input_{0.0};
    };
}
