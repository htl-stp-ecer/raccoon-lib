#pragma once

#include <algorithm>
#include <cmath>

namespace libstp::motion
{
    /**
     * Generates a trapezoidal velocity profile for smooth motion.
     *
     * The profile has three phases:
     * 1. Acceleration: ramp up from 0 to max_velocity
     * 2. Cruise: maintain max_velocity
     * 3. Deceleration: ramp down from max_velocity to 0
     *
     * For short distances, the profile may be triangular (no cruise phase).
     */
    class TrapezoidalProfile
    {
    public:
        struct Constraints
        {
            double max_velocity{1.0};      // Maximum velocity
            double max_acceleration{2.0};  // Maximum acceleration/deceleration
        };

        struct State
        {
            double position{0.0};
            double velocity{0.0};
        };

        /**
         * Initialize the profile.
         * @param initial Initial state (usually {0, 0})
         * @param target Target position
         * @param constraints Velocity and acceleration limits
         */
        TrapezoidalProfile(const State& initial, double target, const Constraints& constraints)
            : initial_(initial)
            , target_(target)
            , constraints_(constraints)
        {
            compute_profile();
        }

        /**
         * Get the setpoint at a given time.
         * @param elapsed_time Time since profile started (seconds)
         * @return Desired state at this time
         */
        State getSetpoint(double elapsed_time) const
        {
            const double t = std::max(0.0, elapsed_time);

            if (t >= total_time_)
            {
                // Profile complete, return target
                return State{target_, 0.0};
            }

            const double sign = (target_ >= initial_.position) ? 1.0 : -1.0;
            const double abs_distance = std::abs(target_ - initial_.position);

            State state;

            if (t <= accel_time_)
            {
                // Acceleration phase
                const double accel = sign * constraints_.max_acceleration;
                state.velocity = initial_.velocity + accel * t;
                state.position = initial_.position + initial_.velocity * t + 0.5 * accel * t * t;
            }
            else if (t <= accel_time_ + cruise_time_)
            {
                // Cruise phase
                const double cruise_t = t - accel_time_;
                state.velocity = sign * cruise_velocity_;
                state.position = accel_end_position_ + state.velocity * cruise_t;
            }
            else
            {
                // Deceleration phase
                const double decel_t = t - accel_time_ - cruise_time_;
                const double decel = -sign * constraints_.max_acceleration;
                state.velocity = sign * cruise_velocity_ + decel * decel_t;
                state.position = decel_start_position_ + sign * cruise_velocity_ * decel_t + 0.5 * decel * decel_t * decel_t;
            }

            return state;
        }

        /**
         * Check if the profile is complete.
         */
        bool isComplete(double elapsed_time) const
        {
            return elapsed_time >= total_time_;
        }

        /**
         * Get the total time for the profile.
         */
        double getTotalTime() const
        {
            return total_time_;
        }

    private:
        void compute_profile()
        {
            const double distance = target_ - initial_.position;
            const double abs_distance = std::abs(distance);
            const double sign = (distance >= 0.0) ? 1.0 : -1.0;

            // Clamp max velocity to what's achievable given max acceleration and distance
            const double max_reachable_velocity = std::sqrt(constraints_.max_acceleration * abs_distance);
            cruise_velocity_ = std::min(constraints_.max_velocity, max_reachable_velocity);

            // Time to accelerate to cruise velocity
            accel_time_ = cruise_velocity_ / constraints_.max_acceleration;

            // Distance covered during acceleration
            const double accel_distance = 0.5 * constraints_.max_acceleration * accel_time_ * accel_time_;

            // Distance covered during deceleration (same as acceleration)
            const double decel_distance = accel_distance;

            // Distance covered during cruise
            const double cruise_distance = abs_distance - accel_distance - decel_distance;

            if (cruise_distance < 0.0)
            {
                // Triangular profile (no cruise phase)
                cruise_time_ = 0.0;
                cruise_velocity_ = max_reachable_velocity;
                accel_time_ = cruise_velocity_ / constraints_.max_acceleration;

                accel_end_position_ = initial_.position + sign * 0.5 * constraints_.max_acceleration * accel_time_ * accel_time_;
                decel_start_position_ = accel_end_position_;
            }
            else
            {
                // Trapezoidal profile (has cruise phase)
                cruise_time_ = cruise_distance / cruise_velocity_;

                accel_end_position_ = initial_.position + sign * accel_distance;
                decel_start_position_ = initial_.position + sign * (accel_distance + cruise_distance);
            }

            // Deceleration time (same as acceleration time for symmetric profile)
            const double decel_time = accel_time_;

            // Total time
            total_time_ = accel_time_ + cruise_time_ + decel_time;
        }

        State initial_;
        double target_;
        Constraints constraints_;

        // Computed profile parameters
        double accel_time_{0.0};
        double cruise_time_{0.0};
        double cruise_velocity_{0.0};
        double accel_end_position_{0.0};
        double decel_start_position_{0.0};
        double total_time_{0.0};
    };
}
