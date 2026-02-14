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
     * Supports asymmetric acceleration/deceleration rates.
     */
    class TrapezoidalProfile
    {
    public:
        struct Constraints
        {
            double max_velocity{1.0};      // Maximum velocity
            double max_acceleration{2.0};  // Maximum acceleration rate
            double max_deceleration{0.0};  // Maximum deceleration rate (0 = use max_acceleration)

            double effectiveDeceleration() const
            {
                return (max_deceleration > 0.0) ? max_deceleration : max_acceleration;
            }
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
            const double a_dec = constraints_.effectiveDeceleration();

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
                const double decel = -sign * a_dec;
                state.velocity = sign * cruise_velocity_ + decel * decel_t;
                state.position = decel_start_position_ + sign * cruise_velocity_ * decel_t + 0.5 * decel * decel_t * decel_t;
            }

            return state;
        }

        /**
         * Compute the next setpoint along a trapezoidal trajectory from current toward goal.
         *
         * Analytical approach (ported from FRC WPILib TrapezoidProfile::Calculate):
         * each call recomputes the full trapezoidal profile from current state to
         * goal, then evaluates it at exactly t=dt.  This is analytically exact --
         * no discretisation overshoot regardless of timestep.
         *
         * Supports asymmetric acceleration/deceleration rates.
         *
         * @param dt Time step (seconds)
         * @param current Current profiled state (position, velocity)
         * @param goal Desired goal state (position, velocity)
         * @param constraints Velocity and acceleration limits
         * @return The exact setpoint state after dt
         */
        static State calculate(double dt, const State& current, const State& goal,
                               const Constraints& constraints)
        {
            const double max_v = constraints.max_velocity;
            const double a_acc = constraints.max_acceleration;
            const double a_dec = constraints.effectiveDeceleration();

            // Direction: flip so goal is always "ahead" of current
            const int dir = (current.position > goal.position) ? -1 : 1;

            // Work in directed frame
            double c_pos = current.position * dir;
            double c_vel = current.velocity * dir;
            const double g_pos = goal.position * dir;
            const double g_vel = goal.velocity * dir;

            // Clamp current velocity to constraints
            if (std::abs(c_vel) > max_v)
                c_vel = std::copysign(max_v, c_vel);

            // Treat as a full zero-to-zero profile, then trim the portions
            // before current velocity and after goal velocity.
            // Acceleration side uses a_acc, deceleration side uses a_dec.
            const double cutoff_begin = c_vel / a_acc;
            const double cutoff_dist_begin = cutoff_begin * cutoff_begin * a_acc * 0.5;

            const double cutoff_end = g_vel / a_dec;
            const double cutoff_dist_end = cutoff_end * cutoff_end * a_dec * 0.5;

            const double full_trapezoid_dist =
                cutoff_dist_begin + (g_pos - c_pos) + cutoff_dist_end;

            double accel_time = max_v / a_acc;
            double decel_time = max_v / a_dec;
            double full_speed_dist =
                full_trapezoid_dist - 0.5 * a_acc * accel_time * accel_time
                                    - 0.5 * a_dec * decel_time * decel_time;

            double cruise_v = max_v;

            // Triangular profile if we can't reach full speed
            if (full_speed_dist < 0.0)
            {
                // v_peak^2 = 2 * d * a_acc * a_dec / (a_acc + a_dec)
                const double v_peak_sq = 2.0 * std::max(0.0, full_trapezoid_dist)
                                       * a_acc * a_dec / (a_acc + a_dec);
                cruise_v = std::sqrt(std::max(0.0, v_peak_sq));
                accel_time = cruise_v / a_acc;
                decel_time = cruise_v / a_dec;
                full_speed_dist = 0.0;
            }

            const double end_accel = accel_time - cutoff_begin;
            const double end_full_speed =
                end_accel + (cruise_v > 1e-10 ? full_speed_dist / cruise_v : 0.0);
            const double end_decel = end_full_speed + decel_time - cutoff_end;

            double r_pos, r_vel;

            if (dt < end_accel)
            {
                // Acceleration phase
                r_vel = c_vel + dt * a_acc;
                r_pos = c_pos + (c_vel + dt * a_acc * 0.5) * dt;
            }
            else if (dt < end_full_speed)
            {
                // Cruise phase
                r_vel = cruise_v;
                r_pos = c_pos
                      + (c_vel + end_accel * a_acc * 0.5) * end_accel
                      + cruise_v * (dt - end_accel);
            }
            else if (dt <= end_decel)
            {
                // Deceleration phase (computed backwards from goal -- exact)
                const double time_left = end_decel - dt;
                r_vel = g_vel + time_left * a_dec;
                r_pos = g_pos - (g_vel + time_left * a_dec * 0.5) * time_left;
            }
            else
            {
                // Profile complete within this timestep
                r_pos = g_pos;
                r_vel = g_vel;
            }

            // Convert back from directed frame
            return State{r_pos * dir, r_vel * dir};
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

            const double a_acc = constraints_.max_acceleration;
            const double a_dec = constraints_.effectiveDeceleration();

            // Max velocity considering asymmetric accel/decel:
            // v_peak = sqrt(2 * d * a_acc * a_dec / (a_acc + a_dec))
            const double max_reachable_velocity = std::sqrt(
                2.0 * abs_distance * a_acc * a_dec / (a_acc + a_dec));
            cruise_velocity_ = std::min(constraints_.max_velocity, max_reachable_velocity);

            // Time to accelerate to cruise velocity
            accel_time_ = cruise_velocity_ / a_acc;
            const double decel_time = cruise_velocity_ / a_dec;

            // Distance covered during acceleration
            const double accel_distance = 0.5 * a_acc * accel_time_ * accel_time_;

            // Distance covered during deceleration (may differ from accel)
            const double decel_distance = 0.5 * a_dec * decel_time * decel_time;

            // Distance covered during cruise
            const double cruise_distance = abs_distance - accel_distance - decel_distance;

            if (cruise_distance < 0.0)
            {
                // Triangular profile (no cruise phase)
                cruise_time_ = 0.0;
                cruise_velocity_ = max_reachable_velocity;
                accel_time_ = cruise_velocity_ / a_acc;

                const double new_accel_distance = 0.5 * a_acc * accel_time_ * accel_time_;
                accel_end_position_ = initial_.position + sign * new_accel_distance;
                decel_start_position_ = accel_end_position_;
            }
            else
            {
                // Trapezoidal profile (has cruise phase)
                cruise_time_ = cruise_distance / cruise_velocity_;

                accel_end_position_ = initial_.position + sign * accel_distance;
                decel_start_position_ = initial_.position + sign * (accel_distance + cruise_distance);
            }

            // Total time with asymmetric decel
            total_time_ = accel_time_ + cruise_time_ + cruise_velocity_ / a_dec;
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
