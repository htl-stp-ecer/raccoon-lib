#pragma once

#include <map>
#include <string>
#include <tuple>
#include <vector>

#include "autotune/types.hpp"
#include "drive/drive.hpp"
#include "hal/odometry.hpp"
#include "motion/linear_motion.hpp"
#include "motion/motion_config.hpp"

namespace libstp::autotune
{
    /**
     * @brief Tunes motion PID gains (distance, lateral, heading) via Hooke-Jeeves
     *        coordinate descent.
     *
     * For each parameter the tuner:
     *  1. Runs a real closed-loop motion trial and scores the result by combining
     *     settle time, overshoot, and final error.
     *  2. Iteratively perturbs kp and kd, keeping improvements, shrinking step
     *     size on no-improvement until deltas drop below the minimum threshold.
     *
     * The `UnifiedMotionPidConfig` referenced on construction is mutated in-place
     * during every trial (the gains are written directly before each motion is
     * created) and, after tuning, retains the best-found gains.  Callers should
     * persist the config themselves after tune() returns.
     *
     * @note MotionTuner holds non-owning references to Drive, IOdometry, and
     *       UnifiedMotionPidConfig.  All three objects must outlive the tuner.
     */
    class MotionTuner
    {
    public:
        /**
         * @param drive      Chassis drive controller (reference, not owned).
         * @param odometry   Odometry source (reference, not owned).
         * @param pid_config Motion PID config to tune in place (reference, not owned).
         */
        MotionTuner(drive::Drive& drive,
                    odometry::IOdometry& odometry,
                    motion::UnifiedMotionPidConfig& pid_config);

        /**
         * @brief Tune multiple parameters in sequence.
         *
         * Valid param names are "distance", "lateral", and "heading".
         * The pid_config is updated with the best-found gains for each parameter.
         *
         * @param params  List of parameter names to tune.
         * @param cfg     Tuning configuration.
         * @return Map from parameter name to MotionTuneResult.
         */
        [[nodiscard]] std::map<std::string, MotionTuneResult> tune(
            const std::vector<std::string>& params,
            const MotionTuneConfig& cfg) const;

        /**
         * @brief Tune a single motion PID parameter.
         *
         * @param param_name  One of "distance", "lateral", or "heading".
         * @param cfg         Tuning configuration.
         * @return MotionTuneResult with initial/final gains and scores.
         */
        [[nodiscard]] MotionTuneResult tuneParam(
            const std::string& param_name,
            const MotionTuneConfig& cfg) const;

    private:
        /**
         * @brief Run one closed-loop linear trial.
         *
         * @return (settle_time_or_penalty, overshoot_m, final_error_m)
         *         On timeout/stuck: (score_timeout_penalty, 0, remaining_distance)
         */
        [[nodiscard]] std::tuple<double, double, double> runLinearTrial(
            motion::LinearAxis axis,
            double distance_m,
            double speed_scale,
            const MotionTuneConfig& cfg) const;

        /**
         * @brief Run one closed-loop turn trial.
         *
         * @return (settle_time_or_penalty, overshoot_rad, final_error_rad)
         *         On timeout/stuck: (score_timeout_penalty, 0, remaining_angle)
         */
        [[nodiscard]] std::tuple<double, double, double> runTurnTrial(
            double angle_rad,
            double speed_scale,
            const MotionTuneConfig& cfg) const;

        /**
         * @brief Return after a LINEAR trial by retracing the SAME axis.
         *
         * Drives the negative of the trial's axis velocity (forward trial →
         * reverse forward, strafe trial → reverse strafe) with heading-hold
         * correction ON, until the robot is back at the start (straight-line
         * distance from the trial origin is minimal) or a timeout. Uses fixed
         * gentle gains, never the gains being tuned. Pure straight motion keeps
         * ω ≈ 0 so the offset-PAA lever arm never contaminates the position.
         */
        void returnLinear(motion::LinearAxis axis, double home_heading_rad,
                          const MotionTuneConfig& cfg) const;

        /**
         * @brief Return after a TURN trial by rotating back to the home heading.
         *
         * Heading-only (gyro), so the offset-PAA position is never used while
         * rotating. Fixed gentle gains; bounded by a timeout.
         */
        void returnTurn(double home_heading_rad, const MotionTuneConfig& cfg) const;

        /**
         * @brief Set kp/kd on the live config, run the appropriate trial,
         *        settle, then return the composite score.
         */
        [[nodiscard]] double evaluateGains(
            const std::string& param_name,
            double kp,
            double kd,
            int trial_idx,
            const MotionTuneConfig& cfg) const;

        /**
         * @brief Compute composite score from trial metrics.
         *
         * Lower is better. Includes soft-limit constraint penalties for
         * overshoot and final error.
         */
        [[nodiscard]] double scoreMotionTrial(
            const std::string& param_name,
            double settle_time,
            double overshoot,
            double final_error,
            const MotionTuneConfig& cfg) const;

        /**
         * @brief Compute the test distance for a linear trial based on
         *        axis constraints and config bounds.
         */
        [[nodiscard]] double linearTargetDistance(
            motion::LinearAxis axis,
            const MotionTuneConfig& cfg) const;

        /**
         * @brief Compute a reasonable trial timeout from expected duration.
         */
        [[nodiscard]] double trialTimeoutS(
            double distance_m,
            double max_velocity,
            double speed_scale,
            const MotionTuneConfig& cfg) const;

        drive::Drive& drive_;
        odometry::IOdometry& odometry_;
        motion::UnifiedMotionPidConfig& pid_config_;
    };

} // namespace libstp::autotune
