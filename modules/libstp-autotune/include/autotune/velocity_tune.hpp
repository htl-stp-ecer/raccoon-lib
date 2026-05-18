#pragma once

#include <map>
#include <optional>
#include <string>
#include <vector>

#include "autotune/types.hpp"
#include "drive/drive.hpp"
#include "hal/odometry.hpp"

namespace libstp::autotune
{
    /**
     * @brief Tunes per-axis velocity PID gains via step-response identification.
     *
     * For each axis (vx, vy, wz) VelocityTuner:
     *  1. Records a baseline open-loop-equivalent step response with the
     *     existing gains and computes its ISE.
     *  2. Fits a FOPDT plant model using the inflection-tangent method
     *     (with a rise-time fallback).
     *  3. Derives CHR (Chien–Hrones–Reswick) PID gains from the model.
     *  4. Applies the candidate gains, records a second step response, and
     *     accepts the new gains only if the tuned ISE is strictly lower than
     *     the baseline ISE.  Otherwise the original gains are restored.
     *
     * @note VelocityTuner holds non-owning references to Drive and IOdometry.
     *       The caller must ensure both objects outlive the VelocityTuner.
     */
    class VelocityTuner
    {
    public:
        /**
         * @param drive    Chassis drive controller (reference, not owned).
         * @param odometry Odometry source (reference, not owned).
         */
        VelocityTuner(drive::Drive& drive, odometry::IOdometry& odometry);

        /**
         * @brief Tune a single velocity axis.
         *
         * @param axis          One of "vx", "vy", or "wz".
         * @param max_velocity  Maximum physical velocity for this axis (m/s or
         *                      rad/s); typically obtained from DriveCharacterizer.
         * @param cfg           Tuning configuration.
         * @return VelocityTuneResult with plant params, gains, ISE metrics, and
         *         whether the new gains were accepted.
         */
        [[nodiscard]] VelocityTuneResult tuneAxis(
            const std::string&       axis,
            double                   max_velocity,
            const VelocityTuneConfig& cfg) const;

        /**
         * @brief Tune multiple axes in sequence.
         *
         * Axes not present in @p max_velocities are skipped with a warning.
         *
         * @param axes            Axis names to tune (e.g. {"vx", "wz"}).
         * @param max_velocities  Map from axis name to max physical velocity.
         * @param cfg             Shared tuning configuration.
         * @return Map from axis name to VelocityTuneResult.
         */
        [[nodiscard]] std::map<std::string, VelocityTuneResult> tune(
            const std::vector<std::string>&        axes,
            const std::map<std::string, double>&   max_velocities,
            const VelocityTuneConfig&              cfg) const;

    private:
        /**
         * @brief Run one step-response recording on the given axis.
         *
         * Commands @p command via drive_.setVelocity(), samples at @p sample_hz,
         * then calls drive_.hardStop() and settles for 300 ms.
         */
        [[nodiscard]] StepResponseData runStepResponse(
            const std::string& axis,
            double             command,
            double             duration_s,
            int                sample_hz) const;

        /**
         * @brief Identify FOPDT plant parameters via the inflection-tangent method.
         *
         * Returns std::nullopt when the algorithm cannot find a valid inflection
         * point or any derived parameter is non-positive.
         */
        [[nodiscard]] std::optional<PlantParams> identifyPlantInflection(
            const StepResponseData& data,
            double                  command) const;

        /**
         * @brief Identify FOPDT plant parameters via the rise-time method
         *        (10 %→63 % of steady-state value).
         *
         * Used as a fallback when the inflection method fails.
         */
        [[nodiscard]] PlantParams identifyPlantRiseTime(
            const StepResponseData& data,
            double                  command) const;

        /**
         * @brief Compute CHR PID gains from FOPDT plant parameters.
         *
         * Returns PidGains{0,0,0} when @p plant.Ks <= 0 or kp would be <= 0.
         */
        [[nodiscard]] foundation::PidGains computeChrGains(
            const PlantParams&       plant,
            const VelocityTuneConfig& cfg) const;

        /**
         * @brief Trapezoidal ISE over (commanded - measured)^2.
         */
        [[nodiscard]] static double computeIse(const StepResponseData& data);

        /**
         * @brief Mean of |measured| over the last 20 % of samples.
         */
        [[nodiscard]] static double tailMeanAbs(const StepResponseData& data);

        /**
         * @brief Maximum of |measured| across all samples.
         */
        [[nodiscard]] static double peakAbs(const StepResponseData& data);

        drive::Drive&        drive_;
        odometry::IOdometry& odometry_;
    };

} // namespace libstp::autotune
