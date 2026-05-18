#pragma once

#include <map>
#include <string>
#include <vector>

#include "autotune/types.hpp"
#include "hal/IIMU.hpp"
#include "hal/IMotor.hpp"

namespace libstp::autotune
{
    /**
     * @brief Tunes the per-motor firmware-side velocity PID (STM32 MAV mode).
     *
     * For each motor the tuner records a baseline step response in BEMF units,
     * fits a FOPDT plant model, derives CHR PID gains, pushes them to the
     * firmware via `IMotor::setFirmwarePidGains()` and re-runs the step
     * response. Gains are accepted only if the tuned ISE is strictly smaller
     * than the baseline ISE.
     *
     * @note FirmwarePidTuner holds non-owning pointers to the supplied motors;
     *       the caller must ensure each motor outlives the tuner.
     */
    class FirmwarePidTuner
    {
    public:
        /**
         * @param motors  Non-owning list of motors to tune.
         * @param imu     Optional IMU pointer — when non-null, every CSV dump
         *                includes the gyro/accel/heading readings sampled in
         *                the same tight loop as the motors.
         */
        explicit FirmwarePidTuner(const std::vector<hal::motor::IMotor*>& motors,
                                  hal::imu::IIMU*                         imu = nullptr);

        /**
         * @brief Tune a single motor.
         *
         * @param motor          Motor to tune.
         * @param max_bemf_speed Steady-state BEMF (firmware velocity units) at
         *                       full power for this motor — typically derived
         *                       from a DriveCharacterizer-style sweep.
         * @param cfg            Tuning configuration.
         */
        [[nodiscard]] FirmwarePidResult tuneMotor(
            hal::motor::IMotor*    motor,
            int                    max_bemf_speed,
            const FirmwarePidConfig& cfg) const;

        /**
         * @brief Tune every motor passed at construction.
         *
         * Motors without an entry in @p max_bemf_speeds are skipped with a
         * warning.
         *
         * @param max_bemf_speeds  Map from port -> max BEMF speed.
         * @param cfg              Shared tuning configuration.
         * @return Map from port -> tuning result.
         */
        [[nodiscard]] std::map<int, FirmwarePidResult> tune(
            const std::map<int, int>& max_bemf_speeds,
            const FirmwarePidConfig&  cfg) const;

    private:
        /**
         * @brief Record one BEMF step response for a single motor.
         *
         * Samples `motor.getBemf()` at @p sample_hz for @p duration_s seconds
         * while commanding @p bemf_target. The motor is braked and settled
         * after the recording.
         *
         * If @p csv_path is non-empty, the function also writes a per-sample
         * dump of every motor's BEMF + position counter and (when an IMU was
         * supplied at construction) the IMU heading/gyro/accel readings to
         * that file. The CSV is intended for offline analysis of the raw
         * step response without re-running on hardware.
         */
        [[nodiscard]] StepResponseData runStepResponse(
            hal::motor::IMotor* motor,
            int                 bemf_target,
            double              duration_s,
            int                 sample_hz,
            const std::string&  csv_path = "") const;

        /**
         * @brief Record BEMF step responses for multiple motors simultaneously.
         *
         * Commands every motor at its matching `bemf_targets[i]` via
         * `setVelocity()` in one batch, then samples every motor's `getBemf()`
         * in a tight loop at @p sample_hz for @p duration_s seconds. All
         * motors are braked together at the end and a `kSettlePauseMs` settle
         * is applied. Returns one `StepResponseData` per input motor (same
         * order as the input vector).
         *
         * Tuning all four drive motors in parallel matches the real operating
         * loading of the wheelbase — shared DC-bus voltage droop, ground
         * coupling and mechanical coupling through the chassis are captured
         * in the plant identification, which sequential per-motor tuning
         * misses.
         */
        [[nodiscard]] std::vector<StepResponseData> runParallelStepResponse(
            const std::vector<hal::motor::IMotor*>& motors,
            const std::vector<int>&                 bemf_targets,
            double                                  duration_s,
            int                                     sample_hz) const;

        std::vector<hal::motor::IMotor*> motors_;
        hal::imu::IIMU*                  imu_{nullptr};
    };

} // namespace libstp::autotune
