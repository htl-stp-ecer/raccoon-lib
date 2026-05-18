#pragma once

#include <functional>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include "autotune/types.hpp"
#include "drive/drive.hpp"
#include "hal/IMotor.hpp"
#include "hal/odometry.hpp"
#include "motion/motion_config.hpp"

namespace libstp::autotune
{
    /**
     * @brief Aggregated configuration for the full auto-tune pipeline.
     *
     * Each `tune_*` flag toggles a phase on or off. The phases run in
     * dependency order: vel_lpf → static_friction → optional firmware_pid →
     * encoder_cal → characterize → velocity → motion → tolerances.
     */
    struct AutoTuneConfig
    {
        // Phase enable flags (in execution order).
        bool tune_vel_lpf{true};
        bool tune_static_friction{true};
        bool tune_firmware_pid{false};
        bool tune_encoder_cal{true};
        bool tune_characterize{true};
        bool tune_velocity{true};
        bool tune_motion{true};
        bool tune_tolerances{true};

        // Per-phase configurations.
        VelLpfConfig         vel_lpf_cfg{};
        StaticFrictionConfig static_friction_cfg{};
        FirmwarePidConfig    firmware_pid_cfg{};
        EncoderCalConfig     encoder_cal_cfg{};
        CharacterizeConfig   characterize_cfg{};
        VelocityTuneConfig   velocity_cfg{};
        MotionTuneConfig     motion_cfg{};
        ToleranceConfig      tolerance_cfg{};

        // Axes / parameters per phase (empty → auto-fill from kinematics support).
        std::vector<std::string> characterize_axes;
        std::vector<std::string> velocity_axes;
        std::vector<std::string> motion_params;

        // Map of axis name → max velocity (m/s or rad/s) for the velocity phase.
        // If empty, populated from the characterize phase results (or sensible
        // defaults when characterize is disabled).
        std::map<std::string, double> max_velocities;

        // Optional pre-supplied max-BEMF speeds for firmware PID tuning.
        // If unset, the AutoTuner runs a short power sweep to estimate them.
        std::optional<std::map<int, int>> max_bemf_speeds{};
    };

    /**
     * @brief Aggregated results of the auto-tune pipeline.
     *
     * Each `<phase>_ran` flag indicates whether the phase actually executed
     * (the corresponding map / struct is populated only when true).
     */
    struct AutoTuneResult
    {
        std::map<int, VelLpfResult>             vel_lpf{};
        std::map<int, StaticFrictionResult>     static_friction{};
        std::map<int, FirmwarePidResult>        firmware_pid{};
        EncoderCalResult                        encoder_cal{};
        std::map<std::string, AxisResult>       characterize{};
        std::map<std::string, VelocityTuneResult> velocity{};
        std::map<std::string, MotionTuneResult> motion{};
        ToleranceResult                         tolerances{};

        bool vel_lpf_ran{false};
        bool static_friction_ran{false};
        bool firmware_pid_ran{false};
        bool encoder_cal_ran{false};
        bool characterize_ran{false};
        bool velocity_ran{false};
        bool motion_ran{false};
        bool tolerances_ran{false};
    };

    /**
     * @brief Orchestrates the full auto-tune pipeline in a single C++ entry
     *        point so phase-to-phase state coherence is guaranteed.
     *
     * Every phase reads its inputs straight from the live `IMotor`,
     * `Drive`, and `UnifiedMotionPidConfig` instances and writes its results
     * back to those same objects before returning. Python wrappers do not
     * need to shuttle calibration values between phases.
     *
     * Execution order (each phase depends on the one before it):
     *   1. vel_lpf          — per-motor IIR alpha (drives velocity feedback)
     *   2. static_friction  — kS for feedforward / minimum-command
     *   3. firmware_pid     — STM32 MAV-mode inner PID
     *   4. encoder_cal      — global ticks_to_rad scale (republishes kinematics)
     *   5. characterize     — physical drive limits (max v, accel, decel)
     *   6. velocity         — chassis-axis velocity PID
     *   7. motion           — distance / heading PID
     *   8. tolerances       — derived from motion residuals
     */
    class AutoTuner
    {
    public:
        /** Per-phase confirmation callback (invoked before the phase runs). */
        using ConfirmCallback = std::function<void(const std::string& phase_key)>;

        AutoTuner(drive::Drive&                  drive,
                  odometry::IOdometry&           odometry,
                  motion::UnifiedMotionPidConfig& motion_pid_config);

        /**
         * @brief Install a callback invoked before every phase in tuneAll().
         *
         * The callback receives a stable phase key — one of "vel_lpf",
         * "static_friction", "firmware_pid", "encoder_cal", "characterize",
         * "velocity", "motion", "tolerances" — so callers can show prompts,
         * record telemetry, or skip phases at runtime.
         *
         * For phases that command motion on multiple axes/params the key is
         * extended with a colon suffix, e.g. "characterize:forward",
         * "velocity:vx", "motion:heading".
         */
        void setConfirmCallback(ConfirmCallback cb);

        /** Run every phase enabled in `cfg`, in dependency order. */
        AutoTuneResult tuneAll(const AutoTuneConfig& cfg);

        // --------------------------------------------------------------
        // Individual phases — usable standalone from Python wrappers.
        // --------------------------------------------------------------

        std::map<int, VelLpfResult>
        runVelLpfPhase(const VelLpfConfig& cfg);

        std::map<int, StaticFrictionResult>
        runStaticFrictionPhase(const StaticFrictionConfig& cfg);

        std::map<int, FirmwarePidResult>
        runFirmwarePidPhase(const FirmwarePidConfig& cfg,
                            std::optional<std::map<int, int>> max_bemf_speeds = std::nullopt);

        EncoderCalResult
        runEncoderCalibrationPhase(const EncoderCalConfig& cfg);

        std::map<std::string, AxisResult>
        runCharacterizePhase(const std::vector<std::string>& axes,
                             const CharacterizeConfig& cfg);

        std::map<std::string, VelocityTuneResult>
        runVelocityPhase(const std::vector<std::string>& axes,
                         const std::map<std::string, double>& max_velocities,
                         const VelocityTuneConfig& cfg);

        std::map<std::string, MotionTuneResult>
        runMotionPhase(const std::vector<std::string>& params,
                       const MotionTuneConfig& cfg);

        ToleranceResult
        runTolerancesPhase(const std::map<std::string, MotionTuneResult>& motion_results,
                           const MotionTuneConfig& motion_cfg,
                           const ToleranceConfig& cfg);

    private:
        /** Invoke the confirm callback (no-op if unset). */
        void confirm(const std::string& phase_key);

        /** Live motor list from `drive_.getMotors()`. */
        std::vector<hal::motor::IMotor*> motors();

        /** Probe each motor briefly to estimate steady-state BEMF at 80% PWM. */
        std::optional<std::map<int, int>> estimateMaxBemfSpeeds();

        /** Quick BEMF sanity check before running BEMF-dependent phases. */
        bool bemfAvailable();

        drive::Drive&                   drive_;
        odometry::IOdometry&            odometry_;
        motion::UnifiedMotionPidConfig& motion_pid_config_;
        ConfirmCallback                 confirm_cb_{};
    };

} // namespace libstp::autotune
