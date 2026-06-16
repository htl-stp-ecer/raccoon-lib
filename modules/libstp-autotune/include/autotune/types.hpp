#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include "foundation/motor.hpp"  // PidGains, Feedforward

namespace libstp::autotune
{
    /**
     * @brief Measured physical limits for a single chassis axis.
     *
     * All velocity units are m/s (forward, lateral) or rad/s (angular).
     * Acceleration and deceleration are the corresponding per-second-squared values.
     */
    struct AxisResult
    {
        /// Peak velocity at full power (m/s or rad/s).
        double max_velocity{0.0};
        /// Acceleration from 10%–90% of max velocity (m/s² or rad/s²).
        double acceleration{0.0};
        /// Deceleration from 90%–10% during coast-down (m/s² or rad/s²).
        double deceleration{0.0};
    };

    /**
     * @brief Configuration for the DriveCharacterizer.
     */
    struct CharacterizeConfig
    {
        /// Raw PWM percentage applied during characterization (1–100).
        int power_percent{100};
        /// Number of independent trials per axis; median is taken.
        int trials{3};
        /// Maximum seconds to wait for the acceleration phase.
        double accel_timeout{3.0};
        /// Maximum seconds to record the deceleration (coast-down) phase.
        double decel_timeout{3.0};
        /// Sampling rate in Hz for position sampling (e.g. 500).
        int sample_hz{500};
    };

    // ========================================================================
    // Phase 2 — VelocityTuner types
    // ========================================================================

    /**
     * @brief Raw step-response recording for one velocity axis.
     *
     * All three vectors are parallel — element i corresponds to the same
     * sample instant.
     */
    struct StepResponseData
    {
        std::vector<double> times;      ///< Elapsed time (s) since step onset.
        std::vector<double> commanded;  ///< Commanded velocity (m/s or rad/s).
        std::vector<double> measured;   ///< Measured velocity (m/s or rad/s).
    };

    /**
     * @brief First-order-plus-dead-time (FOPDT) plant parameters extracted
     *        from a step-response recording.
     */
    struct PlantParams
    {
        double Ks{0.0};    ///< Static gain (steady-state output / step input).
        double Tu{0.0};    ///< Dead time in seconds.
        double Tg{0.0};    ///< Time constant (lag) in seconds.
        std::string method; ///< Identification method used: "inflection",
                            ///  "rise_time", or "insufficient_response".
    };

    /**
     * @brief Result of tuning a single velocity axis.
     */
    struct VelocityTuneResult
    {
        std::string axis;                              ///< "vx", "vy", or "wz"
        PlantParams plant{};
        foundation::PidGains pid{0.0, 0.0, 0.0};
        foundation::Feedforward ff{0.0, 1.0, 0.0};   ///< kV=1 passthrough default
        double baseline_ise{0.0};
        double tuned_ise{0.0};
        bool accepted{false};
        /// MCU-chassis path: the calibrated per-axis velocity-command gain that
        /// was applied to the kinematics fwd_matrix (and pushed to the STM32) so
        /// commanded == achieved body velocity. 1.0 means no correction.
        double velocity_command_gain{1.0};
        /// Measured steady-state gain (achieved/commanded) before tuning.
        double measured_gain_before{0.0};
        /// Measured steady-state gain (achieved/commanded) after tuning.
        double measured_gain_after{0.0};
        /// Raw baseline step-response recording (commanded vs measured), for
        /// offline plotting of the open-loop fit.
        StepResponseData baseline_response{};
        /// Raw tuned step-response recording (empty when CHR was skipped).
        StepResponseData tuned_response{};
    };

    /**
     * @brief Configuration for the VelocityTuner.
     */
    struct VelocityTuneConfig
    {
        /// Duration of each step-response recording in seconds.
        double step_duration_s{2.0};
        /// Fraction of max_velocity used as the step command (0–1).
        double step_command_frac{0.50};
        /// Step-response sampling rate in Hz.
        int sample_hz{200};

        // CHR gain scaling factors.
        double chr_kp_scale{0.3};
        double chr_ki_scale{0.6};
        double chr_kd_scale{0.15};

        // Rejection thresholds.
        /// Tail response must exceed this fraction of |command| to accept.
        double min_response_frac{0.10};
        /// Tu/Tg below this → skip CHR (near-integrator / very fast plant).
        double low_dead_time_ratio{0.05};
        /// |Ks - 1| tolerance accompanying the low-dead-time early-return.
        double low_dead_time_gain_tol{0.15};

        // ---- MCU-chassis velocity-command-gain calibration ----
        /// Clamp range for the calibrated per-axis command gain (1/Ks).
        double gain_min{0.3};
        double gain_max{3.0};
        /// Settle time (s) after re-publishing the kinematics config to the STM32.
        double republish_settle_s{0.4};
        /// Fraction of the run (from the end) used to fit steady-state velocity.
        double steady_state_frac{0.5};
    };

    // ========================================================================
    // Phase 3 — MotionTuner types
    // ========================================================================

    /**
     * @brief Result of tuning a single motion PID parameter (distance, lateral, or heading).
     */
    struct MotionTuneResult
    {
        std::string param_name;
        double initial_kp{0.0};
        double initial_kd{0.0};
        double final_kp{0.0};
        double final_kd{0.0};
        double initial_score{0.0};
        double final_score{0.0};
        int iterations{0};
    };

    /**
     * @brief Configuration for the MotionTuner (Hooke-Jeeves coordinate descent).
     */
    struct MotionTuneConfig
    {
        int sample_hz{100};                    ///< Update loop rate (Hz).
        double settle_s{0.5};                  ///< Post-trial settle time (s).
        int max_iterations{6};                 ///< Maximum Hooke-Jeeves iterations.
        double initial_delta_frac{0.25};       ///< Fraction of initial gain for first delta.
        double min_delta{0.01};                ///< Minimum delta before stopping.
        double delta_shrink{0.5};              ///< Factor to shrink deltas on no improvement.

        // Trial parameters.
        // Every linear trial drives a FIXED distance (not speed-derived) so the
        // robot reliably uses the ~1 m runway the operator cleared. Forward to
        // linear_test_distance_m (recorded for scoring), then a closed return to
        // the start; repeat. Decoupled from max_velocity on purpose — a 0.2 m/s
        // chassis would otherwise only ever test ~0.2 m and never characterise
        // settle/overshoot at a realistic distance.
        double linear_test_distance_m{0.90};   ///< Linear test distance per trial (m).
        double turn_test_angle_deg{90.0};      ///< Turn test angle (degrees).
        double motion_timeout_s{10.0};         ///< Maximum trial duration (s).
        double min_timeout_s{4.0};             ///< Minimum computed timeout (s).
        double stuck_timeout_s{1.5};           ///< Time without progress → stuck (s).
        double stuck_linear_progress_m{0.03};  ///< Linear progress threshold (m).
        double stuck_angular_progress_deg{8.0};///< Angular progress threshold (deg).
        double primary_speed_scale{1.0};       ///< Speed scale for trial motions.
        double min_linear_distance_m{0.40};    ///< Minimum clamped test distance (m).
        double max_linear_distance_m{1.00};    ///< Maximum clamped test distance (m).

        // Scoring weights.
        double score_settle_weight{1.0};
        double score_overshoot_weight{10.0};
        double score_error_weight{5.0};
        double score_timeout_penalty{50.0};
        double score_constraint_breach_base{25.0};

        // Soft limits for constraint-aware scoring.
        double linear_overshoot_soft_m{0.010};
        double linear_final_error_soft_m{0.010};
        double turn_overshoot_soft_rad{0.05236};   ///< 3 degrees in radians.
        double turn_final_error_soft_rad{0.03491}; ///< 2 degrees in radians.
        double score_linear_overshoot_breach_per_m{2000.0};
        double score_linear_error_breach_per_m{1000.0};
        double score_turn_overshoot_breach_per_rad{400.0};
        double score_turn_error_breach_per_rad{250.0};
    };

    // ========================================================================
    // Phase 4 — FirmwarePidTuner types
    // ========================================================================

    /**
     * @brief Configuration for the FirmwarePidTuner.
     *
     * Per-motor analogue of VelocityTuneConfig — the tuner runs a BEMF step
     * response, identifies a FOPDT plant, derives CHR gains, and pushes them
     * to the STM32 firmware MAV-mode PID.
     */
    struct FirmwarePidConfig
    {
        /// Raw PWM percent used to identify the motor plant. All drive motors
        /// run at this power simultaneously so BEMF captures real DC-bus and
        /// chassis loading.
        int raw_pwm_percent{70};
        double step_fraction{0.5};      ///< Fraction of measured max BEMF used for MAV validation.
        /// Duration of the raw PWM step-response recording (s). This is the
        /// "Sprungantwort": raw power on all motors, no IMU straightening, no
        /// velocity PID.
        double step_duration_s{5.0};
        double decel_duration_s{1.0};   ///< Extra samples after raw PWM is set to zero.
        int    sample_hz{200};          ///< Step-response sampling rate (Hz).
        /// CHR PID coefficients from the ControlTheory step-response tuner:
        /// Kp = 0.6*Tg/(Ks*Tu), Ki = 0.6/(Ks*Tu), Kd = 0.3*Tg/Ks.
        double chr_kp_scale{0.6};
        double chr_ki_scale{0.6};
        double chr_kd_scale{0.3};
        double min_response_frac{0.10}; ///< Tail/|command| threshold — skip if smaller.
        /// Number of full parallel baseline responses used to identify the
        /// plant and establish the baseline ISE. Multiple passes reduce BEMF
        /// jitter and random floor/contact effects.
        int identification_trials{3};
        /// Number of full parallel validation responses after candidate gains
        /// are applied. Gains must improve the mean ISE by accept_improvement_frac.
        int validation_trials{3};
        double accept_improvement_frac{0.02};
        /// When non-empty, dump every sample of every step response to CSV
        /// under this directory.  One CSV per (motor_port, phase) plus one
        /// summary CSV with plant fits and accept/reject decisions.  Lets
        /// you replay the raw data offline to iterate on the identification
        /// algorithm without re-running on hardware.
        std::string csv_dir{};

        // --------------------------------------------------------------------
        // ControlTheory differential-evolution gain computation.
        //
        // When use_control_theory is true the per-motor candidate gains are
        // computed by (1) fitting a PT1 plant to the raw-PWM step response via
        // DE system-identification, then (2) DE-optimizing physical firmware
        // kp/ki/kd against the simulated closed loop using the dt-EXPLICIT
        // ControlElementPID (which mirrors STM32 pid.c). The CHR heuristic is
        // kept only as a fallback when the DE fit fails.
        //
        // Scales:
        //  * plant K is BEMF-units-per-%PWM (tens-to-hundreds for the wombat
        //    motors at ~70% PWM), hence the large de_plant_max default.
        //  * gains are physical firmware kp/ki/kd (per-second for ki/kd).
        //  * de_max_overshoot is in output BEMF units.
        //  * de_sim_horizon_s sets the internal sim dt = horizon / 5000
        //    (default 5 s -> 1 ms). With the dt-explicit PID the optimized
        //    gains are physical, so the sim dt need not equal the firmware
        //    rate; the horizon only needs to cover settling.
        // --------------------------------------------------------------------
        bool          use_control_theory{true};
        int           de_plant_steps{80};
        double        de_plant_min{0.0001};
        double        de_plant_max{5000.0};
        int           de_pid_steps{60};
        double        de_gain_max{10.0};
        double        de_sim_horizon_s{5.0};
        double        de_max_overshoot{50.0};
        double        de_weight_ctrl{0.0};
        double        de_weight_overshoot{1.0};
        // Actuator saturation modeled in the closed-loop sim: the simulated PID
        // output and integral term are clamped to +/-de_output_max, matching the
        // firmware MOTOR_MAX_DUTYCYCLE (399). Without this the unbounded sim
        // output produces fictional overshoot and DE picks uselessly small gains.
        double        de_output_max{399.0};
        std::uint32_t de_seed{0xC0FFEEu};
    };

    /**
     * @brief Result of tuning a single motor's firmware-side velocity PID.
     */
    struct FirmwarePidResult
    {
        int         motor_port{-1};
        PlantParams plant{};
        float       kp{0.0f};
        float       ki{0.0f};
        float       kd{0.0f};
        double      baseline_ise{0.0};
        double      tuned_ise{0.0};
        bool        accepted{false};
        /// Gain-computation method: "control_theory_de", "chr_fallback", or
        /// "chr" (legacy default before any computation runs).
        std::string pid_method{"chr"};
        /// PT1 plant gain from the DE fit (BEMF units per %PWM). 0 if unused.
        double      plant_K{0.0};
        /// PT1 plant time constant from the DE fit (s). 0 if unused.
        double      plant_T{0.0};
    };

    // ========================================================================
    // Phase 5 — StaticFrictionMeasurer types
    // ========================================================================

    /**
     * @brief Configuration for the StaticFrictionMeasurer.
     */
    struct StaticFrictionConfig
    {
        int start_pct{2};          ///< Lower bound of the PWM sweep (inclusive).
        int max_pct{50};           ///< Upper bound of the PWM sweep (inclusive).
        int step_pct{1};           ///< PWM increment per sweep step.
        int dwell_ms{50};          ///< Dwell after each setSpeed() call (ms).
        int samples_per_step{5};   ///< BEMF samples taken per step (median).
        int motion_threshold{3};   ///< Median |BEMF| above this counts as "moving".
    };

    /**
     * @brief One PWM-vs-BEMF sample taken during a static-friction sweep.
     */
    struct StaticFrictionSample
    {
        int pwm_pct{0};      ///< Signed PWM percent commanded (+ forward, − reverse).
        int median_bemf{0};  ///< Median |BEMF| (ADC counts) at this step.
    };

    /**
     * @brief Per-motor static-friction measurement result (kS in PWM percent).
     */
    struct StaticFrictionResult
    {
        int  motor_port{-1};
        int  ks_positive_pct{0};
        int  ks_negative_pct{0};
        int  ks_avg_pct{0};
        bool measured{false};
        /// Forward sweep PWM-vs-BEMF curve (pwm_pct > 0), in sweep order.
        std::vector<StaticFrictionSample> forward_sweep{};
        /// Reverse sweep PWM-vs-BEMF curve (pwm_pct < 0), in sweep order.
        std::vector<StaticFrictionSample> reverse_sweep{};
        /// Motion threshold used (median |BEMF| above this counts as "moving").
        int motion_threshold{0};
    };

    // ========================================================================
    // Phase 6 — VelLpfTuner types
    // ========================================================================

    /**
     * @brief Configuration for the VelLpfTuner (encoder-velocity LPF alpha).
     */
    struct VelLpfConfig
    {
        double alpha_min{0.05};        ///< Minimum alpha to evaluate.
        double alpha_max{0.95};        ///< Maximum alpha to evaluate.
        double alpha_step{0.05};       ///< Sweep increment.
        double measure_duration_s{1.0};///< Sampling duration per motor (s).
        int    sample_hz{200};         ///< Sampling rate (Hz).
        double noise_weight{1.0};      ///< Weight on filtered variance in the score.
        double lag_weight{0.5};        ///< Weight on (1 / lag-change-rate) in the score.
        /// Open-loop PWM (%) applied to spin the motors while sampling. The LPF
        /// filters the *running* BEMF stream, so the signal must be captured in
        /// motion — sampling at standstill yields only deadzone noise and tunes
        /// nothing. All drive motors spin forward together (one straight run).
        int    spin_percent{40};
        double settle_s{0.5};          ///< Settle time after spin-up before sampling (s).
    };

    /**
     * @brief One point of the alpha sweep: the score (and its parts) the
     *        candidate alpha achieved on the captured raw BEMF series.
     */
    struct VelLpfSweepPoint
    {
        double alpha{0.0};
        double variance{0.0};        ///< Variance of the filtered series.
        double lag_change_rate{0.0}; ///< Std-dev of |Δfilt| (movement proxy).
        double score{0.0};           ///< Combined noise+lag score (lower is better).
    };

    /**
     * @brief Per-motor LPF-alpha tuning result.
     */
    struct VelLpfResult
    {
        int    motor_port{-1};
        double initial_alpha{0.5};
        double tuned_alpha{0.5};
        double min_score{0.0};
        bool   applied{false};
        /// Raw BEMF samples captured while the motor spun at a steady open-loop
        /// PWM (ADC counts), in acquisition order. These are replayed offline to
        /// render the raw-vs-filtered overlay at the chosen alpha.
        std::vector<int> raw_bemf{};
        /// Score-vs-alpha sweep curve (one entry per evaluated alpha).
        std::vector<VelLpfSweepPoint> sweep{};
    };

    // ========================================================================
    // Phase 7 — ToleranceDeriver types
    // ========================================================================

    /**
     * @brief Configuration for deriveTolerances().
     */
    struct ToleranceConfig
    {
        double margin_factor{1.5};
        double min_distance_tolerance_m{0.005};
        double max_distance_tolerance_m{0.050};
        double min_angle_tolerance_rad{0.017};
        double max_angle_tolerance_rad{0.087};
    };

    /**
     * @brief Result of deriveTolerances(): the derived tolerances and flags
     *        indicating which UnifiedMotionPidConfig fields were updated.
     */
    struct ToleranceResult
    {
        double derived_distance_tolerance_m{0.01};
        double derived_angle_tolerance_rad{0.035};
        bool   distance_updated{false};
        bool   angle_updated{false};
    };

    // ========================================================================
    // Phase 8 — EncoderCalibrator types
    // ========================================================================

    /**
     * @brief Configuration for the EncoderCalibrator.
     */
    struct EncoderCalConfig
    {
        double angular_velocity_rad_s{1.0};
        double total_angle_rad{4.0 * 3.14159265358979}; ///< 2 full turns.
        double settle_s{0.5};
        int    sample_hz{100};
        double min_required_angle_rad{3.14159};         ///< At least 180° to compute scale.
    };

    /**
     * @brief Encoder calibration result.
     *
     * The scale factor is identical for all motors — it reflects the systematic
     * error in the assumed ticks_to_rad. Per-motor variation needs richer
     * kinematics; this calibrator only fixes the global ratio.
     */
    struct EncoderCalResult
    {
        std::array<double, 4> ticks_to_rad{};    ///< New per-motor ticks_to_rad.
        std::array<double, 4> scale_factors{};   ///< Per-motor scale factor (same for all).
        double                imu_total_angle_rad{0.0};
        double                odom_total_angle_rad{0.0};
        bool                  success{false};
        std::string           failure_reason{};
    };

} // namespace libstp::autotune
