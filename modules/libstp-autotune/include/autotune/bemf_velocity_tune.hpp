#pragma once

#include <array>
#include <string>
#include <vector>

#include "autotune/types.hpp"
#include "drive/drive.hpp"
#include "hal/odometry.hpp"

namespace libstp::autotune
{
    /**
     * @brief Configuration for BemfVelocityTuner.
     *
     * The tuner drives the chassis straight forward at a sweep of open-loop PWM
     * levels and, for each level, measures the ground-truth distance travelled
     * (from the external calibration board) against the accumulated BEMF ticks
     * per motor. From that it derives per-motor `ticks_to_rad` AND a linearity
     * report (does a single scale hold across the speed range, or is the
     * ADC-BEMF↔velocity relationship curved / offset?).
     *
     * Back-and-forth motion: after each forward measurement segment the robot
     * reverses back toward its starting pose, so the whole sweep stays within
     * roughly one segment length of the origin.
     */
    struct BemfVelocityConfig
    {
        /// Explicit PWM levels (percent, 1..100) to sweep. If empty, a linear
        /// ramp of `pwm_steps` points from `pwm_min_percent` to
        /// `pwm_max_percent` is generated.
        std::vector<int> pwm_levels{};
        int    pwm_min_percent{30};
        int    pwm_max_percent{90};
        int    pwm_steps{6};
        /// Number of full sweeps. Points from all sweeps are pooled into one
        /// per-motor fit, which stabilises the extrapolated bemf_offset (the
        /// ω=0 intercept is sensitive to noise from a single sweep).
        int    sweeps{1};

        /// Distance to roll (and discard) before each measurement window, so the
        /// chassis reaches steady-state speed at that PWM. Meters.
        double pre_roll_distance_m{0.10};
        /// Ground-truth distance of the measurement window itself. Meters.
        double measure_distance_m{0.15};
        /// Safety cap on a single forward segment (pre-roll + window). Seconds.
        double segment_timeout_s{5.0};
        /// PWM used to drive back toward the origin between segments (percent).
        int    return_pwm_percent{35};
        /// How close to the origin the return move tries to get. Meters.
        double return_tolerance_m{0.04};
        double return_timeout_s{6.0};

        int    sample_hz{100};
        /// Settle/brake dwell between segments. Seconds.
        double settle_s{0.5};

        /// Reject a measurement window whose ground-truth distance is below this
        /// (motor stalled / static friction). Meters.
        double min_window_distance_m{0.03};
        /// Reject a per-motor result whose accumulated |ticks| is below this.
        double min_window_ticks{20.0};

        /// Fail outright unless the calibration board currently backs the
        /// odometry pose (we need an accurate external ground truth).
        bool   require_calib_board{true};
        /// Apply the derived ticks_to_rad to the live motors and republish the
        /// kinematics config (via odometry.reset()) at the end. The Python step
        /// additionally persists to YAML when the user opts in.
        bool   apply{true};
    };

    /// One measurement window at a single PWM level.
    struct BemfVelocityPoint
    {
        int    pwm_percent{0};
        double ground_truth_distance_m{0.0}; ///< calibration-board displacement
        double window_time_s{0.0};
        double body_speed_mps{0.0};          ///< distance / time (≈ steady state)
        double wheel_omega_rad_s{0.0};       ///< body_speed / wheel_radius
        std::array<double, 4> delta_ticks{}; ///< accumulated BEMF ticks per motor
        std::array<double, 4> median_bemf{}; ///< median |instantaneous BEMF| per motor
        std::array<double, 4> ticks_to_rad{};///< (D/r)/|Δticks| per motor
        bool   valid{false};
    };

    /// Per-motor fit + linearity diagnostics across the speed sweep.
    struct BemfMotorFit
    {
        int    port{-1};
        int    n_points{0};
        double ticks_to_rad_median{0.0}; ///< value chosen for persistence
        double ticks_to_rad_mean{0.0};
        double ticks_to_rad_cv{0.0};     ///< coeff. of variation across speeds
                                         ///< (small => tick↔angle is linear)
        // ω = slope·bemf + intercept  (ω from ground truth, bemf from ADC).
        double bemf_omega_slope{0.0};
        double bemf_omega_intercept{0.0};
        double bemf_omega_r2{0.0};
        /// BEMF reading (ADC counts) where the driving line hits ω=0
        /// (= -intercept/slope). This is the per-motor offset to subtract on the
        /// STM32 so the tick integral becomes speed-independent.
        double bemf_offset{0.0};
        bool   linear{false};            ///< cv low AND r² high AND |intercept| small
    };

    /// Full result of a BEMF→velocity calibration run.
    struct BemfVelocityResult
    {
        std::vector<BemfVelocityPoint> points{};
        std::array<BemfMotorFit, 4>    motors{};
        std::array<double, 4>          ticks_to_rad{}; ///< chosen per-motor value
        std::array<double, 4>          bemf_offset{};  ///< per-motor BEMF offset (counts)
        bool   linear_overall{false};
        bool   applied{false};
        bool   success{false};
        std::string failure_reason{};
    };

    /**
     * @brief Calibrate per-motor `ticks_to_rad` (BEMF→rad) against the external
     *        calibration board, fully automatically.
     *
     * @note Requires the calibration-board odometry to back `IOdometry::getPose()`
     *       (i.e. `getActiveSource() == OdometrySource::CalibrationBoard`).
     */
    class BemfVelocityTuner
    {
    public:
        BemfVelocityTuner(drive::Drive& drive, odometry::IOdometry& odometry);

        [[nodiscard]] BemfVelocityResult tune(const BemfVelocityConfig& cfg) const;

    private:
        drive::Drive&        drive_;
        odometry::IOdometry& odometry_;
    };

} // namespace libstp::autotune
