#include "autotune/bemf_velocity_tune.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <thread>

#include "foundation/logging.hpp"
#include "foundation/types.hpp"

namespace libstp::autotune
{
namespace
{
    using Clock  = std::chrono::steady_clock;
    using Micros = std::chrono::microseconds;
    using libstp::foundation::ChassisVelocity;
    using libstp::foundation::Pose;

    const ChassisVelocity kForward{1.0, 0.0, 0.0};
    const ChassisVelocity kBackward{-1.0, 0.0, 0.0};

    double planarDist(const Pose& a, const Pose& b)
    {
        const double dx = static_cast<double>(a.position.x() - b.position.x());
        const double dy = static_cast<double>(a.position.y() - b.position.y());
        return std::hypot(dx, dy);
    }

    double median(std::vector<double> v)
    {
        if (v.empty())
            return 0.0;
        std::sort(v.begin(), v.end());
        const std::size_t n = v.size();
        return (n % 2 == 1) ? v[n / 2] : 0.5 * (v[n / 2 - 1] + v[n / 2]);
    }

    double mean(const std::vector<double>& v)
    {
        if (v.empty())
            return 0.0;
        double s = 0.0;
        for (double x : v) s += x;
        return s / static_cast<double>(v.size());
    }

    /// Coefficient of variation (std/|mean|). 0 => perfectly constant.
    double coeffVar(const std::vector<double>& v)
    {
        if (v.size() < 2)
            return 0.0;
        const double m = mean(v);
        if (std::abs(m) < 1e-12)
            return 0.0;
        double acc = 0.0;
        for (double x : v) acc += (x - m) * (x - m);
        return std::sqrt(acc / static_cast<double>(v.size() - 1)) / std::abs(m);
    }

    struct LinFit { double slope{0.0}; double intercept{0.0}; double r2{0.0}; };

    /// Ordinary least squares y = slope*x + intercept, plus R².
    LinFit linFit(const std::vector<double>& x, const std::vector<double>& y)
    {
        LinFit f;
        const std::size_t n = std::min(x.size(), y.size());
        if (n < 2)
            return f;
        double mx = 0.0, my = 0.0;
        for (std::size_t i = 0; i < n; ++i) { mx += x[i]; my += y[i]; }
        mx /= static_cast<double>(n);
        my /= static_cast<double>(n);
        double sxx = 0.0, sxy = 0.0, syy = 0.0;
        for (std::size_t i = 0; i < n; ++i)
        {
            const double dx = x[i] - mx;
            const double dy = y[i] - my;
            sxx += dx * dx;
            sxy += dx * dy;
            syy += dy * dy;
        }
        if (sxx < 1e-12)
            return f;
        f.slope     = sxy / sxx;
        f.intercept = my - f.slope * mx;
        f.r2        = (syy > 1e-12) ? (sxy * sxy) / (sxx * syy) : 1.0;
        return f;
    }

    std::vector<int> buildPwmLevels(const BemfVelocityConfig& cfg)
    {
        if (!cfg.pwm_levels.empty())
            return cfg.pwm_levels;
        std::vector<int> levels;
        const int steps = std::max(2, cfg.pwm_steps);
        for (int i = 0; i < steps; ++i)
        {
            const double frac = static_cast<double>(i) / static_cast<double>(steps - 1);
            levels.push_back(cfg.pwm_min_percent +
                static_cast<int>(std::lround(frac * (cfg.pwm_max_percent - cfg.pwm_min_percent))));
        }
        return levels;
    }
} // namespace

BemfVelocityTuner::BemfVelocityTuner(drive::Drive& drive, odometry::IOdometry& odometry)
    : drive_(drive), odometry_(odometry)
{
}

BemfVelocityResult BemfVelocityTuner::tune(const BemfVelocityConfig& cfg) const
{
    BemfVelocityResult result{};
    const auto initial_source = odometry_.getActiveSource();

    auto motors = drive_.getMotors();
    if (motors.empty())
    {
        result.failure_reason = "no drive motors";
        LIBSTP_LOG_WARN("[BemfVelocityTune] {}", result.failure_reason);
        return result;
    }
    const std::size_t n_motors = std::min<std::size_t>(motors.size(), 4);
    const double r = drive_.getWheelRadius();
    if (!(r > 1e-6))
    {
        result.failure_reason = "invalid wheel radius";
        LIBSTP_LOG_WARN("[BemfVelocityTune] {}", result.failure_reason);
        return result;
    }

    if (cfg.require_calib_board &&
        initial_source != odometry::OdometrySource::CalibrationBoard)
    {
        result.failure_reason =
            "calibration board is not the active odometry source — connect it before tuning";
        LIBSTP_LOG_WARN("[BemfVelocityTune] {}", result.failure_reason);
        return result;
    }

    const auto pwm_levels = buildPwmLevels(cfg);
    const Micros period{1'000'000 / std::max(1, cfg.sample_hz)};
    const Pose origin = odometry_.getPose();

    LIBSTP_LOG_INFO("============================================================");
    LIBSTP_LOG_INFO("  BEMF→VELOCITY CALIBRATION (calibration-board ground truth)");
    LIBSTP_LOG_INFO("  wheel_radius={:.4f} m, {} PWM levels x {} sweep(s), source=CALIBRATION_BOARD",
                    r, pwm_levels.size(), std::max(1, cfg.sweeps));
    LIBSTP_LOG_INFO("============================================================");

    auto sourceStillValid = [&]() -> bool {
        const auto current_source = odometry_.getActiveSource();
        if (current_source == initial_source)
            return true;
        result.failure_reason =
            "active odometry source changed during BEMF tuning — stopping because a mid-run "
            "source swap invalidates the calibration";
        LIBSTP_LOG_ERROR("[BemfVelocityTune] {}", result.failure_reason);
        return false;
    };

    // --- Drive a direction until `predicate(pose)` is true or timeout. Optionally
    //     accumulate |BEMF| samples per motor while driving. ----------------------
    auto driveUntil = [&](const ChassisVelocity& dir, int pwm,
                          const std::function<bool(const Pose&)>& done,
                          double timeout_s,
                          std::vector<double>* bemf_acc /*size n_motors, may be null*/)
        -> bool
    {
        drive_.applyPowerCommand(dir, pwm);
        const auto t0 = Clock::now();
        auto next = t0;
        while (true)
        {
            if (!sourceStillValid())
                return false;
            const Pose p = odometry_.getPose();
            if (done(p))
                break;
            if (std::chrono::duration<double>(Clock::now() - t0).count() >= timeout_s)
                break;
            if (bemf_acc)
                for (std::size_t i = 0; i < n_motors; ++i)
                    bemf_acc[i].push_back(std::abs(static_cast<double>(motors[i]->getBemf())));
            next += period;
            const auto now = Clock::now();
            if (next > now) std::this_thread::sleep_until(next);
            else            next = now;
        }
        return true;
    };

    auto brakeAll = [&]() { for (auto* m : motors) m->brake(); };

    const int n_sweeps = std::max(1, cfg.sweeps);
    for (int sweep = 0; sweep < n_sweeps; ++sweep)
    for (int pwm : pwm_levels)
    {
        LIBSTP_LOG_INFO("--- sweep {}/{} PWM {}% ---", sweep + 1, n_sweeps, pwm);
        BemfVelocityPoint pt{};
        pt.pwm_percent = pwm;

        // Pre-roll to steady state (discarded).
        const Pose seg_start = odometry_.getPose();
        if (!driveUntil(kForward, pwm,
                        [&](const Pose& p) {
                            return planarDist(p, seg_start) >= cfg.pre_roll_distance_m;
                        },
                        cfg.segment_timeout_s, nullptr))
        {
            brakeAll();
            return result;
        }

        // Measurement window at (hopefully) steady speed.
        const Pose win_start = odometry_.getPose();
        std::array<int, 4> start_ticks{};
        for (std::size_t i = 0; i < n_motors; ++i)
            start_ticks[i] = motors[i]->getPosition();
        const auto win_t0 = Clock::now();

        std::vector<double> bemf_acc[4];
        if (!driveUntil(kForward, pwm,
                        [&](const Pose& p) {
                            return planarDist(p, win_start) >= cfg.measure_distance_m;
                        },
                        cfg.segment_timeout_s, bemf_acc))
        {
            brakeAll();
            return result;
        }

        const Pose win_end = odometry_.getPose();
        const double win_dt = std::chrono::duration<double>(Clock::now() - win_t0).count();
        brakeAll();

        pt.ground_truth_distance_m = planarDist(win_end, win_start);
        pt.window_time_s = win_dt;
        pt.body_speed_mps = (win_dt > 1e-6) ? pt.ground_truth_distance_m / win_dt : 0.0;
        pt.wheel_omega_rad_s = pt.body_speed_mps / r;

        bool point_ok = pt.ground_truth_distance_m >= cfg.min_window_distance_m;
        const double wheel_angle_rad = pt.ground_truth_distance_m / r;
        for (std::size_t i = 0; i < n_motors; ++i)
        {
            const double dticks = std::abs(
                static_cast<double>(motors[i]->getPosition() - start_ticks[i]));
            pt.delta_ticks[i]  = dticks;
            pt.median_bemf[i]  = median(bemf_acc[i]);
            if (dticks >= cfg.min_window_ticks)
                pt.ticks_to_rad[i] = wheel_angle_rad / dticks;
            else
                point_ok = false;
        }
        pt.valid = point_ok;

        LIBSTP_LOG_INFO(
            "    D={:.4f} m, t={:.3f} s, v={:.4f} m/s, ω={:.3f} rad/s, "
            "Δticks=[{:.0f},{:.0f},{:.0f},{:.0f}] valid={}",
            pt.ground_truth_distance_m, pt.window_time_s, pt.body_speed_mps,
            pt.wheel_omega_rad_s, pt.delta_ticks[0], pt.delta_ticks[1],
            pt.delta_ticks[2], pt.delta_ticks[3], pt.valid);

        result.points.push_back(pt);

        // Settle, then drive back toward the origin to conserve runway.
        std::this_thread::sleep_for(std::chrono::duration<double>(cfg.settle_s));
        double prev_dist = planarDist(odometry_.getPose(), origin);
        if (!driveUntil(kBackward, cfg.return_pwm_percent,
                        [&](const Pose& p) {
                            const double d = planarDist(p, origin);
                            const bool reached = d <= cfg.return_tolerance_m;
                            const bool overshoot = d > prev_dist + 0.01;
                            prev_dist = d;
                            return reached || overshoot;
                        },
                        cfg.return_timeout_s, nullptr))
        {
            brakeAll();
            return result;
        }
        brakeAll();
        std::this_thread::sleep_for(std::chrono::duration<double>(cfg.settle_s));
    }

    // ---- Per-motor fits + linearity diagnostics ----------------------------
    int valid_points = 0;
    for (const auto& p : result.points) if (p.valid) ++valid_points;
    if (valid_points < 2)
    {
        result.failure_reason =
            "fewer than 2 valid speed points — check runway, calibration board, friction";
        LIBSTP_LOG_WARN("[BemfVelocityTune] {}", result.failure_reason);
        return result;
    }

    bool overall_linear = true;
    for (std::size_t i = 0; i < n_motors; ++i)
    {
        std::vector<double> ttr, bemf, omega;
        for (const auto& p : result.points)
        {
            if (!p.valid) continue;
            ttr.push_back(p.ticks_to_rad[i]);
            bemf.push_back(p.median_bemf[i]);
            omega.push_back(p.wheel_omega_rad_s);
        }
        BemfMotorFit fit{};
        fit.port = motors[i]->getPort();
        fit.n_points = static_cast<int>(ttr.size());
        fit.ticks_to_rad_median = median(ttr);
        fit.ticks_to_rad_mean   = mean(ttr);
        fit.ticks_to_rad_cv     = coeffVar(ttr);
        const LinFit lf = linFit(bemf, omega);
        fit.bemf_omega_slope     = lf.slope;
        fit.bemf_omega_intercept = lf.intercept;
        fit.bemf_omega_r2        = lf.r2;
        // BEMF reading where the driving line hits ω=0 (in getBemf()'s
        // inversion-corrected space). Keep the sign — inverted motors have a
        // negative corrected-space offset.
        const double b_corrected =
            (std::abs(lf.slope) > 1e-9) ? -lf.intercept / lf.slope : 0.0;
        // The STM32 integrates the RAW BEMF reading (getBemf() applies the
        // inversion on top), so convert the offset back to raw space: negate it
        // for inverted motors. The firmware then subtracts it plainly.
        fit.bemf_offset = motors[i]->isInverted() ? -b_corrected : b_corrected;

        const double max_omega = *std::max_element(omega.begin(), omega.end());
        const bool small_offset = std::abs(fit.bemf_omega_intercept) <= 0.10 * std::max(1e-6, max_omega);
        fit.linear = (fit.ticks_to_rad_cv < 0.12) && (fit.bemf_omega_r2 > 0.95) && small_offset;
        overall_linear = overall_linear && fit.linear;

        result.motors[i] = fit;
        result.ticks_to_rad[i] = fit.ticks_to_rad_median;
        result.bemf_offset[i] = fit.bemf_offset;

        LIBSTP_LOG_INFO(
            "  motor port={}: ticks_to_rad median={:.6g} mean={:.6g} CV={:.1f}% | "
            "ω=({:.4g})·bemf+({:.4g}) r²={:.4f} bemf_offset={:.1f} -> {}",
            fit.port, fit.ticks_to_rad_median, fit.ticks_to_rad_mean,
            100.0 * fit.ticks_to_rad_cv, fit.bemf_omega_slope, fit.bemf_omega_intercept,
            fit.bemf_omega_r2, fit.bemf_offset, fit.linear ? "LINEAR" : "NON-LINEAR");
    }
    result.linear_overall = overall_linear;
    result.success = true;

    if (!overall_linear)
        LIBSTP_LOG_WARN(
            "[BemfVelocityTune] BEMF↔velocity is NOT well described by a single "
            "ticks_to_rad across the speed range — a single scale will be imprecise.");

    // Optional CSV dump for offline analysis.
    if (const char* dir = std::getenv("LIBSTP_AUTOTUNE_CSV"))
    {
        std::ofstream f(std::string(dir) + "/bemf_velocity.csv");
        if (f)
        {
            f << "pwm,distance_m,time_s,speed_mps,omega_rad_s,valid";
            for (std::size_t i = 0; i < n_motors; ++i) f << ",dticks" << i << ",bemf" << i << ",ttr" << i;
            f << "\n";
            for (const auto& p : result.points)
            {
                f << p.pwm_percent << "," << p.ground_truth_distance_m << "," << p.window_time_s
                  << "," << p.body_speed_mps << "," << p.wheel_omega_rad_s << "," << p.valid;
                for (std::size_t i = 0; i < n_motors; ++i)
                    f << "," << p.delta_ticks[i] << "," << p.median_bemf[i] << "," << p.ticks_to_rad[i];
                f << "\n";
            }
        }
    }

    // ---- Apply to live motors + republish kinematics -----------------------
    if (cfg.apply)
    {
        for (std::size_t i = 0; i < n_motors; ++i)
        {
            auto cal = motors[i]->getCalibration();
            if (result.ticks_to_rad[i] > 0.0)
            {
                cal.ticks_to_rad = result.ticks_to_rad[i];
                cal.bemf_offset = result.bemf_offset[i];
                motors[i]->setCalibration(cal);
            }
        }
        // reset() republishes the kinematics config (with the new ticks_to_rad +
        // bemf_offset) to the STM32 and zeroes both odometry frames.
        odometry_.reset();
        result.applied = true;
        LIBSTP_LOG_INFO("[BemfVelocityTune] applied new ticks_to_rad + bemf_offset and republished kinematics");
    }

    return result;
}

} // namespace libstp::autotune
