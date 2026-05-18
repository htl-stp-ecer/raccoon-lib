#include "autotune/velocity_tune.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <thread>
#include <vector>

#include "foundation/logging.hpp"
#include "foundation/types.hpp"

namespace libstp::autotune
{

// ============================================================================
// Constants
// ============================================================================

static constexpr int    kRegressionHalfWindow = 3;   ///< Inflection window half-width.
static constexpr double kSettlePauseMs        = 300.0;
static constexpr double kTailFrac             = 0.20; ///< Tail window (last 20 %).
static constexpr double kSteadyStateFrac      = 0.10; ///< Steady-state window (last 10 %).
static constexpr double kRiseLowFrac          = 0.10; ///< 10 % of y_ss for rise-time.
static constexpr double kRiseHighFrac         = 0.63; ///< 63 % of y_ss for rise-time.

// ============================================================================
// Internal helpers: quadratic regression for inflection detection
// ============================================================================

/**
 * Fit a quadratic  y = a + b*(t-tc) + c*(t-tc)^2  to the window of samples
 * [i-hw, i+hw] around index i using least squares.  Returns {b, 2c} which are
 * the first and second derivatives at the centre point tc.
 *
 * Uses time-stamped samples (not assumed uniform spacing).
 */
static std::pair<double, double> quadraticDerivatives(
    const std::vector<double>& times,
    const std::vector<double>& values,
    int                        centre,
    int                        half_w)
{
    const int n = static_cast<int>(times.size());
    const int lo = std::max(0, centre - half_w);
    const int hi = std::min(n - 1, centre + half_w);

    if (hi - lo < 2)
        return {0.0, 0.0};

    double tc = times[static_cast<std::size_t>(centre)];

    // Accumulate normal equations for  X^T X b = X^T y
    // where X columns are [1, (t-tc), (t-tc)^2].
    double s00 = 0, s10 = 0, s20 = 0, s30 = 0, s40 = 0;
    double r0  = 0, r1  = 0, r2  = 0;

    for (int k = lo; k <= hi; ++k)
    {
        double dt = times[static_cast<std::size_t>(k)] - tc;
        double dt2 = dt * dt;
        double y   = values[static_cast<std::size_t>(k)];

        s00 += 1.0;
        s10 += dt;
        s20 += dt2;
        s30 += dt2 * dt;
        s40 += dt2 * dt2;

        r0 += y;
        r1 += y * dt;
        r2 += y * dt2;
    }

    // Solve the 3x3 normal equations via cofactor expansion (small fixed matrix).
    // [s00 s10 s20]   [a]   [r0]
    // [s10 s20 s30] * [b] = [r1]
    // [s20 s30 s40]   [c]   [r2]
    double det = s00 * (s20 * s40 - s30 * s30)
               - s10 * (s10 * s40 - s30 * s20)
               + s20 * (s10 * s30 - s20 * s20);

    if (std::abs(det) < 1e-30)
        return {0.0, 0.0};

    // Cramer's rule for b (column 1):
    // Replace column 1 of [s00 s10 s20 / s10 s20 s30 / s20 s30 s40] with [r0 r1 r2].
    double det_b = s00 * (r1  * s40 - s30 * r2)
                 - s10 * (r0  * s40 - s20 * r2)
                 + s20 * (r0  * s30 - s20 * r1);
    double b = det_b / det;

    // Cramer's rule for c (column 2):
    double det_c = s00 * (s20 * r2  - r1  * s30)
                 - s10 * (s10 * r2  - r0  * s30)
                 + s20 * (s10 * r1  - r0  * s20);
    double c = det_c / det;

    return {b, 2.0 * c};
}

// ============================================================================
// VelocityTuner
// ============================================================================

VelocityTuner::VelocityTuner(drive::Drive& drive, odometry::IOdometry& odometry)
    : drive_(drive), odometry_(odometry)
{
}

// ----------------------------------------------------------------------------
// runStepResponse
// ----------------------------------------------------------------------------

StepResponseData VelocityTuner::runStepResponse(
    const std::string& axis,
    double             command,
    double             duration_s,
    int                sample_hz) const
{
    using Clock  = std::chrono::steady_clock;
    using Micros = std::chrono::microseconds;

    const Micros period{1'000'000 / sample_hz};

    // Build velocity command for the selected axis.
    foundation::ChassisVelocity vel{};
    if      (axis == "vx") vel = {command, 0.0, 0.0};
    else if (axis == "vy") vel = {0.0, command, 0.0};
    else                   vel = {0.0, 0.0, command}; // "wz"

    drive_.setVelocity(vel);

    StepResponseData data;
    data.times.reserve(static_cast<std::size_t>(duration_s * sample_hz * 1.1));
    data.commanded.reserve(data.times.capacity());
    data.measured.reserve(data.times.capacity());

    auto t0   = Clock::now();
    auto next = t0;

    while (true)
    {
        auto   now     = Clock::now();
        double elapsed = std::chrono::duration<double>(now - t0).count();
        if (elapsed >= duration_s)
            break;

        // Compute actual dt since last sample for the drive update.
        double dt = (data.times.empty())
                    ? (1.0 / sample_hz)
                    : (elapsed - data.times.back());

        drive_.update(dt);

        foundation::ChassisVelocity state = drive_.estimateState();
        double measured = 0.0;
        if      (axis == "vx") measured = state.vx;
        else if (axis == "vy") measured = state.vy;
        else                   measured = state.wz;

        data.times.push_back(elapsed);
        data.commanded.push_back(command);
        data.measured.push_back(measured);

        next += period;
        auto now2 = Clock::now();
        if (next > now2)
            std::this_thread::sleep_until(next);
        else
            next = now2;
    }

    drive_.hardStop();
    std::this_thread::sleep_for(
        std::chrono::milliseconds(static_cast<long long>(kSettlePauseMs)));

    return data;
}

// ----------------------------------------------------------------------------
// identifyPlantInflection
// ----------------------------------------------------------------------------

std::optional<PlantParams> VelocityTuner::identifyPlantInflection(
    const StepResponseData& data,
    double                  command) const
{
    const int n = static_cast<int>(data.times.size());
    if (n < 2 * kRegressionHalfWindow + 3)
        return std::nullopt;

    // Steady-state velocity: mean of last 10 % of samples.
    int ss_start = std::max(0, static_cast<int>(n * (1.0 - kSteadyStateFrac)));
    double y_ss = 0.0;
    int ss_count = n - ss_start;
    for (int i = ss_start; i < n; ++i)
        y_ss += data.measured[static_cast<std::size_t>(i)];
    y_ss /= static_cast<double>(ss_count);

    // Compute d1 and d2 at every interior point.
    std::vector<double> d1(static_cast<std::size_t>(n), 0.0);
    std::vector<double> d2(static_cast<std::size_t>(n), 0.0);

    for (int i = kRegressionHalfWindow; i < n - kRegressionHalfWindow; ++i)
    {
        auto [b, two_c] = quadraticDerivatives(data.times, data.measured,
                                               i, kRegressionHalfWindow);
        d1[static_cast<std::size_t>(i)] = b;
        d2[static_cast<std::size_t>(i)] = two_c;
    }

    // Find inflection: sign change in d2 with largest |d1|.
    int infl_idx = -1;
    double best_d1 = 0.0;

    for (int i = kRegressionHalfWindow + 1; i < n - kRegressionHalfWindow; ++i)
    {
        std::size_t im1 = static_cast<std::size_t>(i - 1);
        std::size_t ii  = static_cast<std::size_t>(i);

        if (d2[im1] * d2[ii] < 0.0)  // sign change
        {
            double abs_d1 = std::abs(d1[ii]);
            if (abs_d1 > best_d1)
            {
                best_d1  = abs_d1;
                infl_idx = i;
            }
        }
    }

    if (infl_idx < 0)
        return std::nullopt;

    double t_infl = data.times[static_cast<std::size_t>(infl_idx)];
    double y_infl = data.measured[static_cast<std::size_t>(infl_idx)];
    double slope  = d1[static_cast<std::size_t>(infl_idx)];

    if (std::abs(slope) < 1e-12)
        return std::nullopt;

    // Tangent line: y = y_infl + slope * (t - t_infl)
    // Intersects y=0:  t_zero = t_infl - y_infl / slope
    // Intersects y=y_ss: t_ss = t_infl + (y_ss - y_infl) / slope
    double t_zero = t_infl - y_infl / slope;
    double t_ss   = t_infl + (y_ss - y_infl) / slope;

    double t_start = data.times.front();
    double Tu = std::max(t_zero - t_start, 0.001);
    double Tg = std::max(t_ss   - t_zero,  0.001);

    double Ks = (std::abs(command) > 1e-9) ? (y_ss / command) : 0.0;

    if (Ks <= 0.0)
        return std::nullopt;

    return PlantParams{Ks, Tu, Tg, "inflection"};
}

// ----------------------------------------------------------------------------
// identifyPlantRiseTime
// ----------------------------------------------------------------------------

PlantParams VelocityTuner::identifyPlantRiseTime(
    const StepResponseData& data,
    double                  command) const
{
    const int n = static_cast<int>(data.times.size());

    // Steady-state velocity: mean of last 10 % of samples.
    int ss_start = std::max(0, static_cast<int>(n * (1.0 - kSteadyStateFrac)));
    double y_ss = 0.0;
    for (int i = ss_start; i < n; ++i)
        y_ss += data.measured[static_cast<std::size_t>(i)];
    y_ss /= static_cast<double>(n - ss_start);

    double Ks = (std::abs(command) > 1e-9) ? (y_ss / command) : 0.0;

    double low_thresh  = std::abs(y_ss) * kRiseLowFrac;
    double high_thresh = std::abs(y_ss) * kRiseHighFrac;

    double t_start = data.times.front();
    double t_low   = -1.0;
    double t_high  = -1.0;

    for (int i = 0; i < n; ++i)
    {
        double abs_m = std::abs(data.measured[static_cast<std::size_t>(i)]);
        double t     = data.times[static_cast<std::size_t>(i)];
        if (t_low < 0.0 && abs_m >= low_thresh)
            t_low = t;
        if (t_low >= 0.0 && t_high < 0.0 && abs_m >= high_thresh)
        {
            t_high = t;
            break;
        }
    }

    double Tu = (t_low >= 0.0) ? std::max(t_low - t_start, 0.01) : 0.01;
    double Tg = (t_high >= 0.0 && t_low >= 0.0 && t_high > t_low)
                ? std::max(t_high - t_low, 0.01)
                : 0.01;

    return PlantParams{Ks, Tu, Tg, "rise_time"};
}

// ----------------------------------------------------------------------------
// computeChrGains
// ----------------------------------------------------------------------------

foundation::PidGains VelocityTuner::computeChrGains(
    const PlantParams&       plant,
    const VelocityTuneConfig& cfg) const
{
    if (plant.Ks <= 0.0 || plant.Tu <= 0.0 || plant.Tg <= 0.0)
        return {0.0, 0.0, 0.0};

    double kp = cfg.chr_kp_scale * plant.Tg / (plant.Ks * plant.Tu);
    double ki = cfg.chr_ki_scale * kp / plant.Tg;
    double kd = cfg.chr_kd_scale * plant.Tg / plant.Ks;

    if (kp <= 0.0)
        return {0.0, 0.0, 0.0};

    return {kp, ki, kd};
}

// ----------------------------------------------------------------------------
// computeIse
// ----------------------------------------------------------------------------

double VelocityTuner::computeIse(const StepResponseData& data)
{
    const std::size_t n = data.times.size();
    if (n < 2)
        return 0.0;

    double ise = 0.0;
    for (std::size_t i = 1; i < n; ++i)
    {
        double dt  = data.times[i] - data.times[i - 1];
        double e0  = data.commanded[i - 1] - data.measured[i - 1];
        double e1  = data.commanded[i]     - data.measured[i];
        ise += 0.5 * (e0 * e0 + e1 * e1) * dt;  // trapezoidal
    }
    return ise;
}

// ----------------------------------------------------------------------------
// tailMeanAbs
// ----------------------------------------------------------------------------

double VelocityTuner::tailMeanAbs(const StepResponseData& data)
{
    const std::size_t n = data.measured.size();
    if (n == 0)
        return 0.0;

    std::size_t tail_start = static_cast<std::size_t>(
        static_cast<double>(n) * (1.0 - kTailFrac));
    tail_start = std::min(tail_start, n - 1);

    double sum = 0.0;
    std::size_t count = n - tail_start;
    for (std::size_t i = tail_start; i < n; ++i)
        sum += std::abs(data.measured[i]);

    return sum / static_cast<double>(count);
}

// ----------------------------------------------------------------------------
// peakAbs
// ----------------------------------------------------------------------------

double VelocityTuner::peakAbs(const StepResponseData& data)
{
    double peak = 0.0;
    for (double v : data.measured)
        peak = std::max(peak, std::abs(v));
    return peak;
}

// ----------------------------------------------------------------------------
// tuneAxis
// ----------------------------------------------------------------------------

VelocityTuneResult VelocityTuner::tuneAxis(
    const std::string&       axis,
    double                   max_velocity,
    const VelocityTuneConfig& cfg) const
{
    if (axis != "vx" && axis != "vy" && axis != "wz")
    {
        LIBSTP_LOG_WARN("[VelocityTuner] Unknown axis '{}' — skipping", axis);
        VelocityTuneResult r;
        r.axis = axis;
        return r;
    }

    LIBSTP_LOG_INFO("[VelocityTuner] Tuning axis '{}'", axis);

    const double command = max_velocity * cfg.step_command_frac;

    // Save current drive config before any modifications.
    auto saved_cfg = drive_.getVelocityControlConfig();

    VelocityTuneResult result;
    result.axis = axis;
    result.ff   = {0.0, 1.0, 0.0};

    // ---- Step 1: Baseline step response ----
    LIBSTP_LOG_INFO("[VelocityTuner] [{}] Running baseline step response (command={:.4f})",
                    axis, command);

    StepResponseData baseline = runStepResponse(axis, command, cfg.step_duration_s,
                                                cfg.sample_hz);
    result.baseline_ise = computeIse(baseline);

    LIBSTP_LOG_INFO("[VelocityTuner] [{}] Baseline ISE={:.6f}", axis, result.baseline_ise);

    // ---- Step 2: Reject if response too small ----
    double tail = tailMeanAbs(baseline);
    if (tail < std::abs(command) * cfg.min_response_frac)
    {
        LIBSTP_LOG_WARN(
            "[VelocityTuner] [{}] Insufficient response: tail={:.4f} < {:.4f} — skipping",
            axis, tail, std::abs(command) * cfg.min_response_frac);
        result.plant.method = "insufficient_response";
        result.accepted     = false;
        return result;
    }

    // ---- Step 3: Plant identification ----
    std::optional<PlantParams> opt_plant = identifyPlantInflection(baseline, command);

    if (opt_plant.has_value())
    {
        result.plant = *opt_plant;
        LIBSTP_LOG_INFO("[VelocityTuner] [{}] Plant (inflection): Ks={:.4f}, Tu={:.4f}s, "
                        "Tg={:.4f}s", axis, result.plant.Ks, result.plant.Tu, result.plant.Tg);
    }
    else
    {
        result.plant = identifyPlantRiseTime(baseline, command);
        LIBSTP_LOG_INFO("[VelocityTuner] [{}] Plant (rise_time): Ks={:.4f}, Tu={:.4f}s, "
                        "Tg={:.4f}s", axis, result.plant.Ks, result.plant.Tu, result.plant.Tg);
    }

    // ---- Step 4: Reject bad Ks ----
    if (result.plant.Ks <= 0.0)
    {
        LIBSTP_LOG_WARN("[VelocityTuner] [{}] Ks <= 0 — rejecting", axis);
        result.accepted = false;
        return result;
    }

    // ---- Step 5: Low dead-time early return ----
    double dt_ratio = result.plant.Tu / result.plant.Tg;
    if (dt_ratio <= cfg.low_dead_time_ratio
        && std::abs(result.plant.Ks - 1.0) <= cfg.low_dead_time_gain_tol)
    {
        LIBSTP_LOG_INFO(
            "[VelocityTuner] [{}] Low dead-time (Tu/Tg={:.4f}) + near-unity Ks — "
            "skipping CHR, returning baseline gains", axis, dt_ratio);
        result.plant.method += "+low_dead_time_baseline";
        result.accepted   = false;  // no new gains were applied
        result.tuned_ise  = result.baseline_ise;
        return result;
    }

    // ---- Step 6: Compute CHR gains ----
    result.pid = computeChrGains(result.plant, cfg);

    if (result.pid.kp <= 0.0)
    {
        LIBSTP_LOG_WARN("[VelocityTuner] [{}] CHR gave kp <= 0 — rejecting", axis);
        result.accepted = false;
        return result;
    }

    LIBSTP_LOG_INFO("[VelocityTuner] [{}] CHR gains: kp={:.4f}, ki={:.4f}, kd={:.4f}",
                    axis, result.pid.kp, result.pid.ki, result.pid.kd);

    // ---- Step 7: Apply candidate gains ----
    auto test_cfg = drive_.getVelocityControlConfig();
    drive::AxisVelocityControlConfig axis_cfg;
    axis_cfg.pid = result.pid;
    axis_cfg.ff  = foundation::Feedforward{0.0, 1.0, 0.0};

    if      (axis == "vx") test_cfg.vx = axis_cfg;
    else if (axis == "vy") test_cfg.vy = axis_cfg;
    else                   test_cfg.wz = axis_cfg;

    drive_.setVelocityControlConfig(test_cfg);
    drive_.resetVelocityControllers();

    // ---- Step 8: Tuned step response ----
    LIBSTP_LOG_INFO("[VelocityTuner] [{}] Running tuned step response", axis);
    StepResponseData tuned = runStepResponse(axis, command, cfg.step_duration_s,
                                             cfg.sample_hz);
    result.tuned_ise = computeIse(tuned);

    LIBSTP_LOG_INFO("[VelocityTuner] [{}] Tuned ISE={:.6f} (baseline={:.6f})",
                    axis, result.tuned_ise, result.baseline_ise);

    // ---- Step 9: Accept or revert ----
    if (result.tuned_ise < result.baseline_ise)
    {
        result.accepted = true;
        LIBSTP_LOG_INFO("[VelocityTuner] [{}] Gains ACCEPTED (ISE improved {:.2f}%%)",
                        axis,
                        100.0 * (result.baseline_ise - result.tuned_ise) / result.baseline_ise);
    }
    else
    {
        result.accepted = false;
        LIBSTP_LOG_WARN("[VelocityTuner] [{}] Gains REJECTED (ISE did not improve) — "
                        "reverting", axis);
        drive_.setVelocityControlConfig(saved_cfg);
        drive_.resetVelocityControllers();
    }

    return result;
}

// ----------------------------------------------------------------------------
// tune (multi-axis)
// ----------------------------------------------------------------------------

std::map<std::string, VelocityTuneResult> VelocityTuner::tune(
    const std::vector<std::string>&      axes,
    const std::map<std::string, double>& max_velocities,
    const VelocityTuneConfig&            cfg) const
{
    LIBSTP_LOG_INFO("============================================================");
    LIBSTP_LOG_INFO("  VELOCITY TUNING");
    {
        std::string axes_str;
        for (const auto& a : axes) { if (!axes_str.empty()) axes_str += ", "; axes_str += a; }
        LIBSTP_LOG_INFO("  Axes: {}", axes_str);
    }
    LIBSTP_LOG_INFO("============================================================");

    std::map<std::string, VelocityTuneResult> results;

    for (const auto& axis : axes)
    {
        auto it = max_velocities.find(axis);
        if (it == max_velocities.end())
        {
            LIBSTP_LOG_WARN("[VelocityTuner] No max_velocity for axis '{}' — skipping",
                            axis);
            continue;
        }

        results[axis] = tuneAxis(axis, it->second, cfg);

        // Settle between axes.
        drive_.hardStop();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    // Summary.
    LIBSTP_LOG_INFO("============================================================");
    LIBSTP_LOG_INFO("  VELOCITY TUNING RESULTS");
    LIBSTP_LOG_INFO("============================================================");
    for (const auto& [ax, r] : results)
    {
        LIBSTP_LOG_INFO("  {}: accepted={}, Ks={:.4f}, Tu={:.4f}s, Tg={:.4f}s, "
                        "kp={:.4f}, ki={:.4f}, kd={:.4f}, "
                        "baseline_ise={:.6f}, tuned_ise={:.6f}",
                        ax, r.accepted ? "yes" : "no",
                        r.plant.Ks, r.plant.Tu, r.plant.Tg,
                        r.pid.kp, r.pid.ki, r.pid.kd,
                        r.baseline_ise, r.tuned_ise);
    }
    LIBSTP_LOG_INFO("============================================================");

    return results;
}

} // namespace libstp::autotune
