#include "autotune/vel_lpf_tune.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <thread>
#include <vector>

#include "foundation/logging.hpp"

namespace libstp::autotune
{

namespace
{

struct ScoreParts
{
    double variance{0.0};
    double lag_change_rate{0.0};
};

/**
 * Run the candidate alpha through the previously-captured raw samples and
 * compute (filtered-variance, lag-change-rate).
 *
 * lag_change_rate is the standard deviation of |filt[i] - filt[i-1]|. A
 * heavily lagged filter is very smooth — the magnitude of step-to-step
 * change varies very little, so the std-dev of that difference is small.
 * We use 1 / lag_change_rate as the "lag penalty" so smaller change-rate ->
 * larger penalty.
 */
ScoreParts evaluateAlpha(const std::vector<int>& raw, double alpha)
{
    ScoreParts parts;
    if (raw.empty())
        return parts;

    std::vector<double> filt;
    filt.reserve(raw.size());

    double y = static_cast<double>(raw.front());
    for (int r : raw)
    {
        y = (1.0 - alpha) * y + alpha * static_cast<double>(r);
        filt.push_back(y);
    }

    // Variance of filtered values.
    double mean = 0.0;
    for (double v : filt) mean += v;
    mean /= static_cast<double>(filt.size());

    double var = 0.0;
    for (double v : filt)
    {
        const double d = v - mean;
        var += d * d;
    }
    var /= static_cast<double>(filt.size());

    // Std-dev of |Δfilt| as a proxy for how much the filter "moves".
    if (filt.size() < 2)
        return parts;

    std::vector<double> diffs;
    diffs.reserve(filt.size() - 1);
    double dmean = 0.0;
    for (std::size_t i = 1; i < filt.size(); ++i)
    {
        const double dabs = std::abs(filt[i] - filt[i - 1]);
        diffs.push_back(dabs);
        dmean += dabs;
    }
    dmean /= static_cast<double>(diffs.size());

    double dvar = 0.0;
    for (double d : diffs)
    {
        const double dd = d - dmean;
        dvar += dd * dd;
    }
    dvar /= static_cast<double>(diffs.size());

    parts.variance        = var;
    parts.lag_change_rate = std::sqrt(dvar);
    return parts;
}

} // namespace

VelLpfTuner::VelLpfTuner(const std::vector<hal::motor::IMotor*>& motors)
    : motors_(motors)
{
}

VelLpfResult VelLpfTuner::tuneMotor(hal::motor::IMotor* motor,
                                    const VelLpfConfig& cfg) const
{
    VelLpfResult result;
    if (motor == nullptr)
    {
        LIBSTP_LOG_WARN("[VelLpfTuner] Null motor pointer — skipping");
        return result;
    }
    result.motor_port    = motor->getPort();
    result.initial_alpha = motor->getCalibration().vel_lpf_alpha;
    result.tuned_alpha   = result.initial_alpha;

    LIBSTP_LOG_INFO("[VelLpfTuner] Tuning vel_lpf_alpha on motor port={} "
                    "(initial alpha={:.3f})",
                    result.motor_port, result.initial_alpha);

    // Ensure the motor is quiescent before sampling.
    motor->brake();

    // ---- Sample raw BEMF at sample_hz for measure_duration_s -------------
    using Clock  = std::chrono::steady_clock;
    using Micros = std::chrono::microseconds;
    const int sample_hz = std::max(1, cfg.sample_hz);
    const Micros period{1'000'000 / sample_hz};

    std::vector<int> raw;
    raw.reserve(static_cast<std::size_t>(cfg.measure_duration_s * sample_hz * 1.1));

    auto t0   = Clock::now();
    auto next = t0;
    while (true)
    {
        auto now = Clock::now();
        const double elapsed = std::chrono::duration<double>(now - t0).count();
        if (elapsed >= cfg.measure_duration_s)
            break;
        raw.push_back(motor->getBemf());
        next += period;
        auto now2 = Clock::now();
        if (next > now2)
            std::this_thread::sleep_until(next);
        else
            next = now2;
    }

    if (raw.size() < 2)
    {
        LIBSTP_LOG_WARN("[VelLpfTuner] port={} too few samples ({}) — skipping",
                        result.motor_port, raw.size());
        return result;
    }

    // ---- Sweep alpha ------------------------------------------------------
    double best_alpha = result.initial_alpha;
    double best_score = std::numeric_limits<double>::infinity();
    const double step = (cfg.alpha_step > 1e-9) ? cfg.alpha_step : 0.05;

    for (double a = cfg.alpha_min; a <= cfg.alpha_max + 1e-9; a += step)
    {
        const double alpha = std::clamp(a, 0.0, 1.0);
        const ScoreParts p = evaluateAlpha(raw, alpha);

        // Guard against zero lag-change-rate (division by zero).
        const double lag_inv = 1.0 / (p.lag_change_rate + 1e-9);
        const double score   = cfg.noise_weight * p.variance + cfg.lag_weight * lag_inv;

        LIBSTP_LOG_DEBUG("[VelLpfTuner] port={} alpha={:.3f} variance={:.4f} "
                         "lag_rate={:.4f} score={:.6f}",
                         result.motor_port, alpha, p.variance,
                         p.lag_change_rate, score);

        if (score < best_score)
        {
            best_score = score;
            best_alpha = alpha;
        }
    }

    result.tuned_alpha = best_alpha;
    result.min_score   = best_score;

    // ---- Apply the new calibration ---------------------------------------
    foundation::MotorCalibration cal = motor->getCalibration();
    cal.vel_lpf_alpha = std::clamp(best_alpha, 0.0, 1.0);
    motor->setCalibration(cal);
    result.applied = true;

    LIBSTP_LOG_INFO("[VelLpfTuner] port={} tuned alpha {:.3f} -> {:.3f} "
                    "(min_score={:.6f})",
                    result.motor_port, result.initial_alpha,
                    result.tuned_alpha, result.min_score);

    return result;
}

std::map<int, VelLpfResult> VelLpfTuner::tune(const VelLpfConfig& cfg) const
{
    LIBSTP_LOG_INFO("============================================================");
    LIBSTP_LOG_INFO("  VELOCITY LPF TUNING");
    LIBSTP_LOG_INFO("============================================================");

    std::map<int, VelLpfResult> results;
    for (auto* motor : motors_)
    {
        if (motor == nullptr)
            continue;
        results[motor->getPort()] = tuneMotor(motor, cfg);
    }

    LIBSTP_LOG_INFO("============================================================");
    LIBSTP_LOG_INFO("  VELOCITY LPF TUNING RESULTS");
    LIBSTP_LOG_INFO("============================================================");
    for (const auto& [port, r] : results)
    {
        LIBSTP_LOG_INFO("  port={}: alpha {:.3f} -> {:.3f} (score={:.6f}, applied={})",
                        port, r.initial_alpha, r.tuned_alpha, r.min_score,
                        r.applied ? "yes" : "no");
    }
    LIBSTP_LOG_INFO("============================================================");

    return results;
}

} // namespace libstp::autotune
