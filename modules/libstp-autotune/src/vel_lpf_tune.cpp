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

/**
 * Sample every motor's BEMF simultaneously at sample_hz for measure_duration_s.
 * The caller is responsible for having the motors already spinning at steady
 * state before this is called.
 */
std::map<int, std::vector<int>>
sampleBemfWhileSpinning(const std::vector<hal::motor::IMotor*>& motors,
                        const VelLpfConfig& cfg)
{
    using Clock  = std::chrono::steady_clock;
    using Micros = std::chrono::microseconds;
    const int    sample_hz = std::max(1, cfg.sample_hz);
    const Micros period{1'000'000 / sample_hz};

    std::map<int, std::vector<int>> raw;
    const auto reserve_n =
        static_cast<std::size_t>(cfg.measure_duration_s * sample_hz * 1.1);
    for (auto* m : motors)
        if (m != nullptr) raw[m->getPort()].reserve(reserve_n);

    auto t0   = Clock::now();
    auto next = t0;
    while (true)
    {
        const auto now = Clock::now();
        if (std::chrono::duration<double>(now - t0).count() >= cfg.measure_duration_s)
            break;
        for (auto* m : motors)
            if (m != nullptr) raw[m->getPort()].push_back(m->getBemf());
        next += period;
        const auto now2 = Clock::now();
        if (next > now2)
            std::this_thread::sleep_until(next);
        else
            next = now2;
    }
    return raw;
}

} // namespace

VelLpfTuner::VelLpfTuner(const std::vector<hal::motor::IMotor*>& motors)
    : motors_(motors)
{
}

VelLpfResult VelLpfTuner::scoreFromRaw(hal::motor::IMotor* motor,
                                       std::vector<int>    raw,
                                       const VelLpfConfig& cfg) const
{
    VelLpfResult result;
    if (motor == nullptr)
        return result;

    result.motor_port    = motor->getPort();
    result.initial_alpha = motor->getCalibration().vel_lpf_alpha;
    result.tuned_alpha   = result.initial_alpha;

    // Preserve the raw BEMF series for the offline report (raw-vs-filtered
    // overlay). Stored regardless of whether the sweep finds a usable signal.
    result.raw_bemf = raw;

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

        result.sweep.push_back(
            VelLpfSweepPoint{alpha, p.variance, p.lag_change_rate, score});

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

VelLpfResult VelLpfTuner::tuneMotor(hal::motor::IMotor* motor,
                                    const VelLpfConfig& cfg) const
{
    if (motor == nullptr)
    {
        LIBSTP_LOG_WARN("[VelLpfTuner] Null motor pointer — skipping");
        return {};
    }

    LIBSTP_LOG_INFO("[VelLpfTuner] Tuning port={} (spin {}% / settle {:.1f}s / "
                    "sample {:.1f}s)",
                    motor->getPort(), cfg.spin_percent, cfg.settle_s,
                    cfg.measure_duration_s);

    // The LPF filters the *running* velocity estimate, so the motor must be
    // spinning while we sample. Spin this single motor (the chassis curves),
    // settle to steady state, capture BEMF, then stop.
    motor->setSpeed(cfg.spin_percent);
    std::this_thread::sleep_for(
        std::chrono::milliseconds(static_cast<int>(cfg.settle_s * 1000.0)));
    auto raw = sampleBemfWhileSpinning({motor}, cfg);
    motor->brake();

    return scoreFromRaw(motor, std::move(raw[motor->getPort()]), cfg);
}

std::map<int, VelLpfResult> VelLpfTuner::tune(const VelLpfConfig& cfg) const
{
    LIBSTP_LOG_INFO("============================================================");
    LIBSTP_LOG_INFO("  VELOCITY LPF TUNING");
    LIBSTP_LOG_INFO("============================================================");

    std::map<int, VelLpfResult> results;

    std::vector<hal::motor::IMotor*> live;
    for (auto* m : motors_)
        if (m != nullptr) live.push_back(m);
    if (live.empty())
    {
        LIBSTP_LOG_WARN("[VelLpfTuner] no drive motors — skipping");
        return results;
    }

    // Spin every drive motor forward together so the chassis tracks roughly
    // straight, let the velocities settle, then sample all motors during a
    // single run. Sampling at standstill only captures deadzone noise.
    LIBSTP_LOG_INFO("[VelLpfTuner] spinning {} motors at {}% (settle {:.1f}s, "
                    "sample {:.1f}s)",
                    live.size(), cfg.spin_percent, cfg.settle_s,
                    cfg.measure_duration_s);
    for (auto* m : live) m->setSpeed(cfg.spin_percent);
    std::this_thread::sleep_for(
        std::chrono::milliseconds(static_cast<int>(cfg.settle_s * 1000.0)));
    auto raw = sampleBemfWhileSpinning(live, cfg);
    for (auto* m : live) m->brake();

    for (auto* m : live)
        results[m->getPort()] = scoreFromRaw(m, std::move(raw[m->getPort()]), cfg);

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
