#include "autotune/static_friction.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <thread>
#include <vector>

#include "foundation/logging.hpp"

namespace libstp::autotune
{

namespace
{

int sweepDirection(hal::motor::IMotor*         motor,
                   const StaticFrictionConfig& cfg,
                   bool                        forward)
{
    const int sign = forward ? +1 : -1;
    const int n_samples = std::max(1, cfg.samples_per_step);

    for (int pct = cfg.start_pct; pct <= cfg.max_pct; pct += std::max(1, cfg.step_pct))
    {
        const int command = sign * pct;
        motor->setSpeed(command);
        std::this_thread::sleep_for(std::chrono::milliseconds(cfg.dwell_ms));

        std::vector<int> samples;
        samples.reserve(static_cast<std::size_t>(n_samples));
        for (int s = 0; s < n_samples; ++s)
        {
            samples.push_back(std::abs(motor->getBemf()));
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        std::sort(samples.begin(), samples.end());
        const int median = samples[static_cast<std::size_t>(samples.size() / 2)];

        LIBSTP_LOG_DEBUG("[StaticFrictionMeasurer] port={} dir={} pct={} median_bemf={}",
                         motor->getPort(), forward ? "+" : "-", pct, median);

        if (median >= cfg.motion_threshold)
        {
            motor->brake();
            return pct;
        }
    }

    motor->brake();
    return 0;
}

} // namespace

StaticFrictionMeasurer::StaticFrictionMeasurer(
    const std::vector<hal::motor::IMotor*>& motors)
    : motors_(motors)
{
}

StaticFrictionResult StaticFrictionMeasurer::measureMotor(
    hal::motor::IMotor*         motor,
    const StaticFrictionConfig& cfg) const
{
    StaticFrictionResult result;
    if (motor == nullptr)
    {
        LIBSTP_LOG_WARN("[StaticFrictionMeasurer] Null motor pointer — skipping");
        return result;
    }
    result.motor_port = motor->getPort();

    LIBSTP_LOG_INFO("[StaticFrictionMeasurer] Measuring kS on motor port={}",
                    result.motor_port);

    // Quiet the motor and reset the encoder so that the very first samples
    // aren't poisoned by leftover motion.
    motor->brake();
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    motor->resetPositionCounter();

    // Forward sweep.
    const int ks_pos = sweepDirection(motor, cfg, /*forward=*/true);

    motor->brake();
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    // Reverse sweep.
    const int ks_neg = sweepDirection(motor, cfg, /*forward=*/false);

    motor->brake();

    result.ks_positive_pct = ks_pos;
    result.ks_negative_pct = ks_neg;
    result.ks_avg_pct      = (ks_pos + ks_neg) / 2;
    result.measured        = (ks_pos > 0 && ks_neg > 0);

    LIBSTP_LOG_INFO("[StaticFrictionMeasurer] port={} ks_pos={}%% ks_neg={}%% "
                    "ks_avg={}%% measured={}",
                    result.motor_port, ks_pos, ks_neg, result.ks_avg_pct,
                    result.measured ? "yes" : "no");

    return result;
}

std::map<int, StaticFrictionResult> StaticFrictionMeasurer::measure(
    const StaticFrictionConfig& cfg) const
{
    LIBSTP_LOG_INFO("============================================================");
    LIBSTP_LOG_INFO("  STATIC-FRICTION MEASUREMENT");
    LIBSTP_LOG_INFO("============================================================");

    std::map<int, StaticFrictionResult> results;
    for (auto* motor : motors_)
    {
        if (motor == nullptr)
            continue;
        const int port = motor->getPort();
        results[port] = measureMotor(motor, cfg);
    }

    LIBSTP_LOG_INFO("============================================================");
    LIBSTP_LOG_INFO("  STATIC-FRICTION RESULTS");
    LIBSTP_LOG_INFO("============================================================");
    for (const auto& [port, r] : results)
    {
        LIBSTP_LOG_INFO("  port={}: ks_pos={}%%, ks_neg={}%%, ks_avg={}%%, measured={}",
                        port, r.ks_positive_pct, r.ks_negative_pct,
                        r.ks_avg_pct, r.measured ? "yes" : "no");
    }
    LIBSTP_LOG_INFO("============================================================");

    return results;
}

} // namespace libstp::autotune
