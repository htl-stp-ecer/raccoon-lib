//
// Created by tobias on 9/8/25.
//

#include "drive/rate_limiter.hpp"
#include "foundation/config.hpp"

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <string>

namespace
{
    [[nodiscard]] bool speedRampsEnabled()
    {
        static constexpr bool enabled = [] {
            return true;
        }();
        return enabled;
    }
}

using namespace libstp::drive;

RateLimiter::RateLimiter(const double max_rate)
{
    setMaxRate(max_rate);
}

void RateLimiter::setMaxRate(const double r)
{
    if (!speedRampsEnabled())
    {
        max_rate_ = 0.0;
        LIBSTP_LOG_INFO(
            "RateLimiter::setMaxRate speed ramps disabled (requested max_rate={} ignored)",
            r);
        return;
    }

    max_rate_ = std::max(0.0, r);
    LIBSTP_LOG_INFO("RateLimiter::setMaxRate max_rate={}", max_rate_);
}

double RateLimiter::maxRate() const
{
    LIBSTP_LOG_TRACE("RateLimiter::maxRate -> {}", max_rate_);
    return max_rate_;
}

double RateLimiter::step(const double target, const double current_prev, const double dt, double& out_accel) const
{
    if (max_rate_ <= 0.0 || dt <= 0.0)
    {
        out_accel = (target - current_prev) / (dt > 0.0 ? dt : 1.0);
        LIBSTP_LOG_TRACE(
            "RateLimiter::step bypass max_rate={} dt={} target={} current_prev={} accel={}",
            max_rate_,
            dt,
            target,
            current_prev,
            out_accel);
        return target;
    }
    const double dv = target - current_prev;
    const double cap = max_rate_ * dt;
    double next = target;
    if (dv > cap) next = current_prev + cap;
    else if (dv < -cap) next = current_prev - cap;

    out_accel = (next - current_prev) / (dt > 0.0 ? dt : 1.0);
    LIBSTP_LOG_TRACE(
        "RateLimiter::step target={} current_prev={} dt={} max_rate={} cap={} next={} accel={}",
        target,
        current_prev,
        dt,
        max_rate_,
        cap,
        next,
        out_accel);
    return next;
}
