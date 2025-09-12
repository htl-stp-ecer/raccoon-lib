//
// Created by tobias on 9/8/25.
//

#include "drive/rate_limiter.hpp"

using namespace libstp::drive;

RateLimiter::RateLimiter(const double max_rate) : max_rate_(max_rate)
{
}

void RateLimiter::setMaxRate(const double r) { max_rate_ = std::max(0.0, r); }

double RateLimiter::maxRate() const { return max_rate_; }

double RateLimiter::step(const double target, const double current_prev, const double dt, double& out_accel) const
{
    if (max_rate_ <= 0.0 || dt <= 0.0)
    {
        out_accel = (target - current_prev) / (dt > 0.0 ? dt : 1.0);
        return target;
    }
    const double dv = target - current_prev;
    const double cap = max_rate_ * dt;
    double next = target;
    if (dv > cap) next = current_prev + cap;
    else if (dv < -cap) next = current_prev - cap;

    out_accel = (next - current_prev) / (dt > 0.0 ? dt : 1.0);
    return next;
}
