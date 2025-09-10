//
// Created by tobias on 9/8/25.
//

#include "drive/rate_limiter.hpp"

libstp::drive::RateLimiter::RateLimiter(const double max_rate) : max_rate_(max_rate)
{
}

void libstp::drive::RateLimiter::setMaxRate(const double r)
{
    max_rate_ = std::max(0.0, r);
}

double libstp::drive::RateLimiter::maxRate() const
{
    return max_rate_;
}

double libstp::drive::RateLimiter::step(const double target, const double current_prev, const double dt) const
{
    if (max_rate_ <= 0.0 || dt <= 0.0) return target;
    const double dv = target - current_prev;
    const double cap = max_rate_ * dt;
    if (dv > cap) return current_prev + cap;
    if (dv < -cap) return current_prev - cap;
    return target;
}
