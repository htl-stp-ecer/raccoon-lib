//
// Created by tobias on 9/8/25.
//

#pragma once
#include <algorithm>
#include <cmath>

namespace libstp::drive
{
    class RateLimiter
    {
    public:
        explicit RateLimiter(double max_rate = 0.0);

        void setMaxRate(double r);
        [[nodiscard]] double maxRate() const;

        [[nodiscard]] double step(double target, double current_prev, double dt, double& out_accel) const;

    private:
        double max_rate_{0.0};
    };
}
