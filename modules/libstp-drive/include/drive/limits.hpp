//
// Created by tobias on 9/8/25.
//

#pragma once

namespace libstp::drive
{
    struct MotionLimits
    {
        double max_v{2.0};
        double max_omega{8.0};
    };

    struct WheelLimits
    {
        double max_w{100.0};
        double max_w_dot{1000.0};
    };
}
