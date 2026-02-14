//
// Created by tobias on 9/15/25.
//

#pragma once

namespace libstp::foundation
{
    struct PidGains
    {
        double kp{1.0}, ki{0.0}, kd{0.0};
    };

    struct Feedforward
    {
        double kS{0.0};
        double kV{0.0};
        double kA{0.0};
    };

    struct MotorCalibration
    {
        double ticks_to_rad{2.0 * 3.14159265359 / 1440.0};
        double vel_lpf_alpha{0.5};
    };
}