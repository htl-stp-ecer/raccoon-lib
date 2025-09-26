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

    struct Deadzone
    {
        bool enable{true};
        double zero_window_percent{10.0};
        double start_percent{40.0};
        double release_percent{15.0};
    };

    struct MotorCalibration
    {
        Feedforward ff{};
        PidGains pid{1.0, 0.0, 0.0};
        Deadzone deadzone{true, 10.0, 40.0, 15.0};

        double max_percent_output{100.0};
        double ticks_to_rad{2.0 * 3.14159265359 / 1440.0};
        double vel_lpf_alpha{0.2};
    };
}