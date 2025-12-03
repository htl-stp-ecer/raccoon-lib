#pragma once

#include "calibration/motor/data/velocity_data.hpp"

namespace libstp::calibration::analysis
{
    struct StepResponseParams
    {
        double tau{0.0};        // Time constant
        double K{0.0};          // Steady-state gain
        double delay{0.0};      // Dead time
        double fit_error{0.0};  // RMS fit error
    };

    StepResponseParams fitStepResponse(
        const data::VelocityProfile& profile,
        double initial_velocity
    );
}
