#pragma once

#include "calibration/motor/data/velocity_data.hpp"

namespace libstp::calibration::analysis
{
    struct RelayParams
    {
        double Ku{0.0};         // Ultimate gain
        double Tu{0.0};         // Ultimate period
        double amplitude{0.0};  // Oscillation amplitude
        int cycles{0};          // Number of complete cycles
    };

    RelayParams analyzeOscillations(
        const data::VelocityProfile& profile,
        double relay_amplitude
    );
}
