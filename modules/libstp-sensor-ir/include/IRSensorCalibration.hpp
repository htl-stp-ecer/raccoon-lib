//
// Created by eternalRose on 12/6/25.
//

#ifndef VERSION_IRSENSORCALIBRATION_HPP
#define VERSION_IRSENSORCALIBRATION_HPP
#include <vector>

#include "IRSensor.hpp"

#endif //VERSION_IRSENSORCALIBRATION_HPP

namespace libstp::sensors::ir {
    class IRSensorCalibration {
    public:
        static std::vector<float> collectValues(const std::vector<IRSensor*>& sensors, float durationSeconds = 5.0f);

        static bool calibrateSensors(const std::vector<IRSensor*>& sensors, float durationSeconds = 5.0f);
    };
}