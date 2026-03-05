//
// Created by eternalRose on 12/6/25.
//

#ifndef VERSION_IRSENSORCALIBRATION_HPP
#define VERSION_IRSENSORCALIBRATION_HPP
#include <string>
#include <vector>

#include "IRSensor.hpp"

#endif //VERSION_IRSENSORCALIBRATION_HPP

namespace libstp::sensors::ir {
    class IRSensorCalibration {
    public:
        /** Sample readings from all sensors for the requested duration in seconds.
         *  Returns one vector of readings per sensor. */
        static std::vector<std::vector<float>> collectValues(const std::vector<IRSensor*>& sensors, float durationSeconds = 5.0f);

        /** Collect readings and update calibration for each sensor in the batch. */
        static bool calibrateSensors(const std::vector<IRSensor*>& sensors, float durationSeconds = 5.0f,
                                     bool usePre = false, const std::string& set_name = "default");
    };
}
