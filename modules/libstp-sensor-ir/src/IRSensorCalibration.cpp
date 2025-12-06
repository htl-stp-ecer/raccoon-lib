//
// Created by eternalRose on 12/6/25.
//

#include "IRSensorCalibration.hpp"

#include <chrono>
#include <thread>

#include "button/button.hpp"
#include "foundation/logging.hpp"

using namespace  libstp::sensors::ir;

IRSensorCalibration::IRSensorCalibration(const int &buttonPort) {
    button::Button::setDigital(buttonPort);
}

void IRSensorCalibration::calibrateSensors(const std::vector<IRSensor*>& sensors, float durationSeconds) {
    LIBSTP_LOG_INFO("Press the button when ready to scan the values");
    button::Button::waitForButtonPress();

    std::vector<float> values = collectValues(sensors, durationSeconds);

    if (values.empty()) {
        LIBSTP_LOG_WARN("Got no values for WHITE or BLACK");
        return;
    }

    for (auto* sensor : sensors) {
        sensor->calibrate(values);
    }
}


std::vector<float> IRSensorCalibration::collectValues(const std::vector<IRSensor*>& sensors, float durationSeconds) {
    std::vector<float> values;

    auto t_end = std::chrono::steady_clock::now() +
                 std::chrono::milliseconds(static_cast<long long>(durationSeconds * 1000));

    LIBSTP_LOG_INFO("Collecting Sensor Values");

    while (std::chrono::steady_clock::now() < t_end) {
        for (auto* sensor : sensors) {
            values.push_back(sensor->read());
            std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<long long>(10)));
        }
    }

    return values;
}
