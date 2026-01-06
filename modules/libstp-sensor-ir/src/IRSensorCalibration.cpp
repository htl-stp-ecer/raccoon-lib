//
// Created by eternalRose on 12/6/25.
//

#include "IRSensorCalibration.hpp"

#include <chrono>
#include <thread>
#include <hal/ScreenRender.hpp>
#include "button/button.hpp"
#include "foundation/logging.hpp"

using namespace libstp::sensors::ir;

IRSensorCalibration::IRSensorCalibration(const int &buttonPort) {
    button::Button::instance().setDigital(buttonPort);
}

bool IRSensorCalibration::calibrateSensors(const std::vector<IRSensor *> &sensors,
                                           float durationSeconds) {
    constexpr int MAX_ATTEMPTS = 5;
    hal::screen_render::ScreenRender::instance().setCurrentScreenSetting("calibrate_sensors");

    for (int attempt = 1; attempt <= MAX_ATTEMPTS; attempt++) {
        LIBSTP_LOG_INFO("Press the button when ready to scan the values (Attempt "
            + std::to_string(attempt) + "/" + std::to_string(MAX_ATTEMPTS) + ")");
        button::Button::instance().waitForButtonPress();
        hal::screen_render::ScreenRender::instance().sendState(
            R"({"type":"IR","state":"readData"})"
        );

        std::vector<float> values = collectValues(sensors, durationSeconds);
        if (values.empty()) {
            LIBSTP_LOG_WARN("No sensor values collected.");
            continue;
        }

        bool allGood = true;

        for (auto *sensor: sensors) {
            if (!sensor->calibrate(values)) {
                allGood = false;
                break;
            }
        }

        if (allGood) {
            LIBSTP_LOG_INFO("All sensors calibrated successfully.");
            std::ostringstream json;
            json << R"({"type":"IR","state":"confirm",)"
                    << R"("collected_values":[)";

            for (size_t i = 0; i < values.size(); ++i) {
                json << values[i];
                if (i + 1 < values.size()) {
                    json << ",";
                }
            }

            json << "],"
                    << R"("black_thresh":)" << sensors[0]->blackThreshold << ","
                    << R"("white_thresh":)" << sensors[0]->whiteThreshold
                    << "}";

            hal::screen_render::ScreenRender::instance().sendState(json.str());
            return true;
        }
        LIBSTP_LOG_WARN("Retrying...");
        hal::screen_render::ScreenRender::instance().sendState(R"({"type":"IR","state":"retrying"})");
    }
    LIBSTP_LOG_ERROR("Calibration failed after maximum attempts.");
    hal::screen_render::ScreenRender::instance().sendState(R"({"type":"IR","state":"tooManyAttempts"})");
    return false;
}


std::vector<float> IRSensorCalibration::collectValues(const std::vector<IRSensor *> &sensors, float durationSeconds) {
    std::vector<float> values;

    auto t_end = std::chrono::steady_clock::now() +
                 std::chrono::milliseconds(static_cast<long long>(durationSeconds * 1000));

    LIBSTP_LOG_INFO("Collecting Sensor Values");

    while (std::chrono::steady_clock::now() < t_end) {
        for (auto *sensor: sensors) {
            values.push_back(sensor->read());
            std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<long long>(10)));
        }
    }

    return values;
}
