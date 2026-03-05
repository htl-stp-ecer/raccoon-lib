#include "IRSensorCalibration.hpp"

#include <chrono>
#include <thread>
#include <hal/ScreenRender.hpp>

#include "CalibrationStore.hpp"
#include "button/button.hpp"
#include "foundation/logging.hpp"

using namespace libstp::sensors::ir;

bool IRSensorCalibration::calibrateSensors(const std::vector<IRSensor *> &sensors,
                                           float durationSeconds,
                                           bool usePre,
                                           const std::string& set_name) {
    hal::screen_render::ScreenRender::instance().setCurrentScreenSetting("calibrate_sensors");

    bool allGood = true;
    const size_t n = sensors.size();

    if (usePre) {
        for (size_t i = 0; i < n; ++i) {
            const std::string key = set_name + "_port" + std::to_string(sensors[i]->port);
            auto vals = calibration_store::CalibrationStore::instance().getReadings(
                calibration_store::CalibrationType::IR_SENSOR, key);
            sensors[i]->whiteThreshold = vals[0];
            sensors[i]->blackThreshold = vals[1];
        }
    } else {
        hal::screen_render::ScreenRender::instance().sendState(
            R"({"type":"IR","state":"readData"})"
        );

        auto perSensor = collectValues(sensors, durationSeconds);
        if (perSensor.empty()) {
            LIBSTP_LOG_WARN("No sensor values collected.");
            return false;
        }

        for (size_t i = 0; i < n; ++i) {
            if (!sensors[i]->calibrate(perSensor[i])) {
                allGood = false;
                break;
            }
        }
    }

    if (allGood) {
        LIBSTP_LOG_INFO("All sensors calibrated successfully.");
        std::ostringstream json;
        json << R"({"type":"IR","state":"confirm","sensors":[)";

        for (size_t i = 0; i < n; ++i) {
            if (i > 0) json << ",";
            json << R"({"black_thresh":)" << sensors[i]->blackThreshold
                 << R"(,"white_thresh":)" << sensors[i]->whiteThreshold << "}";

            const std::string key = set_name + "_port" + std::to_string(sensors[i]->port);
            calibration_store::CalibrationStore::instance().storeReading(
                sensors[i]->blackThreshold,
                sensors[i]->whiteThreshold,
                calibration_store::CalibrationType::IR_SENSOR,
                key);
        }

        json << "]}";
        hal::screen_render::ScreenRender::instance().sendState(json.str());
        return true;
    }
    LIBSTP_LOG_WARN("Retrying...");
    hal::screen_render::ScreenRender::instance().sendState(R"({"type":"IR","state":"retrying"})");
    return false;
}


std::vector<std::vector<float>> IRSensorCalibration::collectValues(const std::vector<IRSensor *> &sensors, float durationSeconds) {
    const size_t n = sensors.size();
    std::vector<std::vector<float>> perSensor(n);

    auto t_end = std::chrono::steady_clock::now() +
                 std::chrono::milliseconds(static_cast<long long>(durationSeconds * 1000));

    LIBSTP_LOG_INFO("Collecting Sensor Values");

    while (std::chrono::steady_clock::now() < t_end) {
        for (size_t i = 0; i < n; ++i) {
            perSensor[i].push_back(sensors[i]->read());
            std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<long long>(10)));
        }
    }

    return perSensor;
}
