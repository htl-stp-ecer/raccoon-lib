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

    std::vector<float> values;
    bool allGood = true;

    if (usePre) {
        values = calibration_store::CalibrationStore::instance().getReadings(calibration_store::CalibrationType::IR_SENSOR, set_name);
        for (auto *sensor: sensors) {
            sensor->whiteThreshold = values[0];
            sensor->blackThreshold = values[1];
        }
    } else {
        hal::screen_render::ScreenRender::instance().sendState(
    R"({"type":"IR","state":"readData"})"
        );
        values = collectValues(sensors, durationSeconds);
        if (values.empty()) {
            LIBSTP_LOG_WARN("No sensor values collected.");
            return false;
        }

        for (auto *sensor: sensors) {
            if (!sensor->calibrate(values)) {
                allGood = false;
                break;
            }
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
        calibration_store::CalibrationStore::instance().storeReading(sensors[0]->blackThreshold,
                                                                     sensors[0]->whiteThreshold,
                                                                     calibration_store::CalibrationType::IR_SENSOR,
                                                                     set_name);
        hal::screen_render::ScreenRender::instance().sendState(json.str());
        return true;
    }
    LIBSTP_LOG_WARN("Retrying...");
    hal::screen_render::ScreenRender::instance().sendState(R"({"type":"IR","state":"retrying"})");
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
