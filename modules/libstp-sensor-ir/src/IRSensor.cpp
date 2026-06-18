
#include "IRSensor.hpp"
#include <numeric>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include "Kmeans.hpp"
#include "CalibrationStore.hpp"
#include "CalibrationType.hpp"
#include "foundation/logging.hpp"
#include "spdlog/spdlog.h"

using namespace libstp::sensors::ir;

IRSensor::IRSensor(const int &port)
    : AnalogSensor(port),
      whiteThreshold(0),
      blackThreshold(0){
    loadStoredCalibration();
}

void IRSensor::loadStoredCalibration() {
    const std::string set_name = "default_port" + std::to_string(port);
    const auto& store = libstp::calibration_store::CalibrationStore::instance();

    if (store.hasReadings(libstp::calibration_store::IR_SENSOR, set_name)) {
        // getReadings returns {white_tresh, black_tresh}.
        const auto readings = store.getReadings(libstp::calibration_store::IR_SENSOR, set_name);
        setCalibration(readings[1], readings[0]);
        LIBSTP_LOG_DEBUG(
            "IRSensor port " + std::to_string(port) +
            ": applied stored calibration (white=" + std::to_string(whiteThreshold) +
            ", black=" + std::to_string(blackThreshold) + ")."
        );
    } else {
        LIBSTP_LOG_DEBUG(
            "IRSensor port " + std::to_string(port) +
            ": no stored calibration found (set '" + set_name +
            "'); sensor stays uncalibrated until calibrated."
        );
    }
}

bool IRSensor::isCalibrated() const {
    return calibrated;
}

void IRSensor::ensureCalibrated(const char* query) const {
    if (!calibrated) {
        throw std::runtime_error(
            "IRSensor port " + std::to_string(port) +
            " is not calibrated; cannot answer " + query +
            ". Calibrate the sensor or store calibration first."
        );
    }
}

void IRSensor::setCalibration(const float newBlackThreshold,
                              const float newWhiteThreshold) {
    blackThreshold = newBlackThreshold;
    whiteThreshold = newWhiteThreshold;
    calibrated = true;
}

bool IRSensor::isOnWhite() const {
    ensureCalibrated("isOnWhite");
    return read() < whiteThreshold;
}

bool IRSensor::isOnBlack() {
    ensureCalibrated("isOnBlack");
    return read() > blackThreshold;
}

float IRSensor::mean(const std::vector<float>& v) {
    if (v.empty()) return 0.0f;
    const float sum = std::accumulate(v.begin(), v.end(), 0.0f);
    return sum / v.size();
}

float IRSensor::variance(const std::vector<float>& v, float m) {
    if (v.empty()) return 0.0f;
    float sum = 0.0f;
    for (float x : v) {
        const float d = x - m;
        sum += d * d;
    }
    return sum / v.size();
}

float IRSensor::stddev(const std::vector<float>& v) {
    const float m = mean(v);
    return std::sqrt(variance(v, m));
}

bool IRSensor::calibrate(const std::vector<float>& values) {
    if (values.empty()) {
        LIBSTP_LOG_WARN("Calibration aborted: no values provided.");
        return false;
    }

    const float minVal = *std::ranges::min_element(values);
    const float maxVal = *std::ranges::max_element(values);
    const float range  = maxVal - minVal;

    if (range < 500.0f) {
        LIBSTP_LOG_WARN(
            "Calibration aborted: insufficient sensor range (" +
            std::to_string(range) + ")."
        );
        return false;
    }

    const std::vector<double> data(values.begin(), values.end());

    const kmeans::KMeans km(10);
    auto [centroid1, centroid2] = km.fit(data);

    float c1 = static_cast<float>(centroid1);
    float c2 = static_cast<float>(centroid2);

    const float white = std::min(c1, c2);
    const float black = std::max(c1, c2);
    const float separation = black - white;

    constexpr float MIN_ABSOLUTE_DELTA = 700.0f;
    constexpr float MIN_RANGE_FRACTION = 0.25f;

    if (separation < MIN_ABSOLUTE_DELTA ||
        separation < range * MIN_RANGE_FRACTION) {

        LIBSTP_LOG_WARN(
            "Calibration aborted: centroids too close (white=" +
            std::to_string(white) +
            ", black=" +
            std::to_string(black) +
            ", separation=" +
            std::to_string(separation) +
            ", range=" +
            std::to_string(range) + ")."
        );
        return false;
    }

    this->whiteThreshold = white;
    this->blackThreshold = black;
    this->calibrated = true;

    LIBSTP_LOG_INFO(
        "Calibration successful (port " + std::to_string(port) + "): whiteThreshold=" +
        std::to_string(whiteThreshold) +
        ", blackThreshold=" +
        std::to_string(blackThreshold) +
        ", separation=" +
        std::to_string(separation) +
        ", range=" +
        std::to_string(range)
    );

    return true;
}

float IRSensor::probabilityOfBlack() const {
    ensureCalibrated("probabilityOfBlack");
    const float value = read();

    if (value <= whiteThreshold) return 0.0f;
    if (value >= blackThreshold) return 1.0f;

    return (value - whiteThreshold) /
           (blackThreshold - whiteThreshold);
}

float IRSensor::probabilityOfWhite() const {
    ensureCalibrated("probabilityOfWhite");
    return 1.0f - probabilityOfBlack();
}
