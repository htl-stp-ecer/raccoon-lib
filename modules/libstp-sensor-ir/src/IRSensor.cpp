//
// Created by eternalRose on 11/13/25.
//

#include "IRSensor.hpp"

#include "Kmeans.hpp"
#include "foundation/logging.hpp"
#include "spdlog/spdlog.h"
using namespace libstp::sensors::ir;

IRSensor::IRSensor(const int &port, float calibrationFactor) : AnalogSensor(port),
                                                               whiteThreshold(0),
                                                               blackThreshold(0),
                                                               calibrationFactor(calibrationFactor * 0.5) {
}

void IRSensor::setCalibration(const float newBlackThreshold, const float newWhiteThreshold) {
    this->whiteThreshold=newWhiteThreshold;
    this->blackThreshold=newBlackThreshold;
}

bool IRSensor::isOnWhite() const {
    return read() < whiteThreshold;
}

bool IRSensor::isOnBlack() {
    return read() > blackThreshold;
}


void IRSensor::calibrate(const std::vector<float>& values) {
    if (values.empty()) {
        LIBSTP_LOG_WARN("No values provided");
        return;
    }

    std::vector<double> data(values.begin(), values.end());

    kmeans::KMeans km(10);
    kmeans::KMeansResult res = km.fit(data);

    whiteThreshold = static_cast<float>(res.centroid1);
    blackThreshold = static_cast<float>(res.centroid2);

    LIBSTP_LOG_DEBUG("Sucessful calibration: whiteThreshold (" + std::to_string(whiteThreshold) + ") blackThreshold (" + std::to_string(blackThreshold) + ")");
}

float IRSensor::probabilityOfBlack() const {
    float value = read();

    const float high = blackThreshold;
    const float low  = whiteThreshold;

    LIBSTP_LOG_INFO(std::to_string(low) + "; Black: " + std::to_string(high) + "; value: " + std::to_string(value));
    if (value <= low)  return 0.0f;
    if (value >= high) return 1.0f;
    return (value - low) / (high - low);
}

float IRSensor::probabilityOfWhite() {
    return 1.0f - probabilityOfBlack();
}