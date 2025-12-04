//
// Created by eternalRose on 11/13/25.
//

#include "IRSensor.hpp"

#include <iostream>
#include <numeric>

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

float IRSensor::probabilityOfBlack() {
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