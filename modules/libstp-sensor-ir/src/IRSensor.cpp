//
// Created by eternalRose on 11/13/25.
//

#include "IRSensor.hpp"

#include <numeric>

#include "spdlog/spdlog.h"
using namespace libstp::sensors::ir;

IRSensor::IRSensor(const int &port, float calibrationFactor) : AnalogSensor(port),
                                                               whiteThreshold(0),
                                                               blackThreshold(0),
                                                               calibrationFactor(calibrationFactor * 0.5) {
}

void IRSensor::setCalibration(const int newBlackThreshold, const int newWhiteThreshold) {
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
    return 0; // Todo: implement proper probability calculation
}

float IRSensor::probabilityOfWhite() {
    return 1.0f - probabilityOfBlack();
}