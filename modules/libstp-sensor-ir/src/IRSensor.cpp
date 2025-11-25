//
// Created by eternalRose on 11/13/25.
//

#include "../include/IRSensor.hpp"

#include <numeric>

#include "button_wraper.hpp"
#include "spdlog/spdlog.h"
using namespace libstp::sensors::ir;

IRSensor::IRSensor(const int &port, float calibrationFactor) : AnalogSensor(port),
                                                               whiteThreshold(0),
                                                               blackThreshold(0),
                                                               calibrationFactor(calibrationFactor * 0.5) {
}

bool IRSensor::calibrate(const std::vector<int> &whiteValues, const std::vector<int> &blackValues) {
    if (whiteValues.empty() || blackValues.empty()) {
        return false;
    }

    auto mean = [](const std::vector<int> &v) -> float {
        return std::accumulate(v.begin(), v.end(), 0.0f) / static_cast<float>(v.size());
    };

    auto stddev = [](const std::vector<int> &v, float mean) -> float {
        auto sum = 0.0f;
        for (const int val: v) {
            float diff = static_cast<float>(val) - mean;
            sum += diff * diff;
        }
        return std::sqrt(sum / static_cast<float>(v.size()));
    };

    whiteMean = mean(whiteValues);
    blackMean = mean(blackValues);

    if (whiteMean >= blackMean) {
        SPDLOG_ERROR("Calibration error: White values (mean: {}) should be lower than black values (mean: {})",
                     whiteMean, blackMean);
        SPDLOG_ERROR("This could indicate the sensors are positioned incorrectly or ambient light issues");
        return false;
    }
    whiteStdDev = std::max(1.0f, stddev(whiteValues, whiteMean));
    blackStdDev = std::max(1.0f, stddev(blackValues, blackMean));

    const float delta = blackMean - whiteMean;

    if (delta < 100.0f) {
        SPDLOG_ERROR("Insufficient contrast between white (mean: {}) and black (mean: {}) values",
                     whiteMean, blackMean);
        SPDLOG_ERROR("Delta = {}. Recommended minimum delta is 100", delta);
        return false;
    }

    if (whiteStdDev > 0.2f * delta || blackStdDev > 0.2f * delta) {
        SPDLOG_WARN("High variance in readings: white stddev = {}, black stddev = {}",
                    whiteStdDev, blackStdDev);
        SPDLOG_WARN("This might indicate uneven lighting or unstable sensor position");
    }

    whiteThreshold = static_cast<int>(whiteMean + calibrationFactor * delta);
    blackThreshold = static_cast<int>(blackMean - calibrationFactor * delta);

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
    const int val = read();
    const float p_black = gaussianProbability(val, blackMean, blackStdDev);
    const float p_white = gaussianProbability(val, whiteMean, whiteStdDev);
    const float sum = p_black + p_white;

    if (sum <= 1e-6f) return 0.5f;
    return p_black / sum;
}

float IRSensor::probabilityOfWhite() {
    return 1.0f - probabilityOfBlack();
}


void libstp::sensors::ir::calibrateLightSensors(const std::vector<IRSensor *> &lightSensors) {
    if (lightSensors.empty()) {
        SPDLOG_WARN("No light sensors to calibrate");
        return;
    }

    bool retry = true;
    while (retry) {
        constexpr int sampleCount = 10;
        SPDLOG_INFO("Calibrating light sensors");

        SPDLOG_INFO("Place the robot on a BLACK surface and press any button");
        button::ButtonWrapper().waitForButtonPress();

        std::vector<std::vector<int> > blackSamples(lightSensors.size());
        for (int sample = 0; sample < sampleCount; ++sample) {
            for (int i = 0; i < static_cast<int>(lightSensors.size()); ++i) {
                blackSamples[i].push_back(lightSensors[i]->read());
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<std::chrono::milliseconds::rep>(100)));
        }

        // --- WHITE SAMPLING ---
        SPDLOG_INFO("Place the robot on a WHITE surface and press any button");
        button::ButtonWrapper().waitForButtonPress();

        std::vector<std::vector<int> > whiteSamples(lightSensors.size());
        for (int sample = 0; sample < sampleCount; ++sample) {
            for (int i = 0; i < static_cast<int>(lightSensors.size()); ++i) {
                whiteSamples[i].push_back(lightSensors[i]->read());
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<std::chrono::milliseconds::rep>(100)));
        }

        // --- CALIBRATION ---
        retry = false;
        for (int i = 0; i < static_cast<int>(lightSensors.size()); ++i) {
            if (lightSensors[i]->calibrate(whiteSamples[i], blackSamples[i]))
                continue;

            retry = true;
            SPDLOG_ERROR("Calibration failed for sensor {}. Please retry.", i);
            break;
        }
    }
    SPDLOG_INFO("Calibration complete.");
}

bool libstp::sensors::ir::areOnBlack(IRSensor *leftSensor, IRSensor *rightSensor) {
    return leftSensor->isOnBlack() || rightSensor->isOnBlack();
}

bool libstp::sensors::ir::areOnWhite(IRSensor *leftSensor, IRSensor *rightSensor) {
    return leftSensor->isOnWhite() || rightSensor->isOnWhite();
}
