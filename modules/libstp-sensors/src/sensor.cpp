//
// Created by tobias on 12/26/24.
//
#include "sensors/sensor.hpp"
#include "foundation/config.hpp"

#include <numeric>

libstp::sensor::DistanceSensor::DistanceSensor(const int& port): AnalogSensor(port)
{
}

double libstp::sensor::DistanceSensor::getDistance()
{
    // ToDo: Use lookup table for better readings
    const int analog_measurement = read();
    const double volt = analog_measurement / 1251.215;
    return (38.6 - 7 * volt) / (0.2 + volt);
}

libstp::sensor::LightSensor::LightSensor(const int& port, const float calibrationFactor): AnalogSensor(port),
    whiteThreshold(0),
    blackThreshold(0),
    calibrationFactor(calibrationFactor * 0.5f)
{
}

bool libstp::sensor::LightSensor::calibrate(const std::vector<int>& whiteValues, const std::vector<int>& blackValues)
{
    if (whiteValues.empty() || blackValues.empty())
        return false;

    auto mean = [](const std::vector<int>& v) -> float
    {
        return std::accumulate(v.begin(), v.end(), 0.0f) / static_cast<float>(v.size());
    };

    auto stddev = [](const std::vector<int>& v, float mean) -> float
    {
        float sum = 0.0f;
        for (const int val : v)
        {
            float diff = static_cast<float>(val) - mean;
            sum += diff * diff;
        }
        return std::sqrt(sum / static_cast<float>(v.size()));
    };

    whiteMean = mean(whiteValues);
    blackMean = mean(blackValues);

    // Check if white mean is less than black mean (expected relationship)
    if (whiteMean >= blackMean)
    {
        SPDLOG_ERROR("Calibration error: White values (mean: {}) should be lower than black values (mean: {})",
                     whiteMean, blackMean);
        SPDLOG_ERROR("This could indicate the sensors are positioned incorrectly or ambient light issues");
        return false;
    }

    whiteStdDev = std::max(1.0f, stddev(whiteValues, whiteMean));
    blackStdDev = std::max(1.0f, stddev(blackValues, blackMean));

    const float delta = blackMean - whiteMean;

    // Check for sufficient contrast
    if (delta < 100.0f)
    {
        SPDLOG_ERROR("Insufficient contrast between white (mean: {}) and black (mean: {}) values",
                     whiteMean, blackMean);
        SPDLOG_ERROR("Delta = {}. Recommended minimum delta is 100", delta);
        return false;
    }

    // Check for excessive sensor variance
    if (whiteStdDev > 0.2f * delta || blackStdDev > 0.2f * delta)
    {
        SPDLOG_WARN("High variance in readings: white stddev = {}, black stddev = {}",
                    whiteStdDev, blackStdDev);
        SPDLOG_WARN("This might indicate uneven lighting or unstable sensor position");
    }

    whiteThreshold = static_cast<int>(whiteMean + calibrationFactor * delta);
    blackThreshold = static_cast<int>(blackMean - calibrationFactor * delta);

    SPDLOG_INFO("Calibration successful: white mean = {}, black mean = {}", whiteMean, blackMean);
    SPDLOG_INFO("Thresholds set: white = {}, black = {}", whiteThreshold, blackThreshold);

    return true;
}

bool libstp::sensor::LightSensor::isOnWhite()
{
    return read() < whiteThreshold;
}

bool libstp::sensor::LightSensor::isOnBlack()
{
    return read() > blackThreshold;
}

void libstp::sensor::calibrateLightSensors(const std::vector<LightSensor*>& lightSensors)
{
    if (lightSensors.empty())
    {
        SPDLOG_WARN("No light sensors to calibrate");
        return;
    }

    bool retry = true;
    while (retry)
    {
        constexpr int sampleCount = 10;
        SPDLOG_INFO("Calibrating light sensors");

        // --- BLACK SAMPLING ---
        SPDLOG_INFO("Place the robot on a BLACK surface and press any button");
        waitForButtonClick();

        std::vector<std::vector<int>> blackSamples(lightSensors.size());
        for (int sample = 0; sample < sampleCount; ++sample)
        {
            for (int i = 0; i < static_cast<int>(lightSensors.size()); ++i)
            {
                blackSamples[i].push_back(lightSensors[i]->read());
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        // --- WHITE SAMPLING ---
        SPDLOG_INFO("Place the robot on a WHITE surface and press any button");
        waitForButtonClick();

        std::vector<std::vector<int>> whiteSamples(lightSensors.size());
        for (int sample = 0; sample < sampleCount; ++sample)
        {
            for (int i = 0; i < static_cast<int>(lightSensors.size()); ++i)
            {
                whiteSamples[i].push_back(lightSensors[i]->read());
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        // --- CALIBRATION ---
        retry = false;
        for (int i = 0; i < static_cast<int>(lightSensors.size()); ++i)
        {
            if (lightSensors[i]->calibrate(whiteSamples[i], blackSamples[i]))
                continue;

            retry = true;
            SPDLOG_ERROR("Calibration failed for sensor {}. Please retry.", i);
            break;
        }
    }

    SPDLOG_INFO("Calibration complete.");
}


void libstp::sensor::waitForButtonClick()
{
    throw std::runtime_error("Button click functionality is not implemented yet.");
    /*while (any_button())
    {
        // Make sure button isn't clicked
        std::this_thread::sleep_for(std::chrono::milliseconds());
    }
    while (!any_button())
    {
        // Wait for button to be clicked
        std::this_thread::sleep_for(std::chrono::milliseconds());
    }*/
}

bool libstp::sensor::isButtonClicked()
{
    throw std::runtime_error("Button click functionality is not implemented yet.");
    /*
    return any_button() != 0;
*/
}

bool libstp::sensor::areOnBlack(LightSensor* leftSensor, LightSensor* rightSensor)
{
    return leftSensor->isOnBlack() || rightSensor->isOnBlack();
}

bool libstp::sensor::areOnWhite(LightSensor* leftSensor, LightSensor* rightSensor)
{
    return leftSensor->isOnWhite() || rightSensor->isOnWhite();
}
