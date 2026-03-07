//
// Created by tobias on 6/1/25.
//
#include "hal/Servo.hpp"

#include <spdlog/fmt/bundled/format.h>
#include <stdexcept>

#include "core/LcmReader.hpp"
#include "core/LcmWriter.hpp"

constexpr int MIN_PORT = 0;
constexpr int MAX_PORT = 4;
constexpr int MIN_POSITION = 0;
constexpr int MAX_POSITION = 2047;

libstp::hal::servo::Servo::Servo(const int port): port(port)
{
#ifdef SAFETY_CHECKS_ENABLED
    if (port < MIN_PORT || port > MAX_PORT)
    {
        throw std::out_of_range("Port number out of range");
    }

    registerServoPort(port);
#endif
}

libstp::hal::servo::Servo::~Servo()
{
#ifdef SAFETY_CHECKS_ENABLED
    unregisterServoPort(port);
#endif
}

void libstp::hal::servo::Servo::setPosition(const int position)
{
#ifdef SAFETY_CHECKS_ENABLED
    if (position < MIN_POSITION || position > MAX_POSITION)
    {
        throw std::out_of_range(fmt::format("Position must be between {} and {} degrees.", MIN_POSITION, MAX_POSITION));
    }
#endif
    
    storedPosition = position;
    platform::wombat::core::LcmDataWriter::instance().setServo(port,
                                     position);
}

int libstp::hal::servo::Servo::getPosition() const { return storedPosition; }

void libstp::hal::servo::Servo::enable() const
{
    platform::wombat::core::LcmDataWriter::instance().setServoMode(port, 2); // Enabled
    platform::wombat::core::LcmDataWriter::instance().setServo(port, storedPosition);
}

void libstp::hal::servo::Servo::disable() const
{
    platform::wombat::core::LcmDataWriter::instance().setServoMode(port, 1); // Disabled
}

void libstp::hal::servo::Servo::fullyDisableAll()
{
    for (uint8_t p = MIN_PORT; p < MAX_PORT; ++p)
        platform::wombat::core::LcmDataWriter::instance().setServoMode(p, 0); // FullyDisabled
}

/*
libstp::async::AsyncAlgorithm<int> libstp::servo::Servo::slowlySetPosition(const int targetPosition,
                                                                           const std::chrono::milliseconds duration,
                                                                           math::InterpolationFunction interpolationFunction)
{
    const int startPosition = getPosition();
    if (startPosition == targetPosition)
    {
        co_return 0;
    }
    if (duration.count() <= 0)
    {
        throw std::invalid_argument(
            "The target position is the same as the current position or the duration is invalid.");
    }
    
    const int clampedTarget = math::clampInt(targetPosition, MIN_POSITION, MAX_POSITION);
    
    using clock = std::chrono::steady_clock;
    const auto startTime = clock::now();
    const auto endTime = startTime + duration;
    
    const std::chrono::duration<double> total = endTime - startTime;
    while (true)
    {
        const auto now = clock::now();
        if (now >= endTime)
        {
            break;
        }
    
        std::chrono::duration<double> elapsed = now - startTime;
        double normalizedTime = elapsed.count() / total.count();
        normalizedTime = std::clamp(normalizedTime, 0.0, 1.0);
    
        const float interpolatedPosition = interpolationFunction(
            static_cast<float>(startPosition),
            static_cast<float>(clampedTarget),
            static_cast<float>(normalizedTime)
        );
    
        const int newPosition = static_cast<int>(std::round(interpolatedPosition));
        setPosition(newPosition);
    
        co_yield 1;
    }

    setPosition(clampedTarget);
}

void libstp::servo::Servo::shake(const int centerPosition,
                                 float amplitude,
                                 const float speedHz,
                                 const datatype::ConditionalFunction& conditional)
{
    const int clampedCenter = math::clampInt(centerPosition, MIN_POSITION, MAX_POSITION);
    const auto maxPossibleAmplitudePos = static_cast<float>(MAX_POSITION - clampedCenter);
    const auto maxPossibleAmplitudeNeg = static_cast<float>(clampedCenter - MIN_POSITION);
    const float maxPossibleAmplitude = std::min(maxPossibleAmplitudePos, maxPossibleAmplitudeNeg);

    float clampedAmplitude = amplitude;
    bool amplitudeAdjusted = false;

    if (clampedAmplitude > maxPossibleAmplitude)
    {
        clampedAmplitude = maxPossibleAmplitude;
        amplitudeAdjusted = true;
    }

    if (clampedAmplitude < 0.0f)
    {
        clampedAmplitude = 0.0f;
        amplitudeAdjusted = true;
    }


    if (amplitudeAdjusted)
    {
        LIBSTP_LOG_WARN(
            "Requested amplitude of {} exceeds the maximum possible amplitude of {} for centerPosition {}. Amplitude has been adjusted accordingly.",
            amplitude, clampedAmplitude, clampedCenter);
    }

    if (clampedAmplitude == 0.0f || speedHz <= 0.0f)
    {
        LIBSTP_LOG_WARN("Shake parameters are invalid. Servo will not be moved.");
        setPosition(clampedCenter);
        return;
    }

    using clock = std::chrono::steady_clock;
    const auto startTime = clock::now();

    constexpr auto twoPi = static_cast<float>(2.0 * M_PI);
    const float angularFrequency = twoPi * speedHz;

    while (conditional(false)->is_loop_running())
    {
        const auto now = clock::now();
        std::chrono::duration<float> elapsed = now - startTime;
        const float t = elapsed.count();

        const float offset = clampedAmplitude * std::sin(angularFrequency * t);

        const float newPositionF = static_cast<float>(clampedCenter) + offset;
        int newPosition = static_cast<int>(std::round(newPositionF));

        newPosition = math::clampInt(newPosition, MIN_POSITION, MAX_POSITION);

        setPosition(newPosition);

        utility::msleep();
    }

    setPosition(clampedCenter);
}
*/
