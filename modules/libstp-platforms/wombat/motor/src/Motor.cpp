//
// Created by tobias on 6/1/25.
//

#include "hal/Motor.hpp"
#include "foundation/config.hpp"

#include <stdexcept>

#include "core/LcmReader.hpp"
#include "core/LcmWriter.hpp"

constexpr int MIN_PORT = 0;
constexpr int MAX_PORT = 4;
constexpr int MIN_SPEED = -100;
constexpr int MAX_SPEED = 100;

libstp::hal::motor::Motor::Motor(const int port, const bool inverted, const foundation::MotorCalibration& calibration): port(port), inverted(inverted), calibration_(calibration)
{
#ifdef SAFETY_CHECKS_ENABLED
    if (port < MIN_PORT || port > MAX_PORT)
    {
        throw std::out_of_range("Port number out of range");
    }

    registerMotorPort(port);
#endif
    SPDLOG_INFO(
        "Wombat Motor ctor port={} inverted={} pid(kp={}, ki={}, kd={}) ff(kS={}, kV={}, kA={})",
        this->port,
        this->inverted,
        calibration_.pid.kp,
        calibration_.pid.ki,
        calibration_.pid.kd,
        calibration_.ff.kS,
        calibration_.ff.kV,
        calibration_.ff.kA);
}

void libstp::hal::motor::Motor::setSpeed(const int percent) const
{
#ifdef SAFETY_CHECKS_ENABLED
    if (percent < MIN_SPEED || percent > MAX_SPEED) throw std::out_of_range("speed -100 - 100");
#endif

    int scaled = percent * 4; // -400 to 400
    if (inverted)
    {
        scaled = -scaled;
    }
    SPDLOG_TRACE(
        "Wombat Motor port={} setSpeed percent={} scaled={} inverted={}",
        port,
        percent,
        scaled,
        inverted);
    platform::wombat::core::LcmDataWriter::instance().setMotor(port, scaled);
}

int libstp::hal::motor::Motor::getPosition() const
{
    const auto reading = platform::wombat::core::LcmReader::instance().readBemf(port).value;

    // Apply inversion to position reading to match command convention
    const int corrected = inverted ? reading : -reading;

    SPDLOG_TRACE("Wombat Motor port={} getPosition raw={} corrected={} inverted={}",
                 port, reading, corrected, inverted);
    return corrected;
}

void libstp::hal::motor::Motor::brake() const
{
    platform::wombat::core::LcmDataWriter::instance().setMotor(port, 0);
    SPDLOG_INFO("Wombat Motor port={} brake", port);
}

void libstp::hal::motor::Motor::disableAll()
{
    for (uint8_t p = MIN_PORT; p < MAX_PORT; ++p)
        platform::wombat::core::LcmDataWriter::instance().setMotor(p, 0);
    SPDLOG_INFO("Wombat Motor disableAll executed");
}

const libstp::foundation::MotorCalibration& libstp::hal::motor::Motor::getCalibration() const
{
    return calibration_;
}
