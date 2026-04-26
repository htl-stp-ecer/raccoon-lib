#include "hal/Motor.hpp"
#include "foundation/config.hpp"

#include <stdexcept>
#include <chrono>

#include "core/LcmReader.hpp"
#include "core/LcmWriter.hpp"

constexpr int MIN_PORT = 0;
constexpr int MAX_PORT = 4;
constexpr int MIN_SPEED = -100;
constexpr int MAX_SPEED = 100;

libstp::hal::motor::Motor::Motor(const int port, const bool inverted, const foundation::MotorCalibration& calibration): port_(port), inverted_(inverted), calibration_(calibration)
{
    calibration_.validate();
#ifdef SAFETY_CHECKS_ENABLED
    if (port < MIN_PORT || port > MAX_PORT)
    {
        throw std::out_of_range("Port number out of range");
    }

    registerMotorPort(port);
#endif
    LIBSTP_LOG_TRACE(
        "Wombat Motor ctor port={} inverted={}",
        port_,
        inverted_);
}

void libstp::hal::motor::Motor::setSpeed(const int percent)
{
#ifdef SAFETY_CHECKS_ENABLED
    if (percent < MIN_SPEED || percent > MAX_SPEED) throw std::out_of_range("speed -100 - 100");
#endif

    const auto ts = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    LIBSTP_LOG_INFO(
        "[TIMING] setSpeed port={} percent={} epoch_us={}",
        port_, percent, ts);

    const int directionPercent = inverted_ ? -percent : percent;
    platform::wombat::core::LcmDataWriter::instance().setMotor(port_, directionPercent);
}

void libstp::hal::motor::Motor::setVelocity(const int velocity)
{
    const int directionVelocity = inverted_ ? -velocity : velocity;
    LIBSTP_LOG_TRACE("Wombat Motor port={} setVelocity={} inverted={}", port_, velocity, inverted_);
    platform::wombat::core::LcmDataWriter::instance().setMotorVelocity(port_, directionVelocity);
}

void libstp::hal::motor::Motor::moveToPosition(const int velocity, const int goalPosition)
{
    const int dirVel = inverted_ ? -velocity : velocity;
    const int dirPos = inverted_ ? -goalPosition : goalPosition;
    LIBSTP_LOG_TRACE("Wombat Motor port={} moveToPosition vel={} goal={}", port_, velocity, goalPosition);
    platform::wombat::core::LcmDataWriter::instance().setMotorPosition(port_, dirVel, dirPos);
}

void libstp::hal::motor::Motor::moveRelative(const int velocity, const int deltaPosition)
{
    const int dirVel = inverted_ ? -velocity : velocity;
    const int dirDelta = inverted_ ? -deltaPosition : deltaPosition;
    LIBSTP_LOG_TRACE("Wombat Motor port={} moveRelative vel={} delta={}", port_, velocity, deltaPosition);
    platform::wombat::core::LcmDataWriter::instance().setMotorRelative(port_, dirVel, dirDelta);
}

int libstp::hal::motor::Motor::getPosition() const
{
    const auto reading = platform::wombat::core::LcmReader::instance().readMotorPosition(port_);

    // Apply inversion to position reading to match command convention
    const int corrected = inverted_ ? -reading : reading;

    LIBSTP_LOG_TRACE("Wombat Motor port={} getPosition raw={} corrected={} inverted={}",
                 port_, reading, corrected, inverted_);
    return corrected;
}

int libstp::hal::motor::Motor::getBemf() const
{
    const auto reading = platform::wombat::core::LcmReader::instance().readBemf(port_);
    const int corrected = inverted_ ? -reading.value : reading.value;
    LIBSTP_LOG_TRACE("Wombat Motor port={} getBemf raw={} corrected={}", port_, reading.value, corrected);
    return corrected;
}

bool libstp::hal::motor::Motor::isDone() const
{
    return platform::wombat::core::LcmReader::instance().readMotorDone(port_);
}

void libstp::hal::motor::Motor::brake()
{
    platform::wombat::core::LcmDataWriter::instance().setMotorMode(port_, 1); // PASSIVE_BRAKE
    LIBSTP_LOG_DEBUG("Wombat Motor port={} brake (passive brake engaged)", port_);
}

void libstp::hal::motor::Motor::off()
{
    platform::wombat::core::LcmDataWriter::instance().setMotorMode(port_, 0); // OFF
    LIBSTP_LOG_DEBUG("Wombat Motor port={} off (motor disabled, free-spinning)", port_);
}

void libstp::hal::motor::Motor::resetPositionCounter()
{
    platform::wombat::core::LcmDataWriter::instance().resetMotorPosition(port_);
    LIBSTP_LOG_DEBUG("Wombat Motor port={} resetPositionCounter", port_);
}

void libstp::hal::motor::Motor::disableAll()
{
    // First, set the STM32 shutdown flag - this is the safest way to ensure
    // all motors and servos stop at the firmware level, even if we crash
    // before completing the individual motor stop commands.
    platform::wombat::core::LcmDataWriter::instance().setShutdown(true);

    // Also send individual brake commands for redundancy
    for (uint8_t p = MIN_PORT; p < MAX_PORT; ++p)
    {
        platform::wombat::core::LcmDataWriter::instance().setMotorMode(p, 1); // PASSIVE_BRAKE
    }
    LIBSTP_LOG_DEBUG("Wombat Motor disableAll executed (STM32 shutdown + stop latch engaged)");
}

void libstp::hal::motor::Motor::enableAll()
{
    // Clear the STM32 shutdown flag to allow motors and servos to operate
    platform::wombat::core::LcmDataWriter::instance().setShutdown(false);

    // Clear individual motor modes back to OFF
    for (uint8_t p = MIN_PORT; p < MAX_PORT; ++p)
    {
        platform::wombat::core::LcmDataWriter::instance().setMotorMode(p, 0); // OFF
    }
    LIBSTP_LOG_DEBUG("Wombat Motor enableAll executed (STM32 shutdown cleared)");
}

const libstp::foundation::MotorCalibration& libstp::hal::motor::Motor::getCalibration() const
{
    return calibration_;
}

void libstp::hal::motor::Motor::setCalibration(const foundation::MotorCalibration& calibration)
{
    calibration.validate();
    calibration_ = calibration;
    LIBSTP_LOG_DEBUG("Wombat Motor port={} setCalibration ticks_to_rad={}", port_, calibration_.ticks_to_rad);
}
