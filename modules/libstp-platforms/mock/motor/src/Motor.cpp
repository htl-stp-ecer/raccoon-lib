#include "hal/Motor.hpp"
#include <stdexcept>
#include "core/MockPlatform.hpp"
#include "foundation/config.hpp"

constexpr int MIN_PORT = 0;
constexpr int MAX_PORT = 4;
constexpr int MIN_SPEED = -100;
constexpr int MAX_SPEED = 100;

libstp::hal::motor::Motor::Motor(const int port, const bool inverted,
                                 const foundation::MotorCalibration& calibration) : port_(port), inverted_(inverted),
    calibration_(calibration)
{
#ifdef SAFETY_CHECKS_ENABLED
    if (port < MIN_PORT || port > MAX_PORT)
    {
        throw std::out_of_range("Port number out of range");
    }

    registerMotorPort(port);
#endif
    platform::mock::core::MockPlatform::instance().init();
    LIBSTP_LOG_INFO(
        "Mock Motor ctor port={} inverted={}",
        port_,
        inverted_);
}

void libstp::hal::motor::Motor::setSpeed(const int percent)
{
#ifdef SAFETY_CHECKS_ENABLED
    if (percent < MIN_SPEED || percent > MAX_SPEED) throw std::out_of_range("speed -100 - 100");
#endif

    using namespace platform::mock::core;
    const bool zero = percent == 0;
    auto dir = MotorDir::Off;
    if (!zero)
        dir = (inverted_ ? percent < 0 : percent > 0) ? MotorDir::CW : MotorDir::CCW;
    const auto duty = static_cast<uint32_t>(std::abs(percent) * 4); // 0-400
    LIBSTP_LOG_INFO(
        "Mock Motor port={} setSpeed percent={} dir={} duty={} inverted={}",
        port_,
        percent,
        static_cast<int>(dir),
        duty,
        inverted_);
    setMotor(port_, dir, duty);
}

void libstp::hal::motor::Motor::setVelocity(const int velocity)
{
    const int directionVelocity = inverted_ ? -velocity : velocity;
    LIBSTP_LOG_INFO("Mock Motor port={} setVelocity={} inverted={}", port_, velocity, inverted_);
    // Closed-loop velocity command — uses the BEMF-units path so the sim can
    // interpret it via the motor calibration instead of as a duty %.
    platform::mock::core::MockPlatform::instance().setMotorVelocity(port_, directionVelocity);
}

void libstp::hal::motor::Motor::moveToPosition(const int velocity, const int goalPosition)
{
    LIBSTP_LOG_INFO("Mock Motor port={} moveToPosition vel={} goal={}", port_, velocity, goalPosition);
    // Mock: store goal, simulate immediate completion
}

void libstp::hal::motor::Motor::moveRelative(const int velocity, const int deltaPosition)
{
    LIBSTP_LOG_INFO("Mock Motor port={} moveRelative vel={} delta={}", port_, velocity, deltaPosition);
    // Mock: store delta, simulate immediate completion
}

int libstp::hal::motor::Motor::getPosition() const
{
    const int value = static_cast<int>(platform::mock::core::bemf(port_));
    LIBSTP_LOG_TRACE("Mock Motor port={} getPosition -> {}", port_, value);
    return value;
}

int libstp::hal::motor::Motor::getBemf() const
{
    const int value = static_cast<int>(platform::mock::core::bemf(port_));
    LIBSTP_LOG_TRACE("Mock Motor port={} getBemf -> {}", port_, value);
    return value;
}

bool libstp::hal::motor::Motor::isDone() const
{
    LIBSTP_LOG_TRACE("Mock Motor port={} isDone -> true", port_);
    return true;
}

void libstp::hal::motor::Motor::brake()
{
    platform::mock::core::setMotor(port_, platform::mock::core::MotorDir::Off, 0);
    LIBSTP_LOG_INFO("Mock Motor port={} brake", port_);
}

void libstp::hal::motor::Motor::off()
{
    platform::mock::core::setMotor(port_, platform::mock::core::MotorDir::Off, 0);
    LIBSTP_LOG_INFO("Mock Motor port={} off", port_);
}

void libstp::hal::motor::Motor::resetPositionCounter()
{
    LIBSTP_LOG_INFO("Mock Motor port={} resetPositionCounter", port_);
    // Mock: no-op, position comes from bemf() which is simulated
}

void libstp::hal::motor::Motor::disableAll()
{
    for (uint8_t p = MIN_PORT; p < MAX_PORT; ++p)
        platform::mock::core::setMotor(p, platform::mock::core::MotorDir::Off, 0);
    LIBSTP_LOG_INFO("Mock Motor disableAll executed");
}

void libstp::hal::motor::Motor::enableAll()
{
    LIBSTP_LOG_INFO("Mock Motor enableAll executed (no-op on mock)");
}

const libstp::foundation::MotorCalibration& libstp::hal::motor::Motor::getCalibration() const
{
    return calibration_;
}

void libstp::hal::motor::Motor::setCalibration(const foundation::MotorCalibration& calibration)
{
    calibration_ = calibration;
    LIBSTP_LOG_INFO("Mock Motor port={} setCalibration ticks_to_rad={}", port_, calibration_.ticks_to_rad);
}
