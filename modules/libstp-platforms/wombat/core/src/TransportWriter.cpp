#include "core/TransportWriter.hpp"

#include "core/CommandTrace.hpp"

#include "foundation/logging.hpp"

#include <chrono>

using namespace platform::wombat::core;

static int64_t currentTimestampUsec()
{
    return std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
}

void TransportWriter::setMotor(uint8_t port, int valueData)
{
    raccoon::scalar_i32_t publishedValue{};
    publishedValue.timestamp = currentTimestampUsec();
    publishedValue.value = valueData;
    const auto ch = raccoon::channels::hardware::motor::cmd::power(port);
    if (transport_.publish(ch, publishedValue) != raccoon::PublishResult::Ok)
        LIBSTP_LOG_WARN("TransportWriter: motor power command DROPPED (publish failed) "
                        "port={} value={} channel={}", port, valueData, ch);
    CommandTrace::instance().record("motor_power", ch, port, valueData, 0, 0, 1,
                                    publishedValue.timestamp);
}

void TransportWriter::setMotorVelocity(uint8_t port, int32_t velocity)
{
    raccoon::scalar_i32_t msg{};
    msg.timestamp = currentTimestampUsec();
    msg.value = velocity;
    const auto ch = raccoon::channels::hardware::motor::cmd::velocity(port);
    if (transport_.publish(ch, msg) != raccoon::PublishResult::Ok)
        LIBSTP_LOG_WARN("TransportWriter: motor velocity command DROPPED (publish failed) "
                        "port={} velocity={} channel={}", port, velocity, ch);
    CommandTrace::instance().record("motor_vel", ch, port, velocity, 0, 0, 1, msg.timestamp);
}

void TransportWriter::setMotorPosition(uint8_t port, int32_t velocity, int32_t goalPosition)
{
    raccoon::vector3f_t msg{};
    msg.timestamp = currentTimestampUsec();
    msg.x = static_cast<float>(velocity);
    msg.y = static_cast<float>(goalPosition);
    msg.z = 0.0f;
    const auto ch = raccoon::channels::hardware::motor::cmd::position(port);
    if (transport_.publish(ch, msg) != raccoon::PublishResult::Ok)
        LIBSTP_LOG_WARN("TransportWriter: motor position command DROPPED (publish failed) "
                        "port={} velocity={} goal={} channel={}", port, velocity, goalPosition, ch);
    CommandTrace::instance().record("motor_pos", ch, port, velocity, goalPosition, 0, 2,
                                    msg.timestamp);
}

void TransportWriter::setServo(uint8_t port, float degrees)
{
    raccoon::scalar_f_t publishedValue{};
    publishedValue.timestamp = currentTimestampUsec();
    publishedValue.value = degrees;
    const auto ch = raccoon::channels::hardware::servo::cmd::position(port);
    transport_.publish(ch, publishedValue);
    CommandTrace::instance().record("servo_pos", ch, port, degrees, 0, 0, 1,
                                    publishedValue.timestamp);
}

void TransportWriter::setSmoothServo(const uint8_t port, const float targetDeg,
                                     const float speedDegPerSec, const int easing)
{
    raccoon::vector3f_t msg{};
    msg.timestamp = currentTimestampUsec();
    msg.x = targetDeg;
    msg.y = speedDegPerSec;
    msg.z = static_cast<float>(easing);
    const auto ch = raccoon::channels::hardware::servo::cmd::smooth(port);
    transport_.publish(ch, msg);
    CommandTrace::instance().record("servo_smooth", ch, port, targetDeg, speedDegPerSec, easing,
                                    3, msg.timestamp);
}

void TransportWriter::setServoMode(uint8_t port, uint8_t mode)
{
    raccoon::scalar_i8_t msg{};
    msg.timestamp = currentTimestampUsec();
    msg.dir = static_cast<int8_t>(mode);
    // Publish on the dedicated COMMAND channel so the reader doesn't
    // ping-pong with its own state publish on the formerly-shared
    // `servoMode` channel. State lives on `servoMode`; commands here.
    const auto ch = raccoon::channels::hardware::servo::cmd::mode(port);
    transport_.publish(ch, msg);
    CommandTrace::instance().record("servo_mode", ch, port, mode, 0, 0, 1, msg.timestamp);
}

void TransportWriter::setMotorPid(uint8_t port, float kp, float ki, float kd)
{
    raccoon::vector3f_t msg{};
    msg.timestamp = currentTimestampUsec();
    msg.x = kp;
    msg.y = ki;
    msg.z = kd;
    transport_.publish(raccoon::channels::hardware::motor::cmd::pid(port), msg);
}

void TransportWriter::setChassisVelocity(float vx, float vy, float wz)
{
    raccoon::vector3f_t msg{};
    msg.timestamp = currentTimestampUsec();
    msg.x = vx;
    msg.y = vy;
    msg.z = wz;
    // The stream self-heals on the next frame. A stop is a command of its own and a reliable
    // channel: it is often sent only once (Drive::hardStop), and a lost stop leaves the robot driving.
    if (vx == 0.0f && vy == 0.0f && wz == 0.0f)
    {
        raccoon::scalar_i32_t stop{};
        stop.timestamp = msg.timestamp;
        stop.value = 1;
        transport_.publish(raccoon::channels::hardware::chassis::cmd::STOP, stop);
    }
    else
        transport_.publish(raccoon::channels::hardware::chassis::cmd::VELOCITY, msg);
    CommandTrace::instance().record("chassis_vel", raccoon::channels::hardware::chassis::cmd::VELOCITY, -1, vx, vy, wz,
                                    3, msg.timestamp);
}

void TransportWriter::resetMotorPosition(uint8_t port)
{
    raccoon::scalar_i32_t msg{};
    msg.timestamp = currentTimestampUsec();
    msg.value = 1;
    const auto ch = raccoon::channels::hardware::motor::cmd::reset_position(port);
    transport_.publish(ch, msg);
    CommandTrace::instance().record("motor_pos_reset", ch, port, 1, 0, 0, 1, msg.timestamp);
}

void TransportWriter::setMotorMode(uint8_t port, int mode)
{
    raccoon::scalar_i32_t modeCmd{};
    modeCmd.timestamp = currentTimestampUsec();
    modeCmd.value = mode;
    const auto ch = raccoon::channels::hardware::motor::cmd::mode(port);
    if (transport_.publish(ch, modeCmd) != raccoon::PublishResult::Ok)
        LIBSTP_LOG_WARN("TransportWriter: motor mode command DROPPED (publish failed) "
                        "port={} mode={} channel={}", port, mode, ch);
    CommandTrace::instance().record("motor_mode", ch, port, mode, 0, 0, 1, modeCmd.timestamp);
}

void TransportWriter::setShutdown(bool enabled)
{
    raccoon::scalar_i32_t shutdownCmd{};
    shutdownCmd.timestamp = currentTimestampUsec();
    shutdownCmd.value = enabled ? 1 : 0;
    transport_.publish(raccoon::channels::hardware::latch::cmd::ENGAGED, shutdownCmd);
}

void TransportWriter::sendKinematicsConfig(const std::array<std::array<float, 4>, 3>& inv_matrix,
                                           const std::array<float, 4>& ticks_to_rad,
                                           const std::array<std::array<float, 3>, 4>& fwd_matrix,
                                           const std::array<float, 4>& bemf_offset)
{
    raccoon::kinematics_config_t msg{};
    msg.timestamp = currentTimestampUsec();
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 4; ++c)
            msg.inv_matrix[r * 4 + c] = inv_matrix[r][c];
    for (int i = 0; i < 4; ++i)
        msg.ticks_to_rad[i] = ticks_to_rad[i];
    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 3; ++c)
            msg.fwd_matrix[r * 3 + c] = fwd_matrix[r][c];
    for (int i = 0; i < 4; ++i)
        msg.bemf_offset[i] = bemf_offset[i];
    transport_.publish(raccoon::channels::hardware::kinematics::cmd::CONFIG, msg);
}

void TransportWriter::resetOdometry()
{
    raccoon::scalar_i32_t msg{};
    msg.timestamp = currentTimestampUsec();
    msg.value = 1;
    transport_.publish(raccoon::channels::hardware::odometry::cmd::RESET, msg);
}

void TransportWriter::resetCalibOdometry()
{
    // Command of the calibration-board bridge; the channel is in the table.
    static constexpr auto kCalibOdomResetCmd = raccoon::channels::hardware::calib_board::odom::cmd::RESET;
    raccoon::scalar_i32_t msg{};
    msg.timestamp = currentTimestampUsec();
    msg.value = 1;
    transport_.publish(kCalibOdomResetCmd, msg);
}

void TransportWriter::sendHeartbeat()
{
    raccoon::scalar_i32_t msg{};
    msg.timestamp = currentTimestampUsec();
    msg.value = 1;
    transport_.publish(raccoon::channels::hardware::watchdog::cmd::HEARTBEAT, msg);
}

TransportWriter::TransportWriter()
    : transport_(libstp::transport_core::SharedTransport::instance())
{
}
