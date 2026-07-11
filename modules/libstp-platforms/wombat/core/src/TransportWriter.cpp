#include "core/TransportWriter.hpp"

#include "core/CommandTrace.hpp"

#include "foundation/logging.hpp"

#include <raccoon/Options.h>
#include <chrono>

using namespace platform::wombat::core;
namespace Channels = raccoon::Channels;

static const raccoon::PublishOptions reliableOpts{.reliable = true};

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
    const auto ch = Channels::motorPowerCommand(port);
    if (!transport_.publish(ch, publishedValue, reliableOpts))
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
    const auto ch = Channels::motorVelocityCommand(port);
    if (!transport_.publish(ch, msg, reliableOpts))
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
    const auto ch = Channels::motorPositionCommand(port);
    if (!transport_.publish(ch, msg, reliableOpts))
        LIBSTP_LOG_WARN("TransportWriter: motor position command DROPPED (publish failed) "
                        "port={} velocity={} goal={} channel={}", port, velocity, goalPosition, ch);
    CommandTrace::instance().record("motor_pos", ch, port, velocity, goalPosition, 0, 2,
                                    msg.timestamp);
}

void TransportWriter::setMotorRelative(uint8_t port, int32_t velocity, int32_t deltaPosition)
{
    raccoon::vector3f_t msg{};
    msg.timestamp = currentTimestampUsec();
    msg.x = static_cast<float>(velocity);
    msg.y = static_cast<float>(deltaPosition);
    msg.z = 0.0f;
    const auto ch = Channels::motorRelativeCommand(port);
    if (!transport_.publish(ch, msg, reliableOpts))
        LIBSTP_LOG_WARN("TransportWriter: motor relative command DROPPED (publish failed) "
                        "port={} velocity={} delta={} channel={}", port, velocity, deltaPosition, ch);
    CommandTrace::instance().record("motor_rel", ch, port, velocity, deltaPosition, 0, 2,
                                    msg.timestamp);
}

void TransportWriter::setServo(uint8_t port, float degrees)
{
    raccoon::scalar_f_t publishedValue{};
    publishedValue.timestamp = currentTimestampUsec();
    publishedValue.value = degrees;
    const auto ch = Channels::servoPositionCommand(port);
    transport_.publish(ch, publishedValue, reliableOpts);
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
    const auto ch = Channels::servoSmoothPositionCommand(port);
    transport_.publish(ch, msg, reliableOpts);
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
    const auto ch = Channels::servoModeCommand(port);
    transport_.publish(ch, msg, reliableOpts);
    CommandTrace::instance().record("servo_mode", ch, port, mode, 0, 0, 1, msg.timestamp);
}

void TransportWriter::setMotorPid(uint8_t port, float kp, float ki, float kd)
{
    raccoon::vector3f_t msg{};
    msg.timestamp = currentTimestampUsec();
    msg.x = kp;
    msg.y = ki;
    msg.z = kd;
    transport_.publish(Channels::motorPidCommand(port), msg, reliableOpts);
}

void TransportWriter::setChassisVelocity(float vx, float vy, float wz)
{
    raccoon::vector3f_t msg{};
    msg.timestamp = currentTimestampUsec();
    msg.x = vx;
    msg.y = vy;
    msg.z = wz;
    transport_.publish(Channels::CHASSIS_VELOCITY_CMD, msg);
    CommandTrace::instance().record("chassis_vel", Channels::CHASSIS_VELOCITY_CMD, -1, vx, vy, wz,
                                    3, msg.timestamp);
}

void TransportWriter::resetMotorPosition(uint8_t port)
{
    raccoon::scalar_i32_t msg{};
    msg.timestamp = currentTimestampUsec();
    msg.value = 1;
    const auto ch = Channels::motorPositionResetCommand(port);
    transport_.publish(ch, msg, reliableOpts);
    CommandTrace::instance().record("motor_pos_reset", ch, port, 1, 0, 0, 1, msg.timestamp);
}

void TransportWriter::setMotorMode(uint8_t port, int mode)
{
    raccoon::scalar_i32_t modeCmd{};
    modeCmd.timestamp = currentTimestampUsec();
    modeCmd.value = mode;
    const auto ch = Channels::motorModeCommand(port);
    if (!transport_.publish(ch, modeCmd, reliableOpts))
        LIBSTP_LOG_WARN("TransportWriter: motor mode command DROPPED (publish failed) "
                        "port={} mode={} channel={}", port, mode, ch);
    CommandTrace::instance().record("motor_mode", ch, port, mode, 0, 0, 1, modeCmd.timestamp);
}

void TransportWriter::setShutdown(bool enabled)
{
    raccoon::scalar_i32_t shutdownCmd{};
    shutdownCmd.timestamp = currentTimestampUsec();
    shutdownCmd.value = enabled ? 1 : 0;
    transport_.publish(Channels::SHUTDOWN_CMD, shutdownCmd, reliableOpts);
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
    transport_.publish(Channels::KINEMATICS_CONFIG_CMD, msg, reliableOpts);
}

void TransportWriter::resetOdometry()
{
    raccoon::scalar_i32_t msg{};
    msg.timestamp = currentTimestampUsec();
    msg.value = 1;
    transport_.publish(Channels::ODOM_RESET_CMD, msg, reliableOpts);
}

void TransportWriter::resetCalibOdometry()
{
    // Channel lives outside raccoon::Channels (calibration-board bridge).
    static constexpr auto kCalibOdomResetCmd = "raccoon/calib_board/cmd/odom/reset";
    raccoon::scalar_i32_t msg{};
    msg.timestamp = currentTimestampUsec();
    msg.value = 1;
    transport_.publish(kCalibOdomResetCmd, msg, reliableOpts);
}

void TransportWriter::sendHeartbeat()
{
    raccoon::scalar_i32_t msg{};
    msg.timestamp = currentTimestampUsec();
    msg.value = 1;
    transport_.publish(Channels::HEARTBEAT_CMD, msg);
}

TransportWriter::TransportWriter()
    : transport_(libstp::transport_core::SharedTransport::instance())
{
}
