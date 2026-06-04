#include "core/TransportReader.hpp"
#include <chrono>
#include <cmath>
#include <thread>
#include "foundation/logging.hpp"

using namespace platform::wombat::core;
namespace Channels = raccoon::Channels;

TransportReader::TransportReader()
    : transport_(libstp::transport_core::SharedTransport::instance())
{
    using raccoon::SubscribeOptions;

    // Retained subscribe options for channels that cache latest values
    static const SubscribeOptions retainedOpts{.requestRetained = true};

    // Subscribe to servo topics (ports 0-3) — retained
    for (int port = 0; port < 4; ++port) {
        transport_.subscribe<raccoon::scalar_i8_t>(
            Channels::servoMode(port),
            [this, port](const raccoon::scalar_i8_t& msg) {
                std::lock_guard<std::mutex> lock(cache_mutex_);
                servo_mode_cache_[port] = msg.dir;
            }, retainedOpts);

        transport_.subscribe<raccoon::scalar_f_t>(
            Channels::servoPosition(port),
            [this, port](const raccoon::scalar_f_t& msg) {
                std::lock_guard<std::mutex> lock(cache_mutex_);
                servo_value_cache_[port] = msg.value;
            }, retainedOpts);
    }

    // Subscribe to IMU topics — high-frequency, no retain needed
    transport_.subscribe<raccoon::vector3f_t>(
        Channels::GYRO,
        [this](const raccoon::vector3f_t& msg) {
            std::lock_guard<std::mutex> lock(cache_mutex_);
            gyro_cache_ = msg;
        });

    transport_.subscribe<raccoon::vector3f_t>(
        Channels::ACCELEROMETER,
        [this](const raccoon::vector3f_t& msg) {
            std::lock_guard<std::mutex> lock(cache_mutex_);
            accel_cache_ = msg;
        });

    transport_.subscribe<raccoon::vector3f_t>(
        Channels::LINEAR_ACCELERATION,
        [this](const raccoon::vector3f_t& msg) {
            std::lock_guard<std::mutex> lock(cache_mutex_);
            linear_accel_cache_ = msg;
        });

    transport_.subscribe<raccoon::vector3f_t>(
        Channels::ACCEL_VELOCITY,
        [this](const raccoon::vector3f_t& msg) {
            std::lock_guard<std::mutex> lock(cache_mutex_);
            accel_velocity_cache_ = msg;
        });

    transport_.subscribe<raccoon::vector3f_t>(
        Channels::MAGNETOMETER,
        [this](const raccoon::vector3f_t& msg) {
            std::lock_guard<std::mutex> lock(cache_mutex_);
            mag_cache_ = msg;
        });

    transport_.subscribe<raccoon::scalar_f_t>(
        Channels::HEADING,
        [this](const raccoon::scalar_f_t& msg) {
            std::lock_guard<std::mutex> lock(cache_mutex_);
            heading_cache_ = msg;
            imu_heading_received_ = true;
        }, retainedOpts);

    // Subscribe to BEMF topics (indices 0-3) — retained
    for (int idx = 0; idx < 4; ++idx) {
        transport_.subscribe<raccoon::scalar_i32_t>(
            Channels::backEmf(idx),
            [this, idx](const raccoon::scalar_i32_t& msg) {
                {
                    std::lock_guard<std::mutex> lock(cache_mutex_);
                    bemf_cache_[idx] = msg.value;
                }
                bemf_received_.store(true, std::memory_order_release);
            }, retainedOpts);
    }

    // Subscribe to motor position and done topics (ports 0-3) — retained
    for (int port = 0; port < 4; ++port) {
        transport_.subscribe<raccoon::scalar_i32_t>(
            Channels::motorPosition(port),
            [this, port](const raccoon::scalar_i32_t& msg) {
                std::lock_guard<std::mutex> lock(cache_mutex_);
                motor_position_cache_[port] = msg.value;
            }, retainedOpts);

        transport_.subscribe<raccoon::scalar_i32_t>(
            Channels::motorDone(port),
            [this, port](const raccoon::scalar_i32_t& msg) {
                std::lock_guard<std::mutex> lock(cache_mutex_);
                motor_done_cache_[port] = msg.value;
            }, retainedOpts);
    }

    // Subscribe to analog topics (ports 0-7)
    for (int port = 0; port < 8; ++port) {
        transport_.subscribe<raccoon::scalar_i32_t>(
            Channels::analog(port),
            [this, port](const raccoon::scalar_i32_t& msg) {
                std::lock_guard<std::mutex> lock(cache_mutex_);
                analog_cache_[port] = msg.value;
            });
    }

    // Subscribe to digital topics (ports 0-15)
    for (int port = 0; port < 16; ++port) {
        transport_.subscribe<raccoon::scalar_i32_t>(
            Channels::digital(port),
            [this, port](const raccoon::scalar_i32_t& msg) {
                std::lock_guard<std::mutex> lock(cache_mutex_);
                digital_cache_[port] = msg.value;
            });
    }

    // Subscribe to temperature topic
    transport_.subscribe<raccoon::scalar_f_t>(
        Channels::TEMPERATURE,
        [this](const raccoon::scalar_f_t& msg) {
            std::lock_guard<std::mutex> lock(cache_mutex_);
            temp_cache_ = msg;
        });

    // Subscribe to STM32 odometry topics — retained
    transport_.subscribe<raccoon::scalar_f_t>(
        Channels::ODOM_POS_X,
        [this](const raccoon::scalar_f_t& msg) {
            { std::lock_guard<std::mutex> lock(cache_mutex_);
              odom_cache_.pos_x = msg.value; }
            odom_cv_.notify_all();
        }, retainedOpts);
    transport_.subscribe<raccoon::scalar_f_t>(
        Channels::ODOM_POS_Y,
        [this](const raccoon::scalar_f_t& msg) {
            { std::lock_guard<std::mutex> lock(cache_mutex_);
              odom_cache_.pos_y = msg.value; }
            odom_cv_.notify_all();
        }, retainedOpts);
    transport_.subscribe<raccoon::scalar_f_t>(
        Channels::ODOM_HEADING,
        [this](const raccoon::scalar_f_t& msg) {
            { std::lock_guard<std::mutex> lock(cache_mutex_);
              odom_cache_.heading = msg.value; }
            odom_cv_.notify_all();
        }, retainedOpts);
    transport_.subscribe<raccoon::scalar_f_t>(
        Channels::ODOM_VX,
        [this](const raccoon::scalar_f_t& msg) {
            std::lock_guard<std::mutex> lock(cache_mutex_);
            odom_cache_.vx = msg.value;
        }, retainedOpts);
    transport_.subscribe<raccoon::scalar_f_t>(
        Channels::ODOM_VY,
        [this](const raccoon::scalar_f_t& msg) {
            std::lock_guard<std::mutex> lock(cache_mutex_);
            odom_cache_.vy = msg.value;
        }, retainedOpts);
    transport_.subscribe<raccoon::scalar_f_t>(
        Channels::ODOM_WZ,
        [this](const raccoon::scalar_f_t& msg) {
            std::lock_guard<std::mutex> lock(cache_mutex_);
            odom_cache_.wz = msg.value;
        }, retainedOpts);

    // Initialize default values
    gyro_cache_.x = 0.0f;
    gyro_cache_.y = 0.0f;
    gyro_cache_.z = 0.0f;

    accel_cache_.x = 0.0f;
    accel_cache_.y = 0.0f;
    accel_cache_.z = 0.0f;

    linear_accel_cache_.x = 0.0f;
    linear_accel_cache_.y = 0.0f;
    linear_accel_cache_.z = 0.0f;

    mag_cache_.x = 0.0f;
    mag_cache_.y = 0.0f;
    mag_cache_.z = 0.0f;

    temp_cache_.value = 0.0f;
    heading_cache_.value = 0.0f;

    // Initialize BEMF cache with zeros (hardware default)
    for (int idx = 0; idx < 4; ++idx) {
        bemf_cache_[idx] = 0;
        motor_position_cache_[idx] = 0;
        motor_done_cache_[idx] = 0;
    }

    // Initialize digital cache with zeros (hardware default)
    for (int port = 0; port < 16; ++port) {
        digital_cache_[port] = 0;
    }

}

TransportReader::~TransportReader() {
}

// Read methods - return cached values
raccoon::scalar_i8_t TransportReader::readServoMode(const int port) {
    std::lock_guard<std::mutex> lock(cache_mutex_);
    raccoon::scalar_i8_t result;
    auto it = servo_mode_cache_.find(port);
    result.dir = (it != servo_mode_cache_.end()) ? it->second : 1;  // Default: Disabled
    return result;
}

raccoon::scalar_f_t TransportReader::readServoValue(const int port) {
    std::lock_guard<std::mutex> lock(cache_mutex_);
    raccoon::scalar_f_t result;
    auto it = servo_value_cache_.find(port);
    result.value = (it != servo_value_cache_.end()) ? it->second : 90.0f;  // Default: middle position
    return result;
}

raccoon::vector3f_t TransportReader::readGyro() {
    std::lock_guard<std::mutex> lock(cache_mutex_);
    return gyro_cache_;
}

raccoon::vector3f_t TransportReader::readAccel() {
    std::lock_guard<std::mutex> lock(cache_mutex_);
    return accel_cache_;
}

raccoon::vector3f_t TransportReader::readLinearAccel() {
    std::lock_guard<std::mutex> lock(cache_mutex_);
    return linear_accel_cache_;
}

raccoon::vector3f_t TransportReader::readMag() {
    std::lock_guard<std::mutex> lock(cache_mutex_);
    return mag_cache_;
}

raccoon::scalar_f_t TransportReader::readHeading() {
    std::lock_guard<std::mutex> lock(cache_mutex_);
    return heading_cache_;
}

raccoon::scalar_i32_t TransportReader::readBemf(const int idx) {
    std::lock_guard<std::mutex> lock(cache_mutex_);
    raccoon::scalar_i32_t result;
    auto it = bemf_cache_.find(idx);
    result.value = (it != bemf_cache_.end()) ? it->second : 0;
    return result;
}

int32_t TransportReader::readMotorPosition(const int port) {
    std::lock_guard<std::mutex> lock(cache_mutex_);
    auto it = motor_position_cache_.find(port);
    return (it != motor_position_cache_.end()) ? it->second : 0;
}

bool TransportReader::readMotorDone(const int port) {
    std::lock_guard<std::mutex> lock(cache_mutex_);
    auto it = motor_done_cache_.find(port);
    return (it != motor_done_cache_.end()) ? it->second != 0 : false;
}

raccoon::scalar_i32_t TransportReader::readAnalog(const int port) {
    std::lock_guard<std::mutex> lock(cache_mutex_);
    raccoon::scalar_i32_t result;
    auto it = analog_cache_.find(port);
    result.value = (it != analog_cache_.end()) ? it->second : 0;
    return result;
}

raccoon::scalar_i32_t TransportReader::readDigital(const int port) {
    std::lock_guard<std::mutex> lock(cache_mutex_);
    raccoon::scalar_i32_t result;
    auto it = digital_cache_.find(port);
    result.value = (it != digital_cache_.end()) ? it->second : 0;
    return result;
}

raccoon::scalar_f_t TransportReader::readTemp() {
    std::lock_guard<std::mutex> lock(cache_mutex_);
    return temp_cache_;
}

raccoon::vector3f_t TransportReader::readAccelVelocity() {
    std::lock_guard<std::mutex> lock(cache_mutex_);
    raccoon::vector3f_t result;
    result.x = accel_velocity_cache_.x - accel_velocity_offset_.x;
    result.y = accel_velocity_cache_.y - accel_velocity_offset_.y;
    result.z = accel_velocity_cache_.z - accel_velocity_offset_.z;
    return result;
}

void TransportReader::resetAccelVelocity() {
    std::lock_guard<std::mutex> lock(cache_mutex_);
    accel_velocity_offset_ = accel_velocity_cache_;
}

TransportReader::OdometrySnapshot TransportReader::readOdometry() {
    std::lock_guard<std::mutex> lock(cache_mutex_);
    return odom_cache_;
}

void TransportReader::resetOdometry() {
    std::lock_guard<std::mutex> lock(cache_mutex_);
    odom_cache_ = OdometrySnapshot{};
}

bool TransportReader::waitForOdometryReset(int timeout_ms) {
    constexpr float kThreshold = 0.002f;
    auto deadline = std::chrono::steady_clock::now()
                  + std::chrono::milliseconds(timeout_ms);
    std::unique_lock<std::mutex> lock(cache_mutex_);
    return odom_cv_.wait_until(lock, deadline, [&] {
        return std::abs(odom_cache_.pos_x) < kThreshold
            && std::abs(odom_cache_.pos_y) < kThreshold
            && std::abs(odom_cache_.heading) < kThreshold;
    });
}

bool TransportReader::waitForImuReady(int timeout_ms) {
    const auto start = std::chrono::steady_clock::now();
    const auto timeout = std::chrono::milliseconds(timeout_ms);

    while (!imu_heading_received_) {
        const auto elapsed = std::chrono::steady_clock::now() - start;
        if (elapsed >= timeout) {
            LIBSTP_LOG_ERROR("[TransportReader] Timeout waiting for IMU heading data");
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    LIBSTP_LOG_TRACE("[TransportReader] IMU heading data received");
    return true;
}

bool TransportReader::waitForBemfData(int timeout_ms) {
    const auto start = std::chrono::steady_clock::now();
    const auto timeout = std::chrono::milliseconds(timeout_ms);

    while (!bemf_received_.load(std::memory_order_acquire)) {
        const auto elapsed = std::chrono::steady_clock::now() - start;
        if (elapsed >= timeout) {
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return true;
}
