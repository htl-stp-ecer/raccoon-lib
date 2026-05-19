#include <gtest/gtest.h>
#include <lcm/lcm-cpp.hpp>
#include <thread>
#include <chrono>
#include <mutex>
#include <unordered_map>

#include "exlcm/scalar_i32_t.hpp"
#include "exlcm/scalar_i8_t.hpp"
#include "exlcm/scalar_f_t.hpp"
#include "exlcm/vector3f_t.hpp"
#include "exlcm/quaternion_t.hpp"

class LcmContractTest : public ::testing::Test {
protected:
    void SetUp() override {
        ASSERT_TRUE(lcm.good());

        for (int port = 0; port < 4; ++port) {
            lcm.subscribe("libstp/motor/" + std::to_string(port) + "/value",
                          &LcmContractTest::handleMotorValue, this);
            lcm.subscribe("libstp/motor/" + std::to_string(port) + "/direction",
                          &LcmContractTest::handleMotorDir, this);
        }

        for (int port = 0; port < 4; ++port) {
            lcm.subscribe("libstp/servo/" + std::to_string(port) + "/position",
                          &LcmContractTest::handleServoValue, this);
            lcm.subscribe("libstp/servo/" + std::to_string(port) + "/mode",
                          &LcmContractTest::handleServoMode, this);
        }

        lcm.subscribe("libstp/gyro/value", &LcmContractTest::handleGyro, this);
        lcm.subscribe("libstp/accel/value", &LcmContractTest::handleAccel, this);
        lcm.subscribe("libstp/mag/value", &LcmContractTest::handleMag, this);
        lcm.subscribe("libstp/imu/quaternion", &LcmContractTest::handleQuaternion, this);

        for (int idx = 0; idx < 4; ++idx) {
            lcm.subscribe("libstp/bemf/" + std::to_string(idx) + "/value",
                          &LcmContractTest::handleBemf, this);
        }

        for (int port = 0; port < 8; ++port) {
            lcm.subscribe("libstp/analog/" + std::to_string(port) + "/value",
                          &LcmContractTest::handleAnalog, this);
        }

        for (int port = 0; port < 16; ++port) {
            lcm.subscribe("libstp/digital/" + std::to_string(port) + "/value",
                          &LcmContractTest::handleDigital, this);
        }

        lcm.subscribe("libstp/temp/value", &LcmContractTest::handleTemp, this);

        running = true;
        listener_thread = std::thread([this]() {
            while (running) {
                lcm.handleTimeout(50);
            }
        });
    }

    void TearDown() override {
        running = false;
        if (listener_thread.joinable())
            listener_thread.join();
    }

    void handleMotorValue(const lcm::ReceiveBuffer*, const std::string& channel, const exlcm::scalar_i32_t* msg) {
        int port = std::stoi(channel.substr(channel.find_last_of('/') - 1, 1));
        std::lock_guard<std::mutex> lock(mutex_);
        motor_values_[port] = msg->value;
    }

    void handleMotorDir(const lcm::ReceiveBuffer*, const std::string& channel, const exlcm::scalar_i8_t* msg) {
        int port = std::stoi(channel.substr(channel.find_last_of('/') - 1, 1));
        std::lock_guard<std::mutex> lock(mutex_);
        motor_dirs_[port] = msg->dir;
    }

    void handleServoValue(const lcm::ReceiveBuffer*, const std::string& channel, const exlcm::scalar_i32_t* msg) {
        int port = std::stoi(channel.substr(channel.find_last_of('/') - 1, 1));
        std::lock_guard<std::mutex> lock(mutex_);
        servo_values_[port] = msg->value;
    }

    void handleServoMode(const lcm::ReceiveBuffer*, const std::string& channel, const exlcm::scalar_i8_t* msg) {
        int port = std::stoi(channel.substr(channel.find_last_of('/') - 1, 1));
        std::lock_guard<std::mutex> lock(mutex_);
        servo_modes_[port] = msg->dir;
    }

    void handleGyro(const lcm::ReceiveBuffer*, const std::string&, const exlcm::vector3f_t* msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        gyro_ = *msg;
    }

    void handleAccel(const lcm::ReceiveBuffer*, const std::string&, const exlcm::vector3f_t* msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        accel_ = *msg;
    }

    void handleMag(const lcm::ReceiveBuffer*, const std::string&, const exlcm::vector3f_t* msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        mag_ = *msg;
    }

    void handleQuaternion(const lcm::ReceiveBuffer*, const std::string&, const exlcm::quaternion_t* msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        quat_ = *msg;
    }

    void handleBemf(const lcm::ReceiveBuffer*, const std::string& channel, const exlcm::scalar_i32_t* msg) {
        int idx = std::stoi(channel.substr(channel.find_last_of('/') - 1, 1));
        std::lock_guard<std::mutex> lock(mutex_);
        bemf_[idx] = msg->value;
    }

    void handleAnalog(const lcm::ReceiveBuffer*, const std::string& channel, const exlcm::scalar_i32_t* msg) {
        int port = std::stoi(channel.substr(channel.find_last_of('/') - 1, 1));
        std::lock_guard<std::mutex> lock(mutex_);
        analog_[port] = msg->value;
    }

    void handleDigital(const lcm::ReceiveBuffer*, const std::string& channel, const exlcm::scalar_i32_t* msg) {
        int port = std::stoi(channel.substr(channel.find_last_of('/') - 1, 1));
        std::lock_guard<std::mutex> lock(mutex_);
        digital_[port] = msg->value;
    }

    void handleTemp(const lcm::ReceiveBuffer*, const std::string&, const exlcm::scalar_f_t* msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        temp_ = *msg;
    }

    lcm::LCM lcm;
    std::thread listener_thread;
    bool running{false};
    std::mutex mutex_;

    std::unordered_map<int,int> motor_values_;
    std::unordered_map<int,int> motor_dirs_;
    std::unordered_map<int,int> servo_values_;
    std::unordered_map<int,int> servo_modes_;
    std::unordered_map<int,int> bemf_;
    std::unordered_map<int,int> analog_;
    std::unordered_map<int,int> digital_;

    exlcm::vector3f_t gyro_;
    exlcm::vector3f_t accel_;
    exlcm::vector3f_t mag_;
    exlcm::quaternion_t quat_;
    exlcm::scalar_f_t temp_;
};

TEST_F(LcmContractTest, MotorValueRoundtrip) {
    exlcm::scalar_i32_t msg{}; msg.value = 42;
    lcm.publish("libstp/motor/1/value", &msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    std::lock_guard<std::mutex> lock(mutex_);
    EXPECT_EQ(motor_values_[1], 42);
}

TEST_F(LcmContractTest, MotorDirRoundtrip) {
    exlcm::scalar_i8_t msg{}; msg.dir = 1;
    lcm.publish("libstp/motor/2/direction", &msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    std::lock_guard<std::mutex> lock(mutex_);
    EXPECT_EQ(motor_dirs_[2], 1);
}

TEST_F(LcmContractTest, ServoValueRoundtrip) {
    exlcm::scalar_i32_t msg{}; msg.value = 512;
    lcm.publish("libstp/servo/0/position", &msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    std::lock_guard<std::mutex> lock(mutex_);
    EXPECT_EQ(servo_values_[0], 512);
}

TEST_F(LcmContractTest, ServoModeRoundtrip) {
    exlcm::scalar_i8_t msg{}; msg.dir = 2;
    lcm.publish("libstp/servo/1/mode", &msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    std::lock_guard<std::mutex> lock(mutex_);
    EXPECT_EQ(servo_modes_[1], 2);
}

TEST_F(LcmContractTest, GyroRoundtrip) {
    exlcm::vector3f_t msg{}; msg.x=1; msg.y=2; msg.z=3;
    lcm.publish("libstp/gyro/value", &msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    std::lock_guard<std::mutex> lock(mutex_);
    EXPECT_EQ(gyro_.x, 1);
    EXPECT_EQ(gyro_.y, 2);
    EXPECT_EQ(gyro_.z, 3);
}

TEST_F(LcmContractTest, AccelRoundtrip) {
    exlcm::vector3f_t msg{}; msg.x=4; msg.y=5; msg.z=6;
    lcm.publish("libstp/accel/value", &msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    std::lock_guard<std::mutex> lock(mutex_);
    EXPECT_EQ(accel_.x, 4);
    EXPECT_EQ(accel_.y, 5);
    EXPECT_EQ(accel_.z, 6);
}

TEST_F(LcmContractTest, MagRoundtrip) {
    exlcm::vector3f_t msg{}; msg.x=7; msg.y=8; msg.z=9;
    lcm.publish("libstp/mag/value", &msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    std::lock_guard<std::mutex> lock(mutex_);
    EXPECT_EQ(mag_.x, 7);
    EXPECT_EQ(mag_.y, 8);
    EXPECT_EQ(mag_.z, 9);
}

TEST_F(LcmContractTest, QuaternionRoundtrip) {
    exlcm::quaternion_t msg{}; msg.w=1; msg.x=0; msg.y=0; msg.z=0;
    lcm.publish("libstp/imu/quaternion", &msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    std::lock_guard<std::mutex> lock(mutex_);
    EXPECT_EQ(quat_.w, 1);
    EXPECT_EQ(quat_.x, 0);
    EXPECT_EQ(quat_.y, 0);
    EXPECT_EQ(quat_.z, 0);
}

TEST_F(LcmContractTest, BemfRoundtrip) {
    exlcm::scalar_i32_t msg{}; msg.value=55;
    lcm.publish("libstp/bemf/2/value", &msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    std::lock_guard<std::mutex> lock(mutex_);
    EXPECT_EQ(bemf_[2], 55);
}

TEST_F(LcmContractTest, AnalogRoundtrip) {
    exlcm::scalar_i32_t msg{}; msg.value=1234;
    lcm.publish("libstp/analog/3/value", &msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    std::lock_guard<std::mutex> lock(mutex_);
    EXPECT_EQ(analog_[3], 1234);
}

TEST_F(LcmContractTest, DigitalRoundtrip) {
    exlcm::scalar_i32_t msg{}; msg.value=1;
    lcm.publish("libstp/digital/7/value", &msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    std::lock_guard<std::mutex> lock(mutex_);
    EXPECT_EQ(digital_[7], 1);
}

TEST_F(LcmContractTest, TempRoundtrip) {
    exlcm::scalar_f_t msg{}; msg.value=36.6f;
    lcm.publish("libstp/temp/value", &msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    std::lock_guard<std::mutex> lock(mutex_);
    EXPECT_FLOAT_EQ(temp_.value, 36.6f);
}
