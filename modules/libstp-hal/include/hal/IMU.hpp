//
// Created by tobias on 4/21/25.
//

#pragma once

#include <Eigen/Geometry>
#include "hal/IIMU.hpp"

namespace libstp::hal::imu
{
    class IMU : public IIMU
    {
#ifdef SAFETY_CHECKS_ENABLED
        static bool imuInstanceCreated;
#endif
    public:
        IMU();

        ~IMU() override;

        void read(float* accel, float* gyro, float* magneto) override;
        void getAngularVelocity(float* gyro) override;
        void calibrate() override;
        [[nodiscard]] Eigen::Quaternionf getOrientation() override;
        void getLinearAcceleration(float* linear_accel) override;
        void setLinearAccelCallback(std::function<void(float, float, float)> callback) override;

        // Wait for IMU to receive initial orientation data
        // Returns true if data received within timeout_ms, false otherwise
        bool waitForReady(int timeout_ms = 1000) override;
    };
}
