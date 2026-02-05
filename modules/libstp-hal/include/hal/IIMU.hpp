//
// IIMU interface for dependency injection and testing
//
#pragma once

#include <Eigen/Geometry>
#include <functional>

namespace libstp::hal::imu
{
    /**
     * @brief Interface for IMU abstraction
     *
     * This interface allows for mocking the IMU in unit tests
     * and enables dependency injection in odometry classes.
     */
    struct IIMU
    {
        virtual ~IIMU() = default;

        virtual void read(float* accel, float* gyro, float* magneto) = 0;
        virtual void calibrate() = 0;
        [[nodiscard]] virtual Eigen::Quaternionf getOrientation() = 0;

        /**
         * Get gravity-compensated linear acceleration in body frame
         * @param linear_accel Output array [x, y, z] for linear acceleration in m/s²
         */
        virtual void getLinearAcceleration(float* linear_accel) = 0;

        /**
         * Register a callback invoked from the sensor thread when new accel data arrives.
         * Pass nullptr to deregister. Default implementation is a no-op.
         */
        virtual void setLinearAccelCallback(std::function<void(float, float, float)> callback) { (void)callback; }

        // Wait for IMU to receive initial orientation data
        // Returns true if data received within timeout_ms, false otherwise
        virtual bool waitForReady(int timeout_ms = 1000) = 0;
    };
}
