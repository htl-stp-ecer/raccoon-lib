//
// Created by tobias on 4/21/25.
//

#pragma once

#include "hal/IIMU.hpp"

namespace libstp::hal::imu
{
    /**
     * Concrete IMU wrapper backed by the selected platform bundle.
     *
     * Higher-level code should depend on `IIMU` when injection is useful and
     * instantiate `IMU` when it wants the configured hardware implementation.
     */
    class IMU : public IIMU
    {
#ifdef SAFETY_CHECKS_ENABLED
        static bool imuInstanceCreated;
#endif
    public:
        /// Construct the platform IMU implementation.
        IMU();

        ~IMU() override;

        /// Read accel, gyro, and magnetometer data into caller-owned buffers.
        void read(float* accel, float* gyro, float* magneto) override;
        /// Read angular velocity in radians per second into a caller-owned buffer.
        void getAngularVelocity(float* gyro) override;
        /// Run whatever calibration behavior the active platform exposes.
        void calibrate() override;
        /// Return the current heading in radians using libstp sign conventions.
        [[nodiscard]] float getHeading() override;
        /// Read gravity-compensated acceleration into a caller-owned buffer.
        void getLinearAcceleration(float* linear_accel) override;
        /// Read firmware-integrated velocity when the platform provides it.
        void getIntegratedVelocity(float* vel) override;
        /// Reset the platform's integrated-velocity accumulator if supported.
        void resetIntegratedVelocity() override;

        /// Wait for the platform to report that initial orientation data is ready.
        bool waitForReady(int timeout_ms = 1000) override;
    };
}
