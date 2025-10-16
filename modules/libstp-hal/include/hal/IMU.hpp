//
// Created by tobias on 4/21/25.
//

#pragma once

#include <Eigen/Geometry>

namespace libstp::hal::imu
{
    class IMU
    {
#ifdef SAFETY_CHECKS_ENABLED
        static bool imuInstanceCreated;
#endif
    public:
        IMU();
        
        ~IMU();
        
        void read(float* accel, float* gyro, float* magneto);
        void calibrate();
        [[nodiscard]] Eigen::Quaternionf getOrientation();
    };
}
