#include "foundation/logging.hpp"
#include "core/MockPlatform.hpp"
#include "hal/IMU.hpp"

#ifdef SAFETY_CHECKS_ENABLED
bool libstp::hal::imu::IMU::imuInstanceCreated = false;
#endif

libstp::hal::imu::IMU::IMU()
{
#ifdef SAFETY_CHECKS_ENABLED
    if (imuInstanceCreated)
    {
        LIBSTP_LOG_WARN("IMU is already initialized!");
        return;
    }
    imuInstanceCreated = true;
#endif

    platform::mock::core::MockPlatform::instance().init();
}

libstp::hal::imu::IMU::~IMU()
{
#ifdef SAFETY_CHECKS_ENABLED
    imuInstanceCreated = false;
#endif
}

void libstp::hal::imu::IMU::read(float* accel, float* gyro, float* magneto)
{
#ifdef SAFETY_CHECKS_ENABLED
    if (accel == nullptr || gyro == nullptr || magneto == nullptr)
    {
        throw std::runtime_error("IMU read failed! One or more output pointers are null.");
    }
#endif
    
    accel[0] = platform::mock::core::accelX();
    accel[1] = platform::mock::core::accelY();
    accel[2] = platform::mock::core::accelZ();
    gyro[0] = platform::mock::core::gyroX();
    gyro[1] = platform::mock::core::gyroY();
    gyro[2] = platform::mock::core::gyroZ();
    magneto[0] = platform::mock::core::magX();
    magneto[1] = platform::mock::core::magY();
    magneto[2] = platform::mock::core::magZ();
}

void libstp::hal::imu::IMU::calibrate()
{
    // Mock calibration - in a real implementation this would collect samples
    LIBSTP_LOG_INFO("[IMU Mock] Calibration complete (simulated).");
}

Eigen::Quaternionf libstp::hal::imu::IMU::getOrientation()
{
    return Eigen::Quaternionf::Identity();
}

bool libstp::hal::imu::IMU::waitForReady(int timeout_ms)
{
    // Mock platform always returns true immediately - no async data needed
    return true;
}

void libstp::hal::imu::IMU::getLinearAcceleration(float* linear_accel)
{
#ifdef SAFETY_CHECKS_ENABLED
    if (linear_accel == nullptr)
    {
        throw std::runtime_error("IMU getLinearAcceleration failed! Output pointer is null.");
    }
#endif
    // Mock returns zero linear acceleration (robot stationary)
    linear_accel[0] = 0.0f;
    linear_accel[1] = 0.0f;
    linear_accel[2] = 0.0f;
}
