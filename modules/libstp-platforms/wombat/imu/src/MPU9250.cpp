//
// Created by tobias on 4/21/25.
//
#include "foundation/logging.hpp"
#include <cmath>
#include <limits>

#include "core/LcmReader.hpp"
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
    // ToDo: Check if buffers are large enough
#endif

    const exlcm::vector3f_t accelValue = platform::wombat::core::LcmReader::instance().readAccel();
    const exlcm::vector3f_t gyroValue = platform::wombat::core::LcmReader::instance().readGyro();
    const exlcm::vector3f_t magValue = platform::wombat::core::LcmReader::instance().readMag();

    accel[0] = accelValue.x;
    accel[1] = accelValue.y;
    accel[2] = accelValue.z;
    gyro[0] = gyroValue.x;
    gyro[1] = gyroValue.y;
    gyro[2] = gyroValue.z;
    magneto[0] = magValue.x;
    magneto[1] = magValue.y;
    magneto[2] = magValue.z;
}

Eigen::Quaternionf libstp::hal::imu::IMU::getOrientation()
{
    const auto orientationMsg = platform::wombat::core::LcmReader::instance().readOrientation();
    Eigen::Quaternionf orientation(orientationMsg.w, orientationMsg.x, orientationMsg.y, orientationMsg.z);

    const float norm = orientation.norm();
    if (!std::isfinite(norm) || norm <= std::numeric_limits<float>::epsilon())
    {
        return Eigen::Quaternionf::Identity();
    }

    orientation.normalize();
    return orientation;
}

bool libstp::hal::imu::IMU::waitForReady(int timeout_ms)
{
    return platform::wombat::core::LcmReader::instance().waitForImuReady(timeout_ms);
}

void libstp::hal::imu::IMU::calibrate()
{
    /*namespace libstp::sensor
{
    using namespace Eigen;

    class GyroSensor
    {
    public:
        void calibrate(std::shared_ptr<MatrixX3d> calibrationMatrix);
        std::shared_ptr<Vector3d> getValue() const;
        std::shared_ptr<Vector3d> getVariance();
        std::shared_ptr<Vector3d> getBias();
        std::shared_ptr<Vector3d> applyCalibration(const std::shared_ptr<Vector3d>& sample) const;

    private:
        std::shared_ptr<Vector3d> offset = std::make_shared<Vector3d>(Vector3d::Zero());
        std::shared_ptr<Vector3d> variance = std::make_shared<Vector3d>(Vector3d::Ones());
    };

    class AccelSensor
    {
    public:
        void calibrate(std::shared_ptr<MatrixX3d> calibrationMatrix);
        std::shared_ptr<Vector3d> getValue() const;
        std::shared_ptr<Vector3d> getVariance();
        std::shared_ptr<Vector3d> getBias();
        std::shared_ptr<Vector3d> getGravity();
        std::shared_ptr<Vector3d> applyCalibration(const std::shared_ptr<Vector3d>& sample) const;

    private:
        std::shared_ptr<Vector3d> offset = std::make_shared<Vector3d>(Vector3d::Zero());
        std::shared_ptr<Vector3d> variance = std::make_shared<Vector3d>(Vector3d::Ones());
        std::shared_ptr<Vector3d> gravity = std::make_shared<Vector3d>(0.0, 0.0, 9.81);
    };

    class MagnetoSensor
    {
    public:
        void calibrate(std::shared_ptr<MatrixX3d> calibrationMatrix);
        void setHardIronOffset(const std::shared_ptr<Vector3d>& newOffset);
        void setSoftIronMatrix(const std::shared_ptr<Matrix3d>& matrix);
        std::shared_ptr<Vector3d> getValue() const;
        std::shared_ptr<Vector3d> getVariance();
        std::shared_ptr<Vector3d> applyCalibration(const std::shared_ptr<Vector3d>& sample) const;

    private:
        std::shared_ptr<Vector3d> offset = std::make_shared<Vector3d>(Vector3d::Zero());
        std::shared_ptr<Vector3d> variance = std::make_shared<Vector3d>(Vector3d::Ones());
        std::shared_ptr<Matrix3d> softIronMatrix = std::make_shared<Matrix3d>(Matrix3d::Identity());
    };

    class IMU
    {
    public:
        IMU() = default;
        async::AsyncAlgorithm<int> calibrate(int sampleCount = 100);
        std::tuple<std::shared_ptr<Vector3d>, std::shared_ptr<Vector3d>, std::shared_ptr<Vector3d>> getReading() const;

        GyroSensor gyro;
        AccelSensor accel;
        MagnetoSensor magneto;
    };

    
void libstp::sensor::GyroSensor::calibrate(std::shared_ptr<MatrixX3d> calibrationMatrix)
{
    const int rows = calibrationMatrix->rows();
    const int cols = calibrationMatrix->cols();

    auto median = std::make_shared<Vector3d>();
    for (int i = 0; i < cols; ++i)
    {
        std::vector<double> colData(rows);
        for (int j = 0; j < rows; ++j)
        {
            colData[j] = (*calibrationMatrix)(j, i);
        }
        std::ranges::sort(colData);
        (*median)[i] = (rows % 2 == 0) ? (colData[rows / 2 - 1] + colData[rows / 2]) / 2.0 : colData[rows / 2];
    }

    offset = median;

    auto applied = std::make_shared<MatrixX3d>(rows, 3);
    for (int i = 0; i < rows; ++i)
    {
        applied->row(i) = calibrationMatrix->row(i) - offset->transpose();
    }

    variance = std::make_shared<Vector3d>(applied->array().square().colwise().mean());

    LIBSTP_LOG_INFO("[IMU] Calibrated gyro sensor with bias: ({}, {}, {}), variance: ({}, {}, {})", 
                (*offset)[0], (*offset)[1], (*offset)[2], 
                (*variance)[0], (*variance)[1], (*variance)[2]);
}

std::shared_ptr<Eigen::Vector3d> libstp::sensor::GyroSensor::getValue() const
{
    const auto rawGyro = std::make_shared<Vector3d>(
        gyro_x() * DEG_TO_RAD,
        gyro_y() * DEG_TO_RAD,
        gyro_z() * DEG_TO_RAD
        );
    return applyCalibration(rawGyro);
}

std::shared_ptr<Eigen::Vector3d> libstp::sensor::GyroSensor::getVariance()
{
    return variance;
}

std::shared_ptr<Eigen::Vector3d> libstp::sensor::GyroSensor::getBias()
{
    return offset;
}

std::shared_ptr<Eigen::Vector3d> libstp::sensor::GyroSensor::applyCalibration(const std::shared_ptr<Vector3d>& sample) const
{
    return std::make_shared<Vector3d>(*sample - *offset);
}

void libstp::sensor::AccelSensor::calibrate(std::shared_ptr<MatrixX3d> calibrationMatrix)
{
    const int rows = calibrationMatrix->rows();
    const int cols = calibrationMatrix->cols();

    auto median = std::make_shared<Vector3d>();
    for (int i = 0; i < cols; ++i)
    {
        std::vector<double> colData(rows);
        for (int j = 0; j < rows; ++j)
        {
            colData[j] = (*calibrationMatrix)(j, i);
        }
        std::ranges::sort(colData);
        (*median)[i] = (rows % 2 == 0) ? (colData[rows / 2 - 1] + colData[rows / 2]) / 2.0 : colData[rows / 2];
    }

    offset = median;

    int gravity_axis;
    offset->maxCoeff(&gravity_axis);
    const double gravity_sign = ((*offset)[gravity_axis] > 0) ? 1.0 : -1.0;
    (*offset)[gravity_axis] -= 9.81 * gravity_sign;

    gravity = std::make_shared<Vector3d>(Vector3d::Zero());
    (*gravity)[gravity_axis] = 9.81 * gravity_sign;

    LIBSTP_LOG_INFO("[IMU] Detected gravity axis: {} ({})", gravity_axis, gravity_sign > 0 ? "+" : "-");

    auto diffMatrix = std::make_shared<MatrixX3d>(rows, 3);
    for (int i = 0; i < rows; ++i) {
        diffMatrix->row(i) = calibrationMatrix->row(i) - offset->transpose();
    }
    variance = std::make_shared<Vector3d>(diffMatrix->array().square().colwise().mean());
    
    LIBSTP_LOG_INFO("[IMU] Calibrated accel sensor with bias: ({}, {}, {}), variance: ({}, {}, {})", 
                (*offset)[0], (*offset)[1], (*offset)[2], 
                (*variance)[0], (*variance)[1], (*variance)[2]);
}

std::shared_ptr<Eigen::Vector3d> libstp::sensor::AccelSensor::getValue() const
{
    return applyCalibration(std::make_shared<Vector3d>(Vector3d(accel_x(), accel_y(), accel_z())));
}

std::shared_ptr<Eigen::Vector3d> libstp::sensor::AccelSensor::getVariance()
{
    return variance;
}

std::shared_ptr<Eigen::Vector3d> libstp::sensor::AccelSensor::getBias()
{
    return offset;
}

std::shared_ptr<Eigen::Vector3d> libstp::sensor::AccelSensor::getGravity()
{
    return gravity;
}

std::shared_ptr<Eigen::Vector3d> libstp::sensor::AccelSensor::applyCalibration(const std::shared_ptr<Vector3d>& sample) const
{
    return std::make_shared<Vector3d>(*sample - *offset);
}

void libstp::sensor::MagnetoSensor::calibrate(std::shared_ptr<MatrixX3d> calibrationMatrix)
{
    variance = std::make_shared<Vector3d>(calibrationMatrix->colwise().squaredNorm());
    LIBSTP_LOG_INFO("[IMU] Calibrated magneto sensor with variance: ({}, {}, {})", 
                (*variance)[0], (*variance)[1], (*variance)[2]);
}

void libstp::sensor::MagnetoSensor::setHardIronOffset(const std::shared_ptr<Vector3d>& offset)
{
    this->offset = offset;
}

void libstp::sensor::MagnetoSensor::setSoftIronMatrix(const std::shared_ptr<Matrix3d>& matrix)
{
    softIronMatrix = matrix;
}

std::shared_ptr<Eigen::Vector3d> libstp::sensor::MagnetoSensor::getValue() const
{
    const auto raw = std::make_shared<Vector3d>(Vector3d(magneto_x(), magneto_y(), magneto_z()));
    return applyCalibration(raw);
}

std::shared_ptr<Eigen::Vector3d> libstp::sensor::MagnetoSensor::getVariance()
{
    return variance;
}

std::shared_ptr<Eigen::Vector3d> libstp::sensor::MagnetoSensor::applyCalibration(const std::shared_ptr<Vector3d>& sample) const
{
    const auto corrected = std::make_shared<Vector3d>(*sample - *offset);
    return std::make_shared<Vector3d>(*softIronMatrix * *corrected);
}

libstp::async::AsyncAlgorithm<int> libstp::sensor::IMU::calibrate(const int sampleCount)
{
    auto samples_gyro = std::make_shared<MatrixX3d>(sampleCount, 3);
    auto samples_accel = std::make_shared<MatrixX3d>(sampleCount, 3);
    auto samples_mag = std::make_shared<MatrixX3d>(sampleCount, 3);

    LIBSTP_LOG_INFO("[IMU] Calibrating IMU... Please keep the device still.");
    for (int i = 0; i < sampleCount; ++i)
    {
        samples_gyro->row(i) = *gyro.getValue();
        samples_accel->row(i) = *accel.getValue();
        samples_mag->row(i) = *magneto.getValue();
        co_yield 1;
    }

    gyro.calibrate(samples_gyro);
    accel.calibrate(samples_accel);
    magneto.calibrate(samples_mag);

    LIBSTP_LOG_INFO("[IMU] Calibration complete.");
}

std::tuple<std::shared_ptr<Eigen::Vector3d>, std::shared_ptr<Eigen::Vector3d>, std::shared_ptr<Eigen::Vector3d>> libstp::sensor::IMU::getReading() const
{
    return std::make_tuple(
        gyro.getValue(),
        accel.getValue(),
        magneto.getValue()
    );
}
}*/
}
