//
// IIMU interface for dependency injection and testing
//
#pragma once

#include <Eigen/Geometry>
#include <algorithm>
#include <cctype>
#include <stdexcept>
#include <string>

namespace libstp::hal::imu
{
    enum class TurnAxisMode
    {
        WorldZ,
        BodyX,
        BodyY,
        BodyZ,
    };

    inline const char* toString(const TurnAxisMode mode)
    {
        switch (mode)
        {
            case TurnAxisMode::WorldZ: return "world_z";
            case TurnAxisMode::BodyX: return "body_x";
            case TurnAxisMode::BodyY: return "body_y";
            case TurnAxisMode::BodyZ: return "body_z";
        }
        return "world_z";
    }

    inline std::string normalizeTurnAxisMode(std::string mode)
    {
        std::transform(mode.begin(), mode.end(), mode.begin(), [](unsigned char c) {
            if (c == '-' || c == ' ') return static_cast<char>('_');
            return static_cast<char>(std::tolower(c));
        });
        return mode;
    }

    struct TurnAxisConfig
    {
        TurnAxisMode mode{TurnAxisMode::WorldZ};
        float sign{1.0f}; // +1 or -1
    };

    inline TurnAxisConfig parseTurnAxisConfig(const std::string& mode)
    {
        // Check for leading '-' to negate the axis
        float sign = 1.0f;
        std::string rest = mode;
        if (!rest.empty() && rest[0] == '-') {
            sign = -1.0f;
            rest = rest.substr(1);
        }

        const std::string normalized = normalizeTurnAxisMode(rest);

        if (normalized.empty() || normalized == "world_z" || normalized == "yaw" || normalized == "heading")
            return {TurnAxisMode::WorldZ, sign};
        if (normalized == "body_x" || normalized == "x")
            return {TurnAxisMode::BodyX, sign};
        if (normalized == "body_y" || normalized == "y")
            return {TurnAxisMode::BodyY, sign};
        if (normalized == "body_z" || normalized == "z")
            return {TurnAxisMode::BodyZ, sign};

        throw std::invalid_argument(
            "Invalid turn axis mode '" + mode +
            "'. Expected one of: [-]world_z, [-]body_x, [-]body_y, [-]body_z");
    }

    inline TurnAxisMode parseTurnAxisMode(const std::string& mode)
    {
        return parseTurnAxisConfig(mode).mode;
    }

    inline TurnAxisMode parseBodyAxisMode(const std::string& mode)
    {
        const TurnAxisMode parsed = parseTurnAxisMode(mode);
        if (parsed == TurnAxisMode::WorldZ)
        {
            throw std::invalid_argument(
                "Invalid body axis '" + mode + "'. Expected one of: body_x, body_y, body_z");
        }
        return parsed;
    }

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
         * Get angular velocity (gyroscope) in body frame
         * @param gyro Output array [x, y, z] for angular velocity in rad/s
         */
        virtual void getAngularVelocity(float* gyro)
        {
            float accel[3], g[3], magneto[3];
            read(accel, g, magneto);
            gyro[0] = g[0]; gyro[1] = g[1]; gyro[2] = g[2];
        }

        void setYawRateAxisMode(const TurnAxisMode mode)
        {
            yaw_rate_axis_mode_ = mode;
            yaw_rate_sign_ = 1.0f;
        }

        void setYawRateAxisMode(const std::string& mode)
        {
            const auto cfg = parseTurnAxisConfig(mode);
            yaw_rate_axis_mode_ = cfg.mode;
            yaw_rate_sign_ = cfg.sign;
        }

        [[nodiscard]] TurnAxisMode getYawRateAxisMode() const
        {
            return yaw_rate_axis_mode_;
        }

        [[nodiscard]] std::string getYawRateAxisModeName() const
        {
            return toString(yaw_rate_axis_mode_);
        }

        /**
         * Get the angular rate around the configured turn axis.
         * Default is world z-axis ("world_z"), which matches the original yaw-rate behavior.
         * Body-axis modes ("body_x", "body_y", "body_z") return the raw body-frame gyro
         * component and are useful when the robot rotates while tilted/upright.
         * @return Angular rate in rad/s along the configured turn axis
         */
        float getYawRate()
        {
            float g[3];
            getAngularVelocity(g);
            const Eigen::Vector3f gyro_body(g[0], g[1], g[2]);

            float rate = 0.0f;
            switch (yaw_rate_axis_mode_)
            {
                case TurnAxisMode::BodyX:
                    rate = gyro_body.x(); break;
                case TurnAxisMode::BodyY:
                    rate = gyro_body.y(); break;
                case TurnAxisMode::BodyZ:
                    rate = gyro_body.z(); break;
                case TurnAxisMode::WorldZ:
                default:
                {
                    const Eigen::Vector3f gyro_world = getOrientation() * gyro_body;
                    rate = gyro_world.z(); break;
                }
            }
            return yaw_rate_sign_ * rate;
        }

        /**
         * Get gravity-compensated linear acceleration in body frame
         * @param linear_accel Output array [x, y, z] for linear acceleration in m/s²
         */
        virtual void getLinearAcceleration(float* linear_accel) = 0;

        /**
         * Get firmware-integrated velocity from accelerometer in body frame (m/s)
         * @param vel Output array [x, y, z] for velocity
         */
        virtual void getIntegratedVelocity(float* vel) { vel[0] = vel[1] = vel[2] = 0; }

        /**
         * Reset the firmware-integrated velocity offset (next read returns zero)
         */
        virtual void resetIntegratedVelocity() {}

        // Wait for IMU to receive initial orientation data
        // Returns true if data received within timeout_ms, false otherwise
        virtual bool waitForReady(int timeout_ms = 1000) = 0;

    private:
        TurnAxisMode yaw_rate_axis_mode_{TurnAxisMode::WorldZ};
        float yaw_rate_sign_{1.0f};
    };
}
