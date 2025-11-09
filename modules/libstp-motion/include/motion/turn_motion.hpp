#pragma once

#include <Eigen/Geometry>

#include "motion/motion.hpp"

namespace libstp::motion
{
    struct TurnMotionConfig
    {
        double angle_deg{0.0};  // Target turn angle in degrees
        double max_angular_speed_rps{0.0};  // Maximum angular speed in radians per second
        double angle_tolerance_deg{2.0};  // Tolerance in degrees
        double angle_kp{3.0};  // Proportional gain for angular control
        double saturation_derating_factor{0.85};  // Multiply angular speed by this on saturation
        double saturation_min_speed_scale{0.1};  // Never reduce angular scale below this
        double saturation_recovery_rate{0.02};  // How quickly angular scale recovers per cycle
    };

    class TurnMotion final : public Motion
    {
    public:
        TurnMotion(MotionContext ctx, double angle_deg, double max_angular_speed_rps);
        TurnMotion(MotionContext ctx, TurnMotionConfig config);

        void start() override;
        void update(double dt) override;
        [[nodiscard]] bool isFinished() const override;
    private:
        void complete();
        double computeYaw(const Eigen::Quaternionf& orientation) const;
        static double wrapAngle(double angle);

        TurnMotionConfig cfg_{};
        Eigen::Quaternionf reference_orientation_{Eigen::Quaternionf::Identity()};  // Starting orientation
        double target_angle_rad_{0.0};  // Target angle to turn (not yaw position!)
        bool finished_{false};
        double angular_scale_{1.0};  // Current angular speed scaling factor due to saturation
        double cumulative_angle_rad_{0.0};  // Cumulative rotation tracking (handles >180° turns)
        double previous_angle_rad_{0.0};  // Previous angle for wrap detection
    };
}
