#pragma once

#include <Eigen/Geometry>

#include "motion/motion.hpp"

namespace libstp::motion
{
    struct MoveToPoseConfig
    {
        foundation::Pose target_pose{};  // Target pose relative to starting position
        double max_linear_speed_mps{0.3};
        double max_angular_speed_rps{1.0};
        double position_tolerance_m{0.02};
        double heading_tolerance_rad{0.05};

        // Proportional gains
        double position_kp{2.0};
        double heading_kp{3.0};

        // Differential drive specific
        double lateral_heading_bias_gain{0.5};  // How much to bias heading based on lateral error
        double lateral_reorient_threshold_m{0.15};  // Stop and reorient if lateral error exceeds this

        double saturation_derating_factor{0.85};  // Scale back speed when saturated
    };

    /**
     * Move-to-pose motion controller
     *
     * Navigates from current pose to a target pose (specified relative to starting position).
     * Works with both differential and mecanum drive:
     * - Mecanum: Uses direct x/y/heading control
     * - Differential: Uses forward + rotation with heading bias for lateral correction
     *
     * Does NOT use global coordinates - all targets are relative to starting position.
     */
    class MoveToPoseMotion final : public Motion
    {
    public:
        MoveToPoseMotion(MotionContext ctx, foundation::Pose target_pose,
                        double max_linear_speed_mps, double max_angular_speed_rps);
        explicit MoveToPoseMotion(MotionContext ctx, MoveToPoseConfig config);

        void start() override;
        void update(double dt) override;
        [[nodiscard]] bool isFinished() const override;

    private:
        void complete();
        double computeYaw(const Eigen::Quaternionf& orientation) const;
        static double wrapAngle(double angle);

        MoveToPoseConfig cfg_{};
        Eigen::Quaternionf reference_orientation_{Eigen::Quaternionf::Identity()};
        Eigen::Vector3f reference_position_{Eigen::Vector3f::Zero()};
        bool finished_{false};
        double speed_scale_{1.0};
        bool reorienting_{false};  // For differential: currently in reorientation mode
    };
}
