#pragma once

#include <Eigen/Geometry>
#include <memory>

#include "motion/motion.hpp"

namespace libstp::motion
{
    class MotionPidController;

    struct DriveStraightConfig
    {
        double distance_m{0.0};
        double max_speed_mps{0.0};
        double distance_tolerance_m{0.01};

        // Distance PID gains
        double distance_kp{2.0};
        double distance_ki{0.0};
        double distance_kd{0.0};

        // Heading PID gains
        double heading_kp{4.0};
        double heading_ki{0.0};
        double heading_kd{0.0};
        double max_heading_rate{3.0};

        double saturation_derating_factor{0.85};  // Multiply translational speed by this on each saturation hit
        double saturation_min_speed_scale{0.1};   // Never reduce translational scale below this
        double saturation_recovery_rate{0.02};    // How quickly translational scale recovers per cycle
        double saturation_heading_error_rad{0.01};  // Only derate when heading error exceeds this

        double heading_saturation_derating_factor{0.85};  // Multiply heading scale when still saturated
        double heading_min_scale{0.25};                   // Minimum heading scale
        double heading_recovery_rate{0.05};               // Recovery rate for heading scale
        double heading_recovery_error_rad{0.005};         // Allow recovery once heading error is below this

        // Lateral drift correction parameters
        double lateral_kp{2.0};  // Proportional gain for lateral correction (mecanum uses vy, differential uses heading bias)
        double lateral_ki{0.0};
        double lateral_kd{0.0};
        double lateral_heading_bias_gain{0.5};  // For differential: how much to bias heading based on lateral error
        double lateral_reorient_threshold_m{0.15};  // For differential: stop and reorient if lateral error exceeds this
    };

    class DriveStraightMotion final : public Motion
    {
    public:
        DriveStraightMotion(MotionContext ctx, double distance_cm, double max_speed_mps);
        DriveStraightMotion(MotionContext ctx, DriveStraightConfig config);

        void start() override;
        void update(double dt) override;
        [[nodiscard]] bool isFinished() const override;
    private:
        void complete();

        DriveStraightConfig cfg_{};
        std::unique_ptr<MotionPidController> distance_pid_;  // PID controller for distance
        std::unique_ptr<MotionPidController> heading_pid_;   // PID controller for heading
        std::unique_ptr<MotionPidController> lateral_pid_;   // PID controller for lateral drift
        double initial_heading_rad_{0.0};  // Heading at start (target heading to maintain)
        bool finished_{false};
        double speed_scale_{1.0};    // Current translational speed scaling factor due to saturation
        double heading_scale_{1.0};  // Current heading command scaling factor
        bool reorienting_{false};  // For differential: currently in reorientation mode
    };
}
