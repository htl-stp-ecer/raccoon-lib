#pragma once

#include <vector>

#include <Eigen/Core>

#include "foundation/types.hpp"
#include "motion/motion.hpp"

namespace libstp::motion
{
    enum class LineFollowCorrectionMode
    {
        Angular,
        Lateral,
        Forward,
    };

    struct DirectionalLineFollowMotionConfig
    {
        double heading_speed{0.0};  // -1..1 fraction of max forward velocity
        double strafe_speed{0.0};   // -1..1 fraction of max lateral velocity
        double distance_m{0.0};
        bool has_distance_target{false};
        double kp{0.4};
        double ki{0.0};
        double kd{0.1};
        LineFollowCorrectionMode correction_mode{LineFollowCorrectionMode::Angular};
        bool heading_hold{true};
        double correction_sign{1.0};
    };

    struct DirectionalLineFollowMotionTelemetry
    {
        double time_s{0.0};
        double dt{0.0};
        double error{0.0};
        double correction{0.0};
        double heading_error_rad{0.0};
        double heading_speed_mps{0.0};
        double strafe_speed_mps{0.0};
        double cmd_vx_mps{0.0};
        double cmd_vy_mps{0.0};
        double cmd_wz_radps{0.0};
        double straight_distance_m{0.0};
        bool finished{false};
    };

    class DirectionalLineFollowMotion final : public Motion
    {
    public:
        DirectionalLineFollowMotion(MotionContext ctx, DirectionalLineFollowMotionConfig config);

        void start() override;
        void update(double dt) override;
        [[nodiscard]] bool isFinished() const override;

        void setSensorError(double error);
        [[nodiscard]] const std::vector<DirectionalLineFollowMotionTelemetry>& getTelemetry() const
        {
            return telemetry_;
        }

    private:
        void captureInitialPose();
        [[nodiscard]] double straightDistanceFromStart() const;
        void complete();

        DirectionalLineFollowMotionConfig cfg_{};
        std::unique_ptr<foundation::PidController> correction_pid_;
        std::unique_ptr<foundation::PidController> heading_pid_;
        Eigen::Vector2d initial_position_m_{Eigen::Vector2d::Zero()};
        double initial_heading_rad_{0.0};
        double vx_mps_{0.0};
        double vy_mps_{0.0};
        double max_linear_mps_{0.0};
        double max_lateral_mps_{0.0};
        double sensor_error_{0.0};
        double elapsed_time_{0.0};
        bool finished_{false};
        static constexpr size_t kMaxTelemetrySamples{3000};
        std::vector<DirectionalLineFollowMotionTelemetry> telemetry_;
    };
}
