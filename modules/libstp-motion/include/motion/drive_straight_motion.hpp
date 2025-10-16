#pragma once

#include <Eigen/Geometry>

#include "motion/motion.hpp"

namespace libstp::motion
{
    struct DriveStraightConfig
    {
        double distance_m{0.0};
        double max_speed_mps{0.0};
        double distance_tolerance_m{0.01};
        double distance_kp{2.0};
        double heading_kp{4.0};
        double max_heading_rate{3.0};
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
        double computeYaw(const Eigen::Quaternionf& orientation) const;
        static double wrapAngle(double angle);

        DriveStraightConfig cfg_{};
        double reference_yaw_{0.0};
        double distance_travelled_m_{0.0};
        bool finished_{false};
    };
}
