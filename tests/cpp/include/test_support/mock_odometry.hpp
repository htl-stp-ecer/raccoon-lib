#pragma once

#include <gmock/gmock.h>
#include "odometry/odometry.hpp"
#include <cmath>

namespace libstp::test
{
    class MockOdometry : public odometry::IOdometry
    {
    public:
        MOCK_METHOD(void, update, (double dt), (override));
        MOCK_METHOD(foundation::Pose, getPose, (), (const, override));
        MOCK_METHOD(odometry::DistanceFromOrigin, getDistanceFromOrigin, (), (const, override));
        MOCK_METHOD(double, getHeading, (), (const, override));
        MOCK_METHOD(double, getAbsoluteHeading, (), (const, override));
        MOCK_METHOD(double, getHeadingError, (double target_heading_rad), (const, override));
        MOCK_METHOD(void, reset, (), (override));

        // Simulation state for testing motion controllers
        void simulateForwardProgress(double forward_m, double lateral_m = 0.0) {
            current_distance_.forward = forward_m;
            current_distance_.lateral = lateral_m;
            current_distance_.straight_line = std::sqrt(forward_m * forward_m + lateral_m * lateral_m);
            ON_CALL(*this, getDistanceFromOrigin())
                .WillByDefault(testing::Return(current_distance_));
        }

        void setHeading(double heading_rad) {
            current_heading_ = heading_rad;
            ON_CALL(*this, getHeading()).WillByDefault(testing::Return(heading_rad));
        }

        void setHeadingError(double error_rad) {
            ON_CALL(*this, getHeadingError(testing::_)).WillByDefault(testing::Return(error_rad));
        }

        void setPose(const foundation::Pose& pose) {
            current_pose_ = pose;
            ON_CALL(*this, getPose()).WillByDefault(testing::Return(pose));
        }

        // Default setup for common test scenarios
        void setupDefaults() {
            simulateForwardProgress(0.0, 0.0);
            setHeading(0.0);
            setHeadingError(0.0);
            setPose(foundation::Pose{});
        }

    private:
        odometry::DistanceFromOrigin current_distance_{0, 0, 0};
        double current_heading_{0.0};
        foundation::Pose current_pose_{};
    };
}
