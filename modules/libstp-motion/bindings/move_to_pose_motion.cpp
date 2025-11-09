#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>

#include "motion/move_to_pose_motion.hpp"

namespace py = pybind11;

void init_move_to_pose(py::module_& m)
{
    using namespace libstp::motion;

    py::class_<MoveToPoseConfig>(m, "MoveToPoseConfig",
        "Configuration for move-to-pose motion controller")
        .def(py::init<>())
        .def_readwrite("target_pose", &MoveToPoseConfig::target_pose,
                      "Target pose relative to starting position")
        .def_readwrite("max_linear_speed_mps", &MoveToPoseConfig::max_linear_speed_mps,
                      "Maximum linear speed in m/s")
        .def_readwrite("max_angular_speed_rps", &MoveToPoseConfig::max_angular_speed_rps,
                      "Maximum angular speed in rad/s")
        .def_readwrite("position_tolerance_m", &MoveToPoseConfig::position_tolerance_m,
                      "Position tolerance for completion in meters")
        .def_readwrite("heading_tolerance_rad", &MoveToPoseConfig::heading_tolerance_rad,
                      "Heading tolerance for completion in radians")
        .def_readwrite("position_kp", &MoveToPoseConfig::position_kp,
                      "Proportional gain for position control")
        .def_readwrite("heading_kp", &MoveToPoseConfig::heading_kp,
                      "Proportional gain for heading control")
        .def_readwrite("lateral_heading_bias_gain", &MoveToPoseConfig::lateral_heading_bias_gain,
                      "Differential drive: heading bias gain based on lateral error")
        .def_readwrite("lateral_reorient_threshold_m", &MoveToPoseConfig::lateral_reorient_threshold_m,
                      "Differential drive: stop and reorient if lateral error exceeds this")
        .def_readwrite("saturation_derating_factor", &MoveToPoseConfig::saturation_derating_factor,
                      "Scale back speed when motors saturate");

    py::class_<MoveToPoseMotion, Motion, std::shared_ptr<MoveToPoseMotion>>(m, "MoveToPoseMotion",
        "Move-to-pose motion controller.\n\n"
        "Navigates from current pose to a target pose (specified relative to starting position).\n"
        "Works with both differential and mecanum drive:\n"
        "  - Mecanum: Uses direct x/y/heading control\n"
        "  - Differential: Uses forward + rotation with heading bias for lateral correction\n\n"
        "Does NOT use global coordinates - all targets are relative to starting position.")
        .def(py::init([](libstp::drive::Drive& drive,
                         libstp::odometry::IOdometry& odometry,
                         const libstp::foundation::Pose& target_pose,
                         double max_linear_speed_mps,
                         double max_angular_speed_rps)
        {
            MotionContext ctx{drive, odometry};
            return std::make_shared<MoveToPoseMotion>(ctx, target_pose, max_linear_speed_mps, max_angular_speed_rps);
        }),
            py::arg("drive"),
            py::arg("odometry"),
            py::arg("target_pose"),
            py::arg("max_linear_speed_mps"),
            py::arg("max_angular_speed_rps"),
            py::keep_alive<1, 2>(),
            py::keep_alive<1, 3>(),
            "Create MoveToPoseMotion with simple parameters.\n\n"
            "Args:\n"
            "    drive: Drive system instance\n"
            "    odometry: Odometry instance\n"
            "    target_pose: Target pose relative to start\n"
            "    max_linear_speed_mps: Maximum linear speed\n"
            "    max_angular_speed_rps: Maximum angular speed")
        .def(py::init([](libstp::drive::Drive& drive,
                         libstp::odometry::IOdometry& odometry,
                         const MoveToPoseConfig& config)
        {
            MotionContext ctx{drive, odometry};
            return std::make_shared<MoveToPoseMotion>(ctx, config);
        }),
            py::arg("drive"),
            py::arg("odometry"),
            py::arg("config"),
            py::keep_alive<1, 2>(),
            py::keep_alive<1, 3>(),
            "Create MoveToPoseMotion with full configuration.\n\n"
            "Args:\n"
            "    drive: Drive system instance\n"
            "    odometry: Odometry instance\n"
            "    config: Full configuration object")
        .def("start", &MoveToPoseMotion::start,
            "Start the motion (resets state)")
        .def("update", &MoveToPoseMotion::update,
            py::arg("dt"),
            "Update motion controller.\n\n"
            "Args:\n"
            "    dt: Time delta in seconds")
        .def("is_finished", &MoveToPoseMotion::isFinished,
            "Check if motion is complete.\n\n"
            "Returns:\n"
            "    True if within position and heading tolerances");
}
