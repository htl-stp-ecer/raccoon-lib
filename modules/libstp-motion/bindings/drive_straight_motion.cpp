#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "motion/drive_straight_motion.hpp"
#include "motion/motion_pid.hpp"

namespace py = pybind11;

void init_drive_straight(py::module_& m)
{
    using namespace libstp::motion;

    py::class_<DriveStraightConfig>(m, "DriveStraightConfig")
        .def(py::init<>())
        .def_readwrite("distance_m", &DriveStraightConfig::distance_m)
        .def_readwrite("max_speed_mps", &DriveStraightConfig::max_speed_mps)
        .def_readwrite("distance_tolerance_m", &DriveStraightConfig::distance_tolerance_m)
        .def_readwrite("distance_kp", &DriveStraightConfig::distance_kp)
        .def_readwrite("distance_ki", &DriveStraightConfig::distance_ki)
        .def_readwrite("distance_kd", &DriveStraightConfig::distance_kd)
        .def_readwrite("heading_kp", &DriveStraightConfig::heading_kp)
        .def_readwrite("heading_ki", &DriveStraightConfig::heading_ki)
        .def_readwrite("heading_kd", &DriveStraightConfig::heading_kd)
        .def_readwrite("max_heading_rate", &DriveStraightConfig::max_heading_rate)
        .def_readwrite("saturation_derating_factor", &DriveStraightConfig::saturation_derating_factor)
        .def_readwrite("saturation_min_speed_scale", &DriveStraightConfig::saturation_min_speed_scale)
        .def_readwrite("saturation_recovery_rate", &DriveStraightConfig::saturation_recovery_rate)
        .def_readwrite("saturation_heading_error_rad", &DriveStraightConfig::saturation_heading_error_rad)
        .def_readwrite("heading_saturation_derating_factor", &DriveStraightConfig::heading_saturation_derating_factor)
        .def_readwrite("heading_min_scale", &DriveStraightConfig::heading_min_scale)
        .def_readwrite("heading_recovery_rate", &DriveStraightConfig::heading_recovery_rate)
        .def_readwrite("heading_recovery_error_rad", &DriveStraightConfig::heading_recovery_error_rad)
        .def_readwrite("lateral_kp", &DriveStraightConfig::lateral_kp)
        .def_readwrite("lateral_ki", &DriveStraightConfig::lateral_ki)
        .def_readwrite("lateral_kd", &DriveStraightConfig::lateral_kd)
        .def_readwrite("lateral_heading_bias_gain", &DriveStraightConfig::lateral_heading_bias_gain)
        .def_readwrite("lateral_reorient_threshold_m", &DriveStraightConfig::lateral_reorient_threshold_m);

    py::class_<DriveStraightMotion, Motion, std::shared_ptr<DriveStraightMotion>>(m, "DriveStraightMotion")
        .def(py::init([](libstp::drive::Drive& drive,
                         libstp::odometry::IOdometry& odometry,
                         double distance_cm,
                         double max_speed_mps)
        {
            MotionContext ctx{drive, odometry};
            return std::make_shared<DriveStraightMotion>(ctx, distance_cm, max_speed_mps);
        }),
            py::arg("drive"),
            py::arg("odometry"),
            py::arg("distance_cm"),
            py::arg("max_speed_mps"),
            py::keep_alive<1, 2>(),
            py::keep_alive<1, 3>())
        .def(py::init([](libstp::drive::Drive& drive,
                         libstp::odometry::IOdometry& odometry,
                         const DriveStraightConfig& config)
        {
            MotionContext ctx{drive, odometry};
            return std::make_shared<DriveStraightMotion>(ctx, config);
        }),
            py::arg("drive"),
            py::arg("odometry"),
            py::arg("config"),
            py::keep_alive<1, 2>(),
            py::keep_alive<1, 3>())
        .def("start", &DriveStraightMotion::start)
        .def("update", &DriveStraightMotion::update, py::arg("dt"))
        .def("is_finished", &DriveStraightMotion::isFinished);
}
