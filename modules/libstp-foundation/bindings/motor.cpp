#include <pybind11/pybind11.h>
#include "foundation/motor.hpp"

#include <sstream>

namespace {

std::string pid_gains_to_string(const libstp::foundation::PidGains& gains)
{
    std::ostringstream oss;
    oss << "PidGains(kp=" << gains.kp << ", ki=" << gains.ki << ", kd=" << gains.kd << ")";
    return oss.str();
}

std::string feedforward_to_string(const libstp::foundation::Feedforward& ff)
{
    std::ostringstream oss;
    oss << "Feedforward(kS=" << ff.kS << ", kV=" << ff.kV << ", kA=" << ff.kA << ")";
    return oss.str();
}

std::string motor_calibration_to_string(const libstp::foundation::MotorCalibration& calibration)
{
    std::ostringstream oss;
    oss << "MotorCalibration(ticks_to_rad=" << calibration.ticks_to_rad
        << ", vel_lpf_alpha=" << calibration.vel_lpf_alpha << ")";
    return oss.str();
}

} // namespace

namespace py = pybind11;
using namespace py::literals;

void init_motor(const py::module_& m)
{
    py::class_<libstp::foundation::PidGains>(m, "PidGains")
        .def(py::init<>())
        .def(py::init<double, double, double>(),
             py::arg("kp"), py::arg("ki"), py::arg("kd"))
        .def_readwrite("kp", &libstp::foundation::PidGains::kp)
        .def_readwrite("ki", &libstp::foundation::PidGains::ki)
        .def_readwrite("kd", &libstp::foundation::PidGains::kd)
        .def("__repr__", &pid_gains_to_string)
        .def("__str__", &pid_gains_to_string);

    py::class_<libstp::foundation::Feedforward>(m, "Feedforward")
        .def(py::init<>())
        .def(py::init<double, double, double>(),
             py::arg("kS"), py::arg("kV"), py::arg("kA"))
        .def_readwrite("kS", &libstp::foundation::Feedforward::kS)
        .def_readwrite("kV", &libstp::foundation::Feedforward::kV)
        .def_readwrite("kA", &libstp::foundation::Feedforward::kA)
        .def("__repr__", &feedforward_to_string)
        .def("__str__", &feedforward_to_string);

    py::class_<libstp::foundation::MotorCalibration>(m, "MotorCalibration")
        .def(py::init<>())
        .def(
            py::init<double, double>(),
            py::arg("ticks_to_rad"), py::arg("vel_lpf_alpha")
        )
        .def_readwrite("ticks_to_rad", &libstp::foundation::MotorCalibration::ticks_to_rad)
        .def_readwrite("vel_lpf_alpha", &libstp::foundation::MotorCalibration::vel_lpf_alpha)
        .def("__repr__", &motor_calibration_to_string)
        .def("__str__", &motor_calibration_to_string);
}
