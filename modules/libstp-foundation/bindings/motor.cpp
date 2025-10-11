#include <pybind11/pybind11.h>
#include "foundation/motor.hpp"

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
        .def_readwrite("kd", &libstp::foundation::PidGains::kd);

    py::class_<libstp::foundation::Feedforward>(m, "Feedforward")
        .def(py::init<>())
        .def(py::init<double, double, double>(),
             py::arg("kS"), py::arg("kV"), py::arg("kA"))
        .def_readwrite("kS", &libstp::foundation::Feedforward::kS)
        .def_readwrite("kV", &libstp::foundation::Feedforward::kV)
        .def_readwrite("kA", &libstp::foundation::Feedforward::kA);

    py::class_<libstp::foundation::MotorCalibration>(m, "MotorCalibration")
        .def(py::init<>())
        .def(
            py::init<
                const libstp::foundation::Feedforward&,
                const libstp::foundation::PidGains&,
                double, double>(),
            py::arg("ff"), py::arg("pid"), py::arg("ticks_to_rad"), py::arg("vel_lpf_alpha")
        )
        .def_readwrite("ff", &libstp::foundation::MotorCalibration::ff)
        .def_readwrite("pid", &libstp::foundation::MotorCalibration::pid)
        .def_readwrite("ticks_to_rad", &libstp::foundation::MotorCalibration::ticks_to_rad)
        .def_readwrite("vel_lpf_alpha", &libstp::foundation::MotorCalibration::vel_lpf_alpha);
}
