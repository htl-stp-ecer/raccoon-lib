#include <pybind11/pybind11.h>
#include "foundation/motor.hpp"

namespace py = pybind11;
using namespace py::literals;

void init_motor(const py::module_& m)
{
    auto pid_cls = py::class_<libstp::foundation::PidGains>(m, "PidGains")
                   .def(py::init<>())
                   .def_readwrite("kp", &libstp::foundation::PidGains::kp)
                   .def_readwrite("ki", &libstp::foundation::PidGains::ki)
                   .def_readwrite("kd", &libstp::foundation::PidGains::kd);

    auto ff_cls = py::class_<libstp::foundation::Feedforward>(m, "Feedforward")
                  .def(py::init<>())
                  .def_readwrite("kS", &libstp::foundation::Feedforward::kS)
                  .def_readwrite("kV", &libstp::foundation::Feedforward::kV)
                  .def_readwrite("kA", &libstp::foundation::Feedforward::kA);

    auto dz_cls = py::class_<libstp::foundation::Deadzone>(m, "Deadzone")
                  .def(py::init<>())
                  .def_readwrite("enable", &libstp::foundation::Deadzone::enable)
                  .def_readwrite("zero_window_percent", &libstp::foundation::Deadzone::zero_window_percent)
                  .def_readwrite("start_percent", &libstp::foundation::Deadzone::start_percent)
                  .def_readwrite("release_percent", &libstp::foundation::Deadzone::release_percent);

    auto mc_cls = py::class_<libstp::foundation::MotorCalibration>(m, "MotorCalibration")
                  .def(py::init<>())
                  .def(
                      py::init<
                          const libstp::foundation::Feedforward&,
                          const libstp::foundation::PidGains&,
                          const libstp::foundation::Deadzone&,
                          double, double, double>(),
                      py::arg("ff"), py::arg("pid"), py::arg("deadzone"),
                      py::arg("max_percent_output"), py::arg("ticks_to_rad"), py::arg("vel_lpf_alpha")
                  )
                  .def_readwrite("ff", &libstp::foundation::MotorCalibration::ff)
                  .def_readwrite("pid", &libstp::foundation::MotorCalibration::pid)
                  .def_readwrite("deadzone", &libstp::foundation::MotorCalibration::deadzone)
                  .def_readwrite("max_percent_output", &libstp::foundation::MotorCalibration::max_percent_output)
                  .def_readwrite("ticks_to_rad", &libstp::foundation::MotorCalibration::ticks_to_rad)
                  .def_readwrite("vel_lpf_alpha", &libstp::foundation::MotorCalibration::vel_lpf_alpha);

    mc_cls.attr("__annotations__") = py::dict(
        "ff"_a = ff_cls,
        "pid"_a = pid_cls,
        "deadzone"_a = dz_cls,
        "max_percent_output"_a = py::module_::import("builtins").attr("float"),
        "ticks_to_rad"_a = py::module_::import("builtins").attr("float"),
        "vel_lpf_alpha"_a = py::module_::import("builtins").attr("float")
    );

    py::module_ inspect = py::module_::import("inspect");
    py::object Parameter = inspect.attr("Parameter");
    py::object Signature = inspect.attr("Signature");
    py::object POK = Parameter.attr("POSITIONAL_OR_KEYWORD");
    py::object empty = Parameter.attr("empty");

    auto Param = [&](const char* name, py::object ann)
    {
        return Parameter(name, POK, "default"_a = empty, "annotation"_a = ann);
    };

    py::list params;
    params.append(Param("ff", ff_cls));
    params.append(Param("pid", pid_cls));
    params.append(Param("deadzone", dz_cls));
    params.append(Param("max_percent_output", py::module_::import("builtins").attr("float")));
    params.append(Param("ticks_to_rad", py::module_::import("builtins").attr("float")));
    params.append(Param("vel_lpf_alpha", py::module_::import("builtins").attr("float")));

    mc_cls.attr("__signature__") = Signature("parameters"_a = params,
                                             "return_annotation"_a = mc_cls);
}
