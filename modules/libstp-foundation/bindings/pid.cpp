#include "foundation/pid.hpp"

#include <pybind11/pybind11.h>

#include <sstream>

namespace py = pybind11;

namespace {

std::string pid_config_to_string(const libstp::foundation::PidConfig& cfg)
{
    std::ostringstream oss;
    oss << "PidConfig(kp=" << cfg.kp << ", ki=" << cfg.ki << ", kd=" << cfg.kd
        << ", integral_max=" << cfg.integral_max
        << ", integral_deadband=" << cfg.integral_deadband
        << ", derivative_lpf_alpha=" << cfg.derivative_lpf_alpha
        << ", output_min=" << cfg.output_min
        << ", output_max=" << cfg.output_max << ")";
    return oss.str();
}

} // namespace

void init_pid(const py::module& m)
{
    using namespace libstp::foundation;

    py::class_<PidConfig>(m, "PidConfig")
        .def(py::init([](double kp, double ki, double kd,
                         double integral_max, double integral_deadband,
                         double derivative_lpf_alpha,
                         double output_min, double output_max)
             {
                 PidConfig cfg;
                 cfg.kp = kp;
                 cfg.ki = ki;
                 cfg.kd = kd;
                 cfg.integral_max = integral_max;
                 cfg.integral_deadband = integral_deadband;
                 cfg.derivative_lpf_alpha = derivative_lpf_alpha;
                 cfg.output_min = output_min;
                 cfg.output_max = output_max;
                 return cfg;
             }),
             py::arg("kp") = 1.0,
             py::arg("ki") = 0.0,
             py::arg("kd") = 0.0,
             py::arg("integral_max") = 10.0,
             py::arg("integral_deadband") = 0.01,
             py::arg("derivative_lpf_alpha") = 0.1,
             py::arg("output_min") = -10.0,
             py::arg("output_max") = 10.0)
        .def_readwrite("kp", &PidConfig::kp)
        .def_readwrite("ki", &PidConfig::ki)
        .def_readwrite("kd", &PidConfig::kd)
        .def_readwrite("integral_max", &PidConfig::integral_max)
        .def_readwrite("integral_deadband", &PidConfig::integral_deadband)
        .def_readwrite("derivative_lpf_alpha", &PidConfig::derivative_lpf_alpha)
        .def_readwrite("output_min", &PidConfig::output_min)
        .def_readwrite("output_max", &PidConfig::output_max)
        .def("__repr__", &pid_config_to_string)
        .def("__str__", &pid_config_to_string);

    py::class_<PidController>(m, "PidController")
        .def(py::init<PidConfig>(), py::arg("config") = PidConfig{})
        .def("update", py::overload_cast<double, double>(&PidController::update),
             py::arg("error"), py::arg("dt"))
        .def("reset", &PidController::reset)
        .def("set_gains", &PidController::setGains,
             py::arg("kp"), py::arg("ki"), py::arg("kd"))
        .def_property_readonly("integral", &PidController::getIntegral)
        .def_property_readonly("derivative", &PidController::getDerivative);
}
