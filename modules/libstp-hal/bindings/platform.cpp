#define PYBIND11_DETAILED_ERROR_MESSAGES
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "hal/Platform.hpp"

namespace py = pybind11;

void init_platform(py::module& m)
{
    using namespace libstp::hal::platform;

    auto sub = m.def_submodule("platform", "Hardware health-check API");

    py::enum_<Component>(sub, "Component")
        .value("STM32_BRIDGE", Component::Stm32Bridge)
        .value("IMU",          Component::Imu)
        .value("MOTORS",       Component::Motors)
        .export_values();

    py::class_<ComponentStatus>(sub, "ComponentStatus")
        .def_readonly("component", &ComponentStatus::component)
        .def_readonly("ok",        &ComponentStatus::ok)
        .def_readonly("detail",    &ComponentStatus::detail)
        .def("__repr__", [](const ComponentStatus& s) {
            return std::string("<ComponentStatus ") + toString(s.component)
                 + " ok=" + (s.ok ? "True" : "False")
                 + (s.detail.empty() ? "" : (" detail='" + s.detail + "'")) + ">";
        });

    py::class_<ProbeResult>(sub, "ProbeResult")
        .def_readonly("ok",         &ProbeResult::ok)
        .def_readonly("components", &ProbeResult::components)
        .def("failed_components",   &ProbeResult::failedComponents)
        .def("summary",             &ProbeResult::summary)
        .def("__bool__",            [](const ProbeResult& r) { return r.ok; })
        .def("__repr__",            [](const ProbeResult& r) { return r.summary(); });

    py::class_<ProbeOptions>(sub, "ProbeOptions")
        .def(py::init<>())
        .def_readwrite("stm32_timeout_ms", &ProbeOptions::stm32_timeout_ms)
        .def_readwrite("imu_timeout_ms",   &ProbeOptions::imu_timeout_ms)
        .def_readwrite("motor_timeout_ms", &ProbeOptions::motor_timeout_ms)
        .def_readwrite("require_stm32",    &ProbeOptions::require_stm32)
        .def_readwrite("require_imu",      &ProbeOptions::require_imu)
        .def_readwrite("require_motors",   &ProbeOptions::require_motors);

    // Translate the C++ exception into a Python RuntimeError carrying the
    // probe summary so callers see the failed components in the traceback.
    py::register_exception<ProbeFailedError>(sub, "ProbeFailedError", PyExc_RuntimeError);

    py::class_<Platform>(sub, "Platform")
        .def_static("probe",            &Platform::probe,            py::arg("options") = ProbeOptions{})
        .def_static("require_healthy",  &Platform::requireHealthy,   py::arg("options") = ProbeOptions{})
        .def_static("set_mock_probe_failure",   &Platform::setMockProbeFailure,
                    py::arg("component"), py::arg("fail"))
        .def_static("clear_mock_probe_failures", &Platform::clearMockProbeFailures);
}
