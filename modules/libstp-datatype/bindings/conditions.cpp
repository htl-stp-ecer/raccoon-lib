//
// Created by tobias on 6/8/25.
//

#include "datatype/conditions.hpp"

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>
#include <pybind11/chrono.h>

namespace py = pybind11;

class ConditionalResultTrampoline final : public libstp::datatype::ConditionalResult
{
public:
    using ConditionalResult::ConditionalResult;

    [[nodiscard]] float progress() const override
    {
        PYBIND11_OVERLOAD_PURE(
                float,
            ConditionalResult,
            progress
        );
    }

    [[nodiscard]] bool is_loop_running() const override
    {
        PYBIND11_OVERLOAD_PURE(
                bool,
            ConditionalResult,
            is_loop_running
        );
    }

    void update(libstp::motion::DifferentialDriveState& state) override
    {
        PYBIND11_OVERLOAD_PURE(
                void,
            ConditionalResult,
            update
        );
    }

    [[nodiscard]] std::string to_string() const override
    {
        PYBIND11_OVERLOAD_PURE(
            std::string,
            ConditionalResult,
            to_string
        );
    }
};

void init_conditions(const py::module& m)
{
    py::class_<libstp::datatype::ConditionalResult, ConditionalResultTrampoline, std::shared_ptr<libstp::datatype::ConditionalResult>>(
            m, "ConditionalResult", "Base class for conditional results")
        .def(py::init<>(), "Initialize a ConditionalResult")
        .def("progress", &libstp::datatype::ConditionalResult::progress, py::return_value_policy::reference,
             "Get the progress of the condition")
        .def("is_loop_running", &libstp::datatype::ConditionalResult::is_loop_running, py::return_value_policy::reference,
             "Check if the loop is running")
        .def("to_string", &libstp::datatype::ConditionalResult::to_string, py::return_value_policy::copy,
             "Get a string representation of the condition state")
        .def("__str__", &libstp::datatype::ConditionalResult::to_string)
        .def("__repr__", [](const libstp::datatype::ConditionalResult& self)
        {
            return "<ConditionalResult: " + self.to_string() + ">";
        });

    py::class_<libstp::datatype::UndefinedConditionalResult, libstp::datatype::ConditionalResult, std::shared_ptr<libstp::datatype::UndefinedConditionalResult>>(
            m, "UndefinedConditionalResult",
            "Represents a condition that may or may not be met")
        .def(py::init<const bool>(), py::arg("conditionMet"), "Initialize with whether the condition is met")
        .def("__str__", &libstp::datatype::UndefinedConditionalResult::to_string)
        .def("__repr__", [](const libstp::datatype::UndefinedConditionalResult& self)
        {
            return "<UndefinedConditionalResult(condition_met=" +
                std::string(self._conditionMet ? "True" : "False") + ")>";
        });

    py::class_<libstp::datatype::DefinedConditionalResult, libstp::datatype::ConditionalResult, std::shared_ptr<libstp::datatype::DefinedConditionalResult>>(
            m, "DefinedConditionalResult",
            "Represents a condition with a defined target")
        .def(py::init<const float>(), py::arg("target"), "Initialize with a target value")
        .def_readwrite("target", &libstp::datatype::DefinedConditionalResult::target, "The target value")
        .def_readwrite("current", &libstp::datatype::DefinedConditionalResult::current, "The current value")
        .def("__str__", &libstp::datatype::DefinedConditionalResult::to_string)
        .def("__repr__", [](const libstp::datatype::DefinedConditionalResult& self)
        {
            return "<DefinedConditionalResult(target=" + std::to_string(self.target) +
                ", current=" + std::to_string(self.current) + ")>";
        });

    py::class_<libstp::datatype::DistanceConditionalResult, libstp::datatype::DefinedConditionalResult, std::shared_ptr<libstp::datatype::DistanceConditionalResult>>(
            m, "DistanceConditionalResult",
            "Represents a distance-based condition")
        .def(py::init<const float>(), py::arg("target"), "Initialize with a distance target")
        .def("__str__", &libstp::datatype::DistanceConditionalResult::to_string)
        .def("__repr__", [](const libstp::datatype::DistanceConditionalResult& self)
        {
            return "<DistanceConditionalResult(target=" + std::to_string(self.target) +
                ", current=" + std::to_string(self.current) + ")>";
        });

    py::class_<libstp::datatype::RotationConditionalResult, libstp::datatype::DefinedConditionalResult, std::shared_ptr<libstp::datatype::RotationConditionalResult>>(
            m, "RotationConditionalResult",
            "Represents a rotation-based condition")
        .def(py::init<const float>(), py::arg("target"), "Initialize with a rotation target")
        .def("__str__", &libstp::datatype::RotationConditionalResult::to_string)
        .def("__repr__", [](const libstp::datatype::RotationConditionalResult& self)
        {
            return "<RotationConditionalResult(target=" + std::to_string(self.target) +
                ", current=" + std::to_string(self.current) + ")>";
        });

    py::class_<libstp::datatype::MotorTicksConditionalResult, libstp::datatype::DefinedConditionalResult, std::shared_ptr<libstp::datatype::MotorTicksConditionalResult>>(
            m, "MotorTicksConditionalResult",
            "Represents a motor ticks-based condition")
        .def(py::init<const float>(), py::arg("target"), "Initialize with a motor ticks target")
        .def("__str__", &libstp::datatype::MotorTicksConditionalResult::to_string)
        .def("__repr__", [](const libstp::datatype::MotorTicksConditionalResult& self)
        {
            return "<MotorTicksConditionalResult(target=" + std::to_string(self.target) +
                ", current=" + std::to_string(self.current) + ")>";
        });

    py::class_<libstp::datatype::TimedConditionalResult, libstp::datatype::DefinedConditionalResult, std::shared_ptr<libstp::datatype::TimedConditionalResult>>(
            m, "TimedConditionalResult",
            "Represents a time-based condition")
        .def(py::init<const float, const float>(), py::arg("target"), py::arg("current"),
             "Initialize with a time target and current time")
        .def("__str__", &libstp::datatype::TimedConditionalResult::to_string)
        .def("__repr__", [](const libstp::datatype::TimedConditionalResult& self)
        {
            return "<TimedConditionalResult(target=" + std::to_string(self.target) +
                ", current=" + std::to_string(self.current) + ")>";
        });
}
