#include <pybind11/pybind11.h>
#include "motion/motion.hpp"

namespace py = pybind11;

void init_motion_base(py::module_& m)
{
    py::class_<libstp::motion::Motion, std::shared_ptr<libstp::motion::Motion>>(m, "Motion")
        .def("start", &libstp::motion::Motion::start)
        .def("update", &libstp::motion::Motion::update, py::arg("dt"))
        .def("is_finished", &libstp::motion::Motion::isFinished);
}
