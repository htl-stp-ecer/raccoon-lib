//
// Created by tobias on 12/29/24.
//

#include <pybind11/pybind11.h>
#include <pybind11/chrono.h>

#include "foundation/config.hpp"
#include "foundation/pid.hpp"

namespace py = pybind11;

void init_pid(const py::module& m);
void init_logger(py::module_& m);
void init_motor(const py::module& m);
void init_types(const py::module& m);

PYBIND11_MODULE(foundation, m)
{
    m.doc() = "Python bindings for libstp-foundation";

    init_pid(m);
    init_logger(m);
    init_motor(m);
}
