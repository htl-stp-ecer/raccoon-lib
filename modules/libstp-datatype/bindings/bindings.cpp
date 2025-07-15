//
// Created by tobias on 12/29/24.
//

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>
#include <pybind11/chrono.h>

namespace py = pybind11;

void init_axis(const py::module& m);
void init_conditions(const py::module& m);
void init_functions(py::module& m);
void init_speed(const py::module& m);

PYBIND11_MODULE(datatype, m)
{
    m.doc() = "Python bindings for libstp-datatype";

    init_axis(m);
    init_conditions(m);
    init_functions(m);
    init_speed(m);
}
