//
// Created by eternalRose on 11/24/25.
//

#include <pybind11/pybind11.h>
namespace py = pybind11;

void init_button(py::module& m);

PYBIND11_MODULE(button, m) {
    m.doc() = "Python bindings for libstp-button";

    init_button(m);
}
