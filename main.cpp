//
// Created by tobias on 6/9/25.
//

#include <pybind11/pybind11.h>

namespace py = pybind11;

PYBIND11_MODULE(_core, m) {
    m.doc() = "Python bindings for libstp";
}
