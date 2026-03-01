//
// Created by tobias on 6/9/25.
//

#include <pybind11/pybind11.h>

namespace py = pybind11;

// Stringify macro for VERSION_INFO
#define STRINGIFY(x) #x
#define MACRO_STRINGIFY(x) STRINGIFY(x)

PYBIND11_MODULE(_core, m) {
    m.doc() = "Python bindings for libstp";
#ifdef VERSION_INFO
    m.attr("__version__") = MACRO_STRINGIFY(VERSION_INFO);
#else
    m.attr("__version__") = "dev";
#endif
}
