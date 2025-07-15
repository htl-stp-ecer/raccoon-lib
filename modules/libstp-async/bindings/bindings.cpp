//
// Created by tobias on 6/1/25.
//
#include <pybind11/pybind11.h>

#include "async/algorithm.hpp"
namespace py = pybind11;

PYBIND11_MODULE(async, m) {
    m.doc() = "Python bindings for libstp-async";

    py::class_<libstp::async::AsyncAlgorithm<int>>(m, "AsyncAlgorithmInt")
        .def("advance", &libstp::async::AsyncAlgorithm<int>::advance)
        .def("current", &libstp::async::AsyncAlgorithm<int>::current);
}
