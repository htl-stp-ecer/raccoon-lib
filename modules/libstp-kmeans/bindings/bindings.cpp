#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "Kmeans.hpp"

namespace py = pybind11;

PYBIND11_MODULE(kmeans, m) {
    m.doc() = "Python bindings for libstp-kmeans";

    py::class_<libstp::kmeans::KMeansResult>(m, "KMeansResult")
        .def(py::init<>())
        .def_readwrite("centroid1", &libstp::kmeans::KMeansResult::centroid1)
        .def_readwrite("centroid2", &libstp::kmeans::KMeansResult::centroid2);

    py::class_<libstp::kmeans::KMeans>(m, "KMeans")
        .def(py::init<int>(), py::arg("max_iterations") = 10)
        .def("fit", &libstp::kmeans::KMeans::fit, py::arg("data"));
}
