//
// Created by eternalRose on 11/17/25.
//
#include <pybind11/pybind11.h>
#include <memory>
#include "button/button.hpp"
#include "hal/Digital.hpp"

namespace py = pybind11;

void init_button(py::module& m) {
    // DigitalSensor lives in the hal module; importing it ensures the type
    // (and its shared_ptr holder) is registered before set_digital is called.
    py::module::import("raccoon.hal");

    // Expose singleton methods as module-level functions
    m.def("is_pressed", []() {
        return libstp::button::Button::instance().isPressed();
    }, "Check if the button is pressed");

    m.def("set_digital", [](int port) {
        libstp::button::Button::instance().setDigital(port);
    }, "Set the button from a digital sensor port", py::arg("port"));

    // Share the existing sensor instance (preserves the dynamic type, e.g. a
    // ButtonGroup) instead of rebuilding one from its port number.
    m.def("set_digital", [](std::shared_ptr<libstp::hal::digital::DigitalSensor> sensor) {
        libstp::button::Button::instance().setDigital(std::move(sensor));
    }, "Set the button from an existing digital sensor (shares the instance)", py::arg("sensor"));

    m.def("wait_for_button_press", []() {
        libstp::button::Button::instance().waitForButtonPress();
    }, "Wait for the button to be pressed");
}
