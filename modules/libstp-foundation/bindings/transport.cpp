#include <pybind11/pybind11.h>

#include <cstdint>
#include <string>
#include <utility>

#ifdef LIBSTP_HAS_TRANSPORT
#include "transport_core/shared_transport.hpp"
#endif

namespace py = pybind11;

namespace
{
    struct PySubscription
    {
        std::uint64_t id{};
    };

#ifdef LIBSTP_HAS_TRANSPORT
    py::bytes encode_message(py::object message)
    {
        return py::reinterpret_borrow<py::bytes>(message.attr("encode")());
    }
#endif
}

void init_transport(py::module_& m)
{
    py::class_<PySubscription>(m, "Subscription")
        .def_property_readonly(
            "subscription_id", [](const PySubscription& sub) { return sub.id; });

#ifdef LIBSTP_HAS_TRANSPORT
    using libstp::transport_core::SharedTransport;

    py::class_<SharedTransport, std::unique_ptr<SharedTransport, py::nodelete>>(
        m, "SharedTransport")
        .def(
            "publish",
            [](SharedTransport& self,
               const std::string& channel,
               py::object message,
               bool reliable,
               bool retained,
               int retry_interval_ms,
               std::uint32_t max_retries)
            {
                try
                {
                    if (py::hasattr(message, "timestamp")
                        && message.attr("timestamp").cast<long long>() == 0)
                    {
                        py::module_ time = py::module_::import("time");
                        const auto now_us = static_cast<long long>(
                            time.attr("time")().cast<double>() * 1'000'000.0);
                        message.attr("timestamp") = now_us;
                    }
                }
                catch (const py::error_already_set&)
                {
                    // Messages without normal timestamp semantics are still publishable.
                }

                const std::string data = encode_message(std::move(message));
                py::gil_scoped_release release;
                const bool ok = self.publish_raw(
                    channel,
                    data.data(),
                    static_cast<int>(data.size()),
                    reliable,
                    retained,
                    retry_interval_ms,
                    max_retries);
                if (!ok)
                {
                    throw std::runtime_error("failed to publish on LCM channel " + channel);
                }
            },
            py::arg("channel"),
            py::arg("message"),
            py::arg("reliable") = false,
            py::arg("retained") = false,
            py::arg("retry_interval_ms") = 100,
            py::arg("max_retries") = 10)
        .def(
            "subscribe",
            [](SharedTransport& self,
               const std::string& channel,
               py::function handler,
               bool reliable,
               bool request_retained)
            {
                auto id = self.subscribe_raw(
                    channel,
                    [channel, handler = std::move(handler)](const void* data, int data_len)
                    {
                        py::gil_scoped_acquire acquire;
                        handler(
                            channel,
                            py::bytes(
                                static_cast<const char*>(data),
                                static_cast<std::size_t>(data_len)));
                    },
                    reliable,
                    request_retained);
                return PySubscription{id};
            },
            py::arg("channel"),
            py::arg("handler"),
            py::arg("reliable") = false,
            py::arg("request_retained") = false)
        .def(
            "unsubscribe",
            [](SharedTransport& self, const PySubscription& sub) { self.unsubscribe(sub.id); },
            py::arg("subscription"))
        .def("shutdown", &SharedTransport::shutdown);

    m.def(
        "get_transport",
        []() -> SharedTransport& { return SharedTransport::instance(); },
        py::return_value_policy::reference);

    m.def("shutdown_transport", []() { SharedTransport::instance().shutdown(); });
#else
    // Mock builds: keep the symbols importable so that downstream modules
    // (e.g. raccoon.ui.step) can `from raccoon.foundation import get_transport`
    // without ImportError. Calling get_transport() then raises so the caller
    // can choose how to degrade.
    m.def(
        "get_transport",
        []() -> py::object
        {
            throw std::runtime_error(
                "raccoon transport is not available in this build (mock driver bundle)");
        });
    m.def("shutdown_transport", []() {});
#endif
}
