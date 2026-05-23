#include <pybind11/pybind11.h>

#include "transport_core/shared_transport.hpp"

#include <string>
#include <utility>

namespace py = pybind11;

namespace
{
    struct PySubscription
    {
        std::uint64_t id{};
    };

    py::bytes encode_message(py::object message)
    {
        return py::reinterpret_borrow<py::bytes>(message.attr("encode")());
    }
}

PYBIND11_MODULE(transport, m)
{
    m.doc() = "Native shared raccoon transport backed by ThreadManager";

    py::class_<PySubscription>(m, "Subscription")
        .def_property_readonly("subscription_id", [](const PySubscription& sub) { return sub.id; });

    py::class_<
        libstp::transport_core::SharedTransport,
        std::unique_ptr<libstp::transport_core::SharedTransport, py::nodelete>>(
        m,
        "SharedTransport")
        .def(
            "publish",
            [](libstp::transport_core::SharedTransport& self,
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
            [](libstp::transport_core::SharedTransport& self,
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
            [](libstp::transport_core::SharedTransport& self, const PySubscription& sub)
            {
                self.unsubscribe(sub.id);
            },
            py::arg("subscription"))
        .def("shutdown", &libstp::transport_core::SharedTransport::shutdown);

    m.def(
        "get_transport",
        []() -> libstp::transport_core::SharedTransport&
        {
            return libstp::transport_core::SharedTransport::instance();
        },
        py::return_value_policy::reference);

    m.def(
        "shutdown_transport",
        []()
        {
            libstp::transport_core::SharedTransport::instance().shutdown();
        });
}
