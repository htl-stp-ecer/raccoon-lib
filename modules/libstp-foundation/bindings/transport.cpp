#include <pybind11/pybind11.h>
#include <pybind11/eval.h>  // py::exec — used for the atexit-registration snippet

#include <cstdint>
#include <cstdio>
#include <string>
#include <utility>
#ifndef _WIN32
#include <unistd.h>  // POSIX-only; the actual orderly exit goes through Python os._exit
#endif

#ifdef LIBSTP_HAS_TRANSPORT
#include "transport_core/shared_transport.hpp"
#include "raccoon/Transport.h"
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

    py::dict latency_to_dict(const raccoon::TransportStats::Latency& latency)
    {
        py::dict d;
        d["min_us"] = latency.minUs;
        d["max_us"] = latency.maxUs;
        d["avg_us"] = latency.avgUs;
        d["p99_us"] = latency.p99Us;
        d["count"] = latency.count;
        return d;
    }

    py::dict callback_to_dict(const raccoon::TransportStats::Callback& callback)
    {
        py::dict d;
        d["min_us"] = callback.minUs;
        d["max_us"] = callback.maxUs;
        d["avg_us"] = callback.avgUs;
        d["total_us"] = callback.totalUs;
        d["count"] = callback.count;
        return d;
    }

    py::dict spin_to_dict(const raccoon::TransportStats::Spin& spin)
    {
        py::dict d;
        d["min_us"] = spin.minUs;
        d["max_us"] = spin.maxUs;
        d["avg_us"] = spin.avgUs;
        d["count"] = spin.count;
        d["active_count"] = spin.activeCount;
        d["idle_count"] = spin.idleCount;
        return d;
    }

    py::dict stats_to_dict(const raccoon::TransportStats& stats, std::size_t active_subscriptions)
    {
        py::dict d;
        d["latency"] = latency_to_dict(stats.latency);
        d["callback"] = callback_to_dict(stats.callback);
        d["spin"] = spin_to_dict(stats.spin);
        d["publishes_deduplicated"] = stats.publishesDeduplicated;
        d["active_subscriptions"] = active_subscriptions;

        py::list channels;
        for (const auto& channel : stats.channels)
        {
            py::dict entry;
            entry["name"] = channel.name;
            entry["deliveries"] = channel.deliveries;
            entry["latency"] = latency_to_dict(channel.latency);
            entry["callback"] = callback_to_dict(channel.callback);
            channels.append(std::move(entry));
        }
        d["channels"] = std::move(channels);
        return d;
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

    // Emit a Python DeprecationWarning the first time a caller passes
    // reliable=True. Keep it once-per-channel-or-call by leaving stacklevel=2
    // so the warning points at the user's publish()/subscribe() call.
    static const auto warn_reliable_deprecated = [](const char* api) {
        PyErr_WarnEx(PyExc_DeprecationWarning,
                     "reliable=True is a no-op on the raccoon-transport backend — "
                     "drop the kwarg; the shared-memory backend doesn't lose packets.",
                     /*stacklevel=*/2);
        (void)api;
    };

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
                if (reliable) warn_reliable_deprecated("publish");
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
            "publish_raw",
            [](SharedTransport& self,
               const std::string& channel,
               py::bytes payload,
               bool reliable,
               bool retained,
               int retry_interval_ms,
               std::uint32_t max_retries)
            {
                if (reliable) warn_reliable_deprecated("publish_raw");
                const std::string data = payload;
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
                    throw std::runtime_error("failed to publish raw payload on channel " + channel);
                }
            },
            py::arg("channel"),
            py::arg("payload"),
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
                if (reliable) warn_reliable_deprecated("subscribe");
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
        .def(
            "get_and_reset_stats",
            [](SharedTransport& self)
            {
                py::gil_scoped_release release;
                const auto active_subscriptions = self.active_subscription_count();
                auto stats = self.get_and_reset_stats();
                py::gil_scoped_acquire acquire;
                return stats_to_dict(stats, active_subscriptions);
            })
        .def("shutdown", &SharedTransport::shutdown);

    m.def(
        "get_transport",
        []() -> SharedTransport& { return SharedTransport::instance(); },
        py::return_value_policy::reference);

    m.def("shutdown_transport", []() { SharedTransport::instance().shutdown(); });

    // Install a Python-level atexit hook that performs an orderly
    // shutdown and then short-circuits the rest of process exit via
    // os._exit(0). We register it from Python (via py::exec) rather
    // than as a `py::cpp_function` because the pybind11 callable path
    // didn't fire reliably during interpreter teardown on Pi 3B (the
    // lambda was effectively dropped — the hook never ran, the
    // segfault still happened). A plain Python lambda invoked through
    // the atexit registry works.
    //
    // Why both shutdown_transport() and _exit(0):
    //   1) shutdown_transport() stops the spin daemon and destroys the
    //      raccoon-transport backend handle while the surrounding process
    //      state is still alive.
    //   2) _exit(0) skips Python interpreter teardown and the C++
    //      static-destructor phase that follows. Bypassing that tail-end
    //      teardown keeps process exit on the controlled shutdown path.
    {
        py::dict scope;
        // Pass the just-registered shutdown_transport callable explicitly so
        // we don't trigger a re-import of the module while it's mid-init.
        scope["_shutdown"] = m.attr("shutdown_transport");
        py::exec(R"(
import atexit as _atexit
import os as _os
def _raccoon_finalize():
    try:
        _shutdown()
    except Exception:
        pass
    _os._exit(0)
_atexit.register(_raccoon_finalize)
)", scope);
    }
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
