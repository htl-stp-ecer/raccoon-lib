# transport_core provides the C++ SharedTransport singleton. Python bindings for
# it live in libstp-foundation so that user code can do `from raccoon.foundation
# import get_transport`. This keeps transport_core a pure C++ library and avoids
# the threading <-> foundation link cycle that would arise if foundation owned
# the SharedTransport sources directly. The iox2 → spdlog log bridge in
# src/iox2_log_bridge.cpp calls into foundation's logging::log /
# logging::is_enabled, so transport_core needs foundation's symbols at link
# time. foundation itself doesn't link transport_core back (only the
# libstp_foundation pybind module does), so this stays a one-way edge — no
# static-lib link cycle.
register_libstp_module(transport_core "FALSE"
                       "threading;raccoon::transport;foundation")
