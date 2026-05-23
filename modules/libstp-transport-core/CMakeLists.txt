# transport_core provides the C++ SharedTransport singleton. Python bindings for
# it live in libstp-foundation so that user code can do `from raccoon.foundation
# import get_transport`. This keeps transport_core a pure C++ library and avoids
# the threading <-> foundation link cycle that would arise if foundation owned
# the SharedTransport sources directly.
register_libstp_module(transport_core "FALSE" "threading;raccoon::transport")
