# transport_core provides the C++ SharedTransport singleton (the raccoon::Transport of the
# process, with the reliable-delivery events routed into the libstp log). Python uses the
# transport of the raccoon_transport package (`raccoon_transport.get_transport()`).
register_libstp_module(transport_core "FALSE" "foundation;raccoon::transport")
