#pragma once

namespace libstp::transport_core {

// Install a bridge that forwards iceoryx2's own log output through the
// libstp-foundation spdlog logger and clamps the iox2 global log level to
// Warn (unless the IOX2_LOG_LEVEL env var overrides it).
//
// Must be called BEFORE the first iceoryx2 type is touched (Config,
// NodeBuilder, set_logger, ...). iox2::set_logger() can only be called once.
// Safe to call repeatedly — second and later calls are no-ops.
//
// If spdlog hasn't been initialized yet at the time iox2 emits a message
// (early static init / module import), the bridge falls back to a minimal
// stderr write so the message isn't lost. Once logging::init() runs, all
// subsequent iox2 messages flow through spdlog (file + console sinks).
void install_iox2_log_bridge();

} // namespace libstp::transport_core
