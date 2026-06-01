#pragma once

#include "raccoon/Transport.h"
#include "threading/thread_manager.hpp"

#include <atomic>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace libstp::transport_core
{
    class SharedTransport
    {
    public:
        using RawHandler = std::function<void(const void*, int)>;

        static SharedTransport& instance();

        bool publish_raw(
            const std::string& channel,
            const void* data,
            int data_len,
            bool reliable,
            bool retained,
            int retry_interval_ms,
            std::uint32_t max_retries);

        template <raccoon::TransportMessage T>
        bool publish(
            const std::string& channel,
            const T& message,
            const raccoon::PublishOptions& options = {})
        {
            ensure_started();
            return transport_.publish<T>(channel, message, options);
        }

        std::uint64_t subscribe_raw(
            const std::string& channel,
            RawHandler handler,
            bool reliable,
            bool request_retained);

        template <raccoon::TransportMessage T>
        std::uint64_t subscribe(
            const std::string& channel,
            std::function<void(const T&)> handler,
            const raccoon::SubscribeOptions& options = {})
        {
            // We forward `options.reliable` as a no-op (kept for binary
            // compatibility with the LCM-era callers). Silence the
            // template-instantiation cascade of deprecation warnings; the
            // attribute still fires for user code that writes
            // `opts.reliable = true` outside this header.
#if defined(__GNUC__) || defined(__clang__)
#  pragma GCC diagnostic push
#  pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif
            const bool reliable_passthrough = options.reliable;
#if defined(__GNUC__) || defined(__clang__)
#  pragma GCC diagnostic pop
#endif
            return subscribe_raw(
                channel,
                [handler = std::move(handler)](const void* data, int data_len)
                {
                    T msg;
                    if (msg.decode(static_cast<const uint8_t*>(data), data_len) >= 0)
                    {
                        handler(msg);
                    }
                },
                reliable_passthrough,
                options.requestRetained);
        }

        void unsubscribe(std::uint64_t subscription_id);
        [[nodiscard]] raccoon::TransportStats get_and_reset_stats();
        [[nodiscard]] std::size_t active_subscription_count() const;
        void shutdown();

        SharedTransport(const SharedTransport&) = delete;
        SharedTransport& operator=(const SharedTransport&) = delete;

    private:
        SharedTransport();
        ~SharedTransport();

        void ensure_started();

        raccoon::Transport transport_;
        mutable std::mutex mu_;
        std::unordered_map<std::string, std::unordered_map<std::uint64_t, RawHandler>> channel_handlers_;
        std::unordered_map<std::uint64_t, std::string> subscription_channels_;
        std::unordered_set<std::string> subscribed_channels_;
        std::atomic<std::uint64_t> next_subscription_id_{1};
        libstp::threading::ThreadManager::DaemonHandle spin_daemon_;
    };
}
