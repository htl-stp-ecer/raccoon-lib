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

        template <raccoon::LcmMessage T>
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

        template <raccoon::LcmMessage T>
        std::uint64_t subscribe(
            const std::string& channel,
            std::function<void(const T&)> handler,
            const raccoon::SubscribeOptions& options = {})
        {
            return subscribe_raw(
                channel,
                [handler = std::move(handler)](const void* data, int data_len)
                {
                    T msg;
                    if (msg.decode(data, 0, data_len) >= 0)
                    {
                        handler(msg);
                    }
                },
                options.reliable,
                options.requestRetained);
        }

        void unsubscribe(std::uint64_t subscription_id);
        void shutdown();

        SharedTransport(const SharedTransport&) = delete;
        SharedTransport& operator=(const SharedTransport&) = delete;

    private:
        SharedTransport();
        ~SharedTransport();

        void ensure_started();

        raccoon::Transport transport_;
        std::mutex mu_;
        std::unordered_set<std::uint64_t> active_subscriptions_;
        std::unordered_map<std::uint64_t, RawHandler> handlers_;
        std::atomic<std::uint64_t> next_subscription_id_{1};
        libstp::threading::ThreadManager::DaemonHandle spin_daemon_;
    };
}
