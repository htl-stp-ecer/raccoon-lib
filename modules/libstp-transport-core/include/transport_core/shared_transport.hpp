#pragma once

#include "raccoon/Transport.h"

#include <cstdint>
#include <mutex>
#include <string_view>

namespace libstp::transport_core
{
    /**
     * The raccoon::Transport of this process, started on first use. The transport
     * serves its subscriptions on a thread of its own (see
     * `raccoon::Transport::start_background_spin`), so the handlers run on that
     * thread. Retransmits and give-ups of reliable commands go to the libstp log.
     */
    class SharedTransport
    {
    public:
        using RawHandler = raccoon::Transport::RawHandler;
        using SubscriptionId = raccoon::Transport::SubscriptionId;

        static SharedTransport& instance();

        raccoon::PublishResult publish_raw(std::string_view channel, const void* data, std::size_t len,
                                           const raccoon::PublishOptions& options = {});

        template <raccoon::TransportMessage T>
        raccoon::PublishResult publish(std::string_view channel, const T& message,
                                       const raccoon::PublishOptions& options = {})
        {
            return started().publish<T>(channel, message, options);
        }

        SubscriptionId subscribe_raw(std::string_view channel, RawHandler handler);

        template <raccoon::TransportMessage T>
        SubscriptionId subscribe(std::string_view channel, std::function<void(const T&)> handler)
        {
            return started().subscribe<T>(channel, std::move(handler));
        }

        void unsubscribe(SubscriptionId id);
        [[nodiscard]] raccoon::TransportStats get_and_reset_stats();

        /// Stop the transport. A later call re-creates it.
        void shutdown();

        SharedTransport(const SharedTransport&) = delete;
        SharedTransport& operator=(const SharedTransport&) = delete;

    private:
        SharedTransport() = default;
        ~SharedTransport();

        raccoon::Transport& started();

        raccoon::Transport transport_;
        bool started_ = false;
        std::mutex mu_;
    };
}
