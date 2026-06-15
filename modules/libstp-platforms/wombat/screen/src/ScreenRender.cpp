//
// Created by eternalRose on 12/18/25.
//

#include "hal/ScreenRender.hpp"

#include <raccoon/ui_messages.hpp>
#include <raccoon/Channels.h>
#include <raccoon/Transport.h>

#include <chrono>
#include <string>
#include <vector>

namespace
{
    std::string current_screen = "";

    // The raccoon transport lives here, not as a member of ScreenRender, so the
    // public ScreenRender class layout stays transport-free and identical across
    // platform bundles. Lazily created on first publish so merely constructing
    // the singleton never opens a transport.
    raccoon::Transport& screen_transport()
    {
        static raccoon::Transport transport = raccoon::Transport::create();
        return transport;
    }
}

void libstp::hal::screen_render::ScreenRender::setCurrentScreenSetting(std::string newScreen) {
    current_screen = std::move(newScreen);
}

libstp::hal::screen_render::ScreenRender::ScreenRender()
{
}


void libstp::hal::screen_render::ScreenRender::sendState(const std::string &jsonData) {
    if (current_screen != "") {
        raccoon::ui::ScreenRender msg{};
        msg.timestamp = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        msg.screen_name = current_screen;
        msg.entries = jsonData;
        std::vector<std::uint8_t> payload;
        msg.encode(payload);
        screen_transport().publishRaw(
            raccoon::Channels::SCREEN_RENDER,
            payload.data(),
            static_cast<int>(payload.size()));
    }
}
