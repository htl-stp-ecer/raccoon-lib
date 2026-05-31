//
// Created by eternalRose on 12/18/25.
//

#include "hal/ScreenRender.hpp"

#include <raccoon/ui_messages.hpp>
#include <raccoon/Channels.h>

#include <chrono>
#include <string>
#include <vector>


std::string current_screen = "";

void libstp::hal::screen_render::ScreenRender::setCurrentScreenSetting(std::string newScreen) {
    current_screen = newScreen;
}

libstp::hal::screen_render::ScreenRender::ScreenRender()
    : transport_(raccoon::Transport::create())
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
        transport_.publishRaw(
            raccoon::Channels::SCREEN_RENDER,
            payload.data(),
            static_cast<int>(payload.size()));
    }
}
