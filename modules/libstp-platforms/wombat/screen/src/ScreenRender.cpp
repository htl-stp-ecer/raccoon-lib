//
// Created by eternalRose on 12/18/25.
//

#include "hal/ScreenRender.hpp"
#include <string>
#include <chrono>
#include <exlcm/screen_render_t.hpp>
#include <raccoon/Channels.h>


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
        exlcm::screen_render_t msg{};
        msg.timestamp = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        msg.screen_name = current_screen;
        msg.entries = jsonData;
        transport_.publish(raccoon::Channels::SCREEN_RENDER, msg);
    }
}
