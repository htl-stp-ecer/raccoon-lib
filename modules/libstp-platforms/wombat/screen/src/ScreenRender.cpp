//
// Created by eternalRose on 12/18/25.
//

#include "hal/ScreenRender.hpp"
#include <stdexcept>
#include <string>
#include <chrono>
#include <exlcm/screen_render_t.hpp>
#include "lcm/lcm-cpp.hpp"


std::string current_screen = "";

void libstp::hal::screen_render::ScreenRender::setCurrentScreenSetting(std::string newScreen) {
    current_screen = newScreen;
}

libstp::hal::screen_render::ScreenRender::ScreenRender() {
    if (!lcm.good()) {
        throw std::runtime_error("[LCM-Writer] Failed to initialize LCM");
    }
}


void libstp::hal::screen_render::ScreenRender::sendState(const std::string &jsonData) {
    if (current_screen != "") {
        exlcm::screen_render_t msg{};
        msg.timestamp = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        msg.screen_name = current_screen;
        msg.entries = jsonData;
        lcm.publish("libstp/screen_render", &msg);
    }
}

