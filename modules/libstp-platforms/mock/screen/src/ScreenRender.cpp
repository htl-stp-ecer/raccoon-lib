#include "hal/ScreenRender.hpp"

libstp::hal::screen_render::ScreenRender::ScreenRender()
    : transport_()
{
    // Mock: leave transport uninitialised (no LCM) — sendState is a no-op.
}

void libstp::hal::screen_render::ScreenRender::setCurrentScreenSetting(std::string newScreen)
{
    screenName = std::move(newScreen);
}

void libstp::hal::screen_render::ScreenRender::sendState(const std::string& jsonData)
{
    // no-op on mock — no raccoon transport publishing
}
