#include "calibration/motor/utils/time_utils.hpp"
#include <chrono>
#include <thread>

namespace libstp::calibration::utils
{
    double getCurrentTime()
    {
        auto now = std::chrono::steady_clock::now();
        auto duration = now.time_since_epoch();
        return std::chrono::duration<double>(duration).count();
    }

    void sleepSeconds(double seconds)
    {
        std::this_thread::sleep_for(
            std::chrono::duration<double>(seconds)
        );
    }
}
