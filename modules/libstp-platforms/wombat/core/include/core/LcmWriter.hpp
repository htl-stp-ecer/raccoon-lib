#ifndef VERSION_LCMWRITER_HPP
#define VERSION_LCMWRITER_HPP

#endif //VERSION_LCMWRITER_HPP

#include <exlcm/vector3f_t.hpp>
#include <exlcm/scalar_i8_t.hpp>
#include <exlcm/scalar_i32_t.hpp>
#include <exlcm/scalar_f_t.hpp>
#include "lcm/lcm-cpp.hpp"

#include <unordered_map>
#include <vector>
#include <string>
#include <cstring>

namespace platform::wombat::core
{

    class LcmDataWriter
    {
    public:
        static LcmDataWriter& instance()
        {
            static LcmDataWriter writer;
            return writer;
        }

        // Publish motor stop latch command. Non-zero -> stop/latch; 0 -> wake/enable.
        void setMotorStop(uint8_t port, int value);
        void setMotor(uint8_t port, int valueData);
        void setServo(uint8_t port, int valueData);
        void requestDataDump();
        void resetBemfCounters();

    private:
        explicit LcmDataWriter();
        ~LcmDataWriter() = default;

        lcm::LCM lcm;

        std::unordered_map<std::string, std::vector<uint8_t>> lastMsgs;

        template <typename Msg>
        bool publishIfChanged(const std::string& channel, const Msg& msg)
        {
            const auto* bytes = reinterpret_cast<const uint8_t*>(&msg);
            const size_t size = sizeof(Msg);

            auto& cached = lastMsgs[channel];
            if (cached.size() == size && std::memcmp(cached.data(), bytes, size) == 0)
            {
                return false;
            }

            cached.assign(bytes, bytes + size);
            lcm.publish(channel, &msg);
            return true;
        }
    };
}
