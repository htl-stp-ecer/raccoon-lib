#pragma once

#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

namespace raccoon::ui
{
    namespace detail
    {
        inline void require_available(std::size_t offset, std::size_t needed, std::size_t len)
        {
            if (offset > len || needed > len - offset)
            {
                throw std::runtime_error("screen message buffer underflow");
            }
        }

        inline void append_u32(std::vector<std::uint8_t>& out, std::uint32_t value)
        {
            out.push_back(static_cast<std::uint8_t>((value >> 24) & 0xFFu));
            out.push_back(static_cast<std::uint8_t>((value >> 16) & 0xFFu));
            out.push_back(static_cast<std::uint8_t>((value >> 8) & 0xFFu));
            out.push_back(static_cast<std::uint8_t>(value & 0xFFu));
        }

        inline void append_i64(std::vector<std::uint8_t>& out, std::int64_t value)
        {
            const auto raw = static_cast<std::uint64_t>(value);
            for (int shift = 56; shift >= 0; shift -= 8)
            {
                out.push_back(static_cast<std::uint8_t>((raw >> shift) & 0xFFu));
            }
        }

        inline std::uint32_t read_u32(const std::uint8_t* data, std::size_t len, std::size_t& offset)
        {
            require_available(offset, 4, len);
            const auto value = (static_cast<std::uint32_t>(data[offset]) << 24)
                               | (static_cast<std::uint32_t>(data[offset + 1]) << 16)
                               | (static_cast<std::uint32_t>(data[offset + 2]) << 8)
                               | static_cast<std::uint32_t>(data[offset + 3]);
            offset += 4;
            return value;
        }

        inline std::int64_t read_i64(const std::uint8_t* data, std::size_t len, std::size_t& offset)
        {
            require_available(offset, 8, len);
            std::uint64_t raw = 0;
            for (int i = 0; i < 8; ++i)
            {
                raw = (raw << 8) | static_cast<std::uint64_t>(data[offset + i]);
            }
            offset += 8;
            return static_cast<std::int64_t>(raw);
        }

        inline void append_string(std::vector<std::uint8_t>& out, const std::string& value)
        {
            if (value.size() > 0xFFFFFFFFu)
            {
                throw std::runtime_error("screen message string too large");
            }
            append_u32(out, static_cast<std::uint32_t>(value.size()));
            out.insert(out.end(), value.begin(), value.end());
        }

        inline std::string read_string(
            const std::uint8_t* data, std::size_t len, std::size_t& offset)
        {
            const auto size = read_u32(data, len, offset);
            require_available(offset, size, len);
            std::string value(
                reinterpret_cast<const char*>(data + offset),
                reinterpret_cast<const char*>(data + offset + size));
            offset += size;
            return value;
        }
    }

    struct ScreenRender
    {
        std::int64_t timestamp{};
        std::string screen_name{};
        std::string entries{};

        void encode(std::vector<std::uint8_t>& out) const
        {
            detail::append_i64(out, timestamp);
            detail::append_string(out, screen_name);
            detail::append_string(out, entries);
        }

        static ScreenRender decode(const std::uint8_t* data, std::size_t len)
        {
            std::size_t offset = 0;
            ScreenRender msg;
            msg.timestamp = detail::read_i64(data, len, offset);
            msg.screen_name = detail::read_string(data, len, offset);
            msg.entries = detail::read_string(data, len, offset);
            if (offset != len)
            {
                throw std::runtime_error("trailing bytes after ScreenRender payload");
            }
            return msg;
        }
    };

    struct ScreenRenderAnswer
    {
        std::int64_t timestamp{};
        std::string screen_name{};
        std::string value{};
        std::string reason{};

        void encode(std::vector<std::uint8_t>& out) const
        {
            detail::append_i64(out, timestamp);
            detail::append_string(out, screen_name);
            detail::append_string(out, value);
            detail::append_string(out, reason);
        }

        static ScreenRenderAnswer decode(const std::uint8_t* data, std::size_t len)
        {
            std::size_t offset = 0;
            ScreenRenderAnswer msg;
            msg.timestamp = detail::read_i64(data, len, offset);
            msg.screen_name = detail::read_string(data, len, offset);
            msg.value = detail::read_string(data, len, offset);
            msg.reason = detail::read_string(data, len, offset);
            if (offset != len)
            {
                throw std::runtime_error("trailing bytes after ScreenRenderAnswer payload");
            }
            return msg;
        }
    };
}
