#pragma once

#include <cmath>
#include <cstdint>

namespace libstp::map
{
    /// One line or wall segment in table coordinates (cm).
    /// Mirrors TableMapFileV1.lines[] from the web-ide ftmap schema, kept as
    /// a POD so the particle filter can keep `vector<MapSegment>` in a tight
    /// cache-friendly layout.
    struct MapSegment
    {
        enum class Kind : uint8_t { Line, Wall };

        Kind kind{Kind::Line};
        float startX{0.0f};
        float startY{0.0f};
        float endX{0.0f};
        float endY{0.0f};
        float widthCm{0.0f};

        /// Length of the segment in cm. Provided here so the binding can
        /// expose it as a Python read-only property without duplicating the
        /// formula on the Python side.
        float length() const noexcept
        {
            return std::hypot(endX - startX, endY - startY);
        }
    };
}
