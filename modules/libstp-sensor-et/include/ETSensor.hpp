//
// ET (electro-topographic) range finder sensor.
//

#pragma once

#include "hal/Analog.hpp"

namespace libstp::sensors::et {

    /**
     * Distance sensor wrapping a HAL analog input.
     *
     * Provides a named type for ET range finders so they can be
     * distinguished from generic analog sensors in robot definitions.
     */
    class ETSensor : public hal::analog::AnalogSensor
    {
    public:
        explicit ETSensor(int port);

        /** Read the raw analog value (convenience alias for read()). */
        int raw() const;
    };

} // namespace libstp::sensors::et
