//
// Created by tobias on 1/12/25.
//

#pragma once
#include "datatype/functions.hpp"

namespace libstp::device::omni_wheeled::datatype {
    libstp::datatype::ConditionalFunction forForwardDistance(const float& distanceCm);
    libstp::datatype::ConditionalFunction forSideDistance(const float& distanceCm);
}
