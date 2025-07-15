//
// Created by tobias on 6/8/25.
//

#pragma once
#include <stdexcept>

namespace libstp::drive::base
{
    struct AccelLimits {
        float forward;
        float strafe;
        float angular;
    };
    
    class DriveBase
    {
    public:
        virtual ~DriveBase() = default;
        virtual AccelLimits getAccelLimits() const;
    };
}
