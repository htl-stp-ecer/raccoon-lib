#include "foundation/chassis_control_context.hpp"

namespace libstp::foundation
{
    ChassisControlContext& ChassisControlContext::instance()
    {
        static ChassisControlContext s_instance;
        return s_instance;
    }
}
