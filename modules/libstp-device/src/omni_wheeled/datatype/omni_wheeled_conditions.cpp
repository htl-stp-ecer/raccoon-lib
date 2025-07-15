//
// Created by tobias on 6/9/25.
//

#include "device/omni_wheeled/datatype/omni_wheeld_conditions.h"
#include "motion/differential_drive.h"

inline void libstp::device::omni_wheeled::datatype::ForwardDistanceConditionalResult::update(libstp::motion::DifferentialDriveState& state)
{
    auto [forwardDistance, strafeDistance] = state.computeDrivenDistance();
    updateResult(forwardDistance);
}

void libstp::device::omni_wheeled::datatype::SideDistanceConditionalResult::update(
    motion::DifferentialDriveState& state)
{
    auto [forwardDistance, strafeDistance] = state.computeDrivenDistance();
    updateResult(strafeDistance);
}
