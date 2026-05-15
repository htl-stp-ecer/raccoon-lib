"""Helpers for reading the current heading from odometry or localization.

Every ``LinearMotion`` / ``DiagonalMotion`` / ``SplineMotion`` config needs
an absolute ``target_heading_rad``. This module provides that value with the
following priority:

1. **Localization** (``robot.localization``) — IMU-fused world pose; the
   primary source whenever it is enabled. Tracks the robot's actual
   orientation in space, including manual rotations between steps.
2. **Fused odometry** (``robot.odometry``) — fallback when localization is
   not available. May go stale when the robot is moved by hand (wheel
   encoders see nothing), so it must not override localization.

When both are present and disagree by more than 0.02 rad a warning is
logged and localization still wins. Raises ``RuntimeError`` only when
*neither* source is available.
"""

from __future__ import annotations

import logging
import math
from typing import TYPE_CHECKING

_log = logging.getLogger(__name__)

if TYPE_CHECKING:
    from raccoon.robot.api import GenericRobot


_REQUIRED_MSG = (
    "Neither robot.odometry nor robot.localization is available. "
    "At least one must be enabled — motion steps need a heading source."
)


def get_world_heading_rad(robot: "GenericRobot") -> float:
    """Return the current heading in radians.

    Prefers ``robot.localization`` (IMU-fused world heading) so that manual
    rotations between steps are honoured. Falls back to ``robot.odometry``
    only when localization is unavailable.
    """
    odom = getattr(robot, "odometry", None)
    loc = getattr(robot, "localization", None)

    if odom is None and loc is None:
        raise RuntimeError(_REQUIRED_MSG)

    if loc is None:
        return float(odom.get_pose().heading)

    world_heading = float(loc.get_pose().heading)

    if odom is not None:
        odom_heading = float(odom.get_pose().heading)
        if abs(math.remainder(world_heading - odom_heading, 2.0 * math.pi)) > 0.02:
            _log.warning(
                "Heading sources diverge: localization=%.3f rad, odometry=%.3f rad "
                "(using localization — odometry may be stale from manual rotation or wheel slip)",
                world_heading,
                odom_heading,
            )

    return world_heading


def get_world_pose(robot: "GenericRobot"):
    """Return the current pose snapshot.

    Prefers ``robot.localization`` when available; falls back to
    ``robot.odometry``. Raises ``RuntimeError`` when neither is set.
    """
    loc = getattr(robot, "localization", None)
    if loc is not None:
        return loc.get_pose()
    odom = getattr(robot, "odometry", None)
    if odom is not None:
        return odom.get_pose()
    raise RuntimeError(_REQUIRED_MSG)


__all__ = ["get_world_heading_rad", "get_world_pose"]
