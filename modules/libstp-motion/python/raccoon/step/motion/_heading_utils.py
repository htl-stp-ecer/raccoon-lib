"""Helpers for reading the absolute world heading from localization.

Phase 4 of the absolute-motion plan removed every relative-heading code
path from the C++ motion classes — every ``LinearMotion`` /
``DiagonalMotion`` / ``SplineMotion`` config now needs an absolute
``target_heading_rad`` set by the Python caller. The world heading lives
in ``robot.localization.get_pose().heading`` (radians), so every motion
step that doesn't take an explicit user heading must read it from there.

This module centralises that read so the error message is consistent and
callers don't sprinkle ``getattr(robot, "localization", None)`` checks
across the codebase.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from raccoon.robot.api import GenericRobot


_REQUIRED_MSG = (
    "robot.localization is required (phase 4 of the absolute-motion plan). "
    "Enable it in your robot subclass — motion configs need an absolute "
    "target_heading_rad and there is no fallback path any more."
)


def get_world_heading_rad(robot: "GenericRobot") -> float:
    """Return the current absolute world heading in radians.

    Reads ``robot.localization.get_pose().heading``. Raises a clear
    ``RuntimeError`` when localization is not enabled — callers cannot
    construct a valid motion config without it.
    """
    loc = getattr(robot, "localization", None)
    if loc is None:
        raise RuntimeError(_REQUIRED_MSG)
    return float(loc.get_pose().heading)


def get_world_pose(robot: "GenericRobot"):
    """Return the current absolute world pose snapshot.

    Same precondition as :func:`get_world_heading_rad`; useful when
    callers need both position and heading in the same tick.
    """
    loc = getattr(robot, "localization", None)
    if loc is None:
        raise RuntimeError(_REQUIRED_MSG)
    return loc.get_pose()


__all__ = ["get_world_heading_rad", "get_world_pose"]
