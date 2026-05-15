"""Helpers for reading the current world heading and pose from odometry.

Every ``LinearMotion`` / ``DiagonalMotion`` / ``SplineMotion`` config needs
an absolute ``target_heading_rad``. With the platform-owned odometry model
the canonical ``robot.odometry`` is always live (the platform's IOdometry
implementation), so these helpers are now thin wrappers around it.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from raccoon.robot.api import GenericRobot


def get_world_heading_rad(robot: "GenericRobot") -> float:
    """Return the current heading in radians from ``robot.odometry``."""
    return float(robot.odometry.get_pose().heading)


def get_world_pose(robot: "GenericRobot"):
    """Return the current pose snapshot from ``robot.odometry``."""
    return robot.odometry.get_pose()


__all__ = ["get_world_heading_rad", "get_world_pose"]
