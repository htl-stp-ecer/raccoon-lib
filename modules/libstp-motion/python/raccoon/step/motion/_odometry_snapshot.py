"""Snapshot-based body-frame projection against absolute odometry.

The canonical ``robot.odometry`` is absolute and is never reset during motion.
A step that wants "distance traveled since I started" snapshots the pose at
``on_start`` and projects subsequent reads into that snapshot's body frame.

This mirrors ``LinearMotion::captureInitialPose`` / ``projectBodyFrame`` in
``linear_motion.cpp``.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from raccoon.robot.api import GenericRobot


@dataclass(frozen=True)
class PoseSnapshot:
    x: float
    y: float
    heading: float

    @classmethod
    def capture(cls, robot: "GenericRobot") -> "PoseSnapshot":
        pose = robot.odometry.get_pose()
        return cls(
            x=float(pose.position[0]),
            y=float(pose.position[1]),
            heading=float(pose.heading),
        )

    def project(self, robot: "GenericRobot") -> tuple[float, float, float]:
        """Return ``(forward, lateral, straight_line)`` in the snapshot body frame.

        Matches ``IOdometry::getDistanceFromOrigin`` convention:
            forward = +x in snapshot frame
            lateral = +y rotated to "right-positive"
            straight_line = unsigned euclidean distance
        """
        pose = robot.odometry.get_pose()
        dx = float(pose.position[0]) - self.x
        dy = float(pose.position[1]) - self.y
        cos_h = math.cos(self.heading)
        sin_h = math.sin(self.heading)
        forward = dx * cos_h + dy * sin_h
        lateral = -dx * sin_h + dy * cos_h
        straight = math.hypot(dx, dy)
        return forward, lateral, straight


__all__ = ["PoseSnapshot"]
