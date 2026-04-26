"""Runtime middleware for the motion path executor.

A middleware hooks into the executor at well-defined points around segment
execution.  All hooks have default no-op implementations, so a middleware
only overrides what it cares about.

Public API:
    PathMiddleware              — protocol every middleware satisfies
    WorldCorrectionMiddleware   — closes the world-frame error loop across segments
    WorldPoseTracker            — the underlying expected-vs-actual tracker
"""

from __future__ import annotations

from typing import Optional, Protocol, runtime_checkable, TYPE_CHECKING

from ..ir import Segment, Correction
from .world_correction import (
    wrap_angle,
    WorldPoseTracker,
    WorldCorrectionMiddleware,
)

if TYPE_CHECKING:
    from raccoon.robot.api import GenericRobot


@runtime_checkable
class PathMiddleware(Protocol):
    """Hook into path execution at segment boundaries.

    Default implementations are no-ops; override only the hooks you need.
    """

    def on_path_start(self, robot: "GenericRobot") -> None:
        """Called once before the first segment starts."""

    def on_segment_start(
        self, seg: Segment, is_first: bool, robot: "GenericRobot",
    ) -> Optional[Correction]:
        """Called before each segment's motion is constructed.

        Return a ``Correction`` to influence the next motion's parameters,
        or ``None`` if no correction applies.  ``is_first`` is ``True`` for
        the first segment of the path (typically no correction).
        """

    def on_cold_start(self, seg: Segment, robot: "GenericRobot") -> None:
        """Called immediately before an odometry-resetting transition.

        Lets middleware snapshot pre-reset state (e.g. accumulating world
        position before odometry zeros out).
        """

    def on_segment_end(self, seg: Segment, robot: "GenericRobot") -> None:
        """Called after each segment completes (before the next begins)."""

    def on_path_end(self, robot: "GenericRobot") -> None:
        """Called once after the last segment finishes."""


__all__ = [
    "PathMiddleware",
    "WorldPoseTracker",
    "WorldCorrectionMiddleware",
    "wrap_angle",
]
