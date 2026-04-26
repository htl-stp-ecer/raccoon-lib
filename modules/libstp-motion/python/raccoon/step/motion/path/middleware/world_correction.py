"""World-frame error correction middleware.

Tracks the difference between the geometrically-expected pose (where the
robot *should* be after each segment, based on segment parameters) and the
actually-measured pose (from odometry + absolute heading), and produces
``Correction`` hints that the motion factory consumes when constructing the
next motion.

The tracker has to deal with one quirk: cold-start transitions reset the
odometry counter to 0.  Position is therefore accumulated across resets in
``_accumulated_x/y`` rather than read directly from ``odometry.get_pose()``.
Heading uses ``getAbsoluteHeading()`` which is reset-immune.
"""

from __future__ import annotations

import logging
import math
from typing import Optional, TYPE_CHECKING

from raccoon.motion import LinearAxis

from ..ir import Segment, Correction

if TYPE_CHECKING:
    from raccoon.robot.api import GenericRobot

_log = logging.getLogger(__name__)


def wrap_angle(a: float) -> float:
    """Wrap angle to ``[-pi, pi]``."""
    return (a + math.pi) % (2 * math.pi) - math.pi


class WorldPoseTracker:
    """Track expected vs actual world-frame pose across segment transitions.

    Position is tracked by accumulating ``getPose().position`` snapshots
    across odometry resets (cold starts).  Heading uses
    ``getAbsoluteHeading()`` which is reset-immune.
    """

    MAX_DISTANCE_CORRECTION_M = 0.03
    MAX_HEADING_CORRECTION_RAD = 0.087  # ~5 deg
    MAX_TURN_CORRECTION_RAD = 0.087     # ~5 deg

    def __init__(self, start_heading: float) -> None:
        # Expected cumulative world pose (from geometry)
        self.expected_x = 0.0
        self.expected_y = 0.0
        self.expected_heading = start_heading

        # Accumulated actual world position across odometry resets
        self._accumulated_x = 0.0
        self._accumulated_y = 0.0

    # -- Expected pose projection -----------------------------------------

    def advance_expected(self, seg: Segment) -> None:
        """Project expected pose forward by the completed segment's geometry."""
        h = self.expected_heading

        if seg.kind == "linear":
            dist = seg.distance_m
            if dist is None:
                return  # condition-based, can't predict
            if seg.axis == LinearAxis.Forward:
                self.expected_x += dist * math.cos(h)
                self.expected_y += dist * math.sin(h)
            else:  # Lateral
                self.expected_x += dist * -math.sin(h)
                self.expected_y += dist * math.cos(h)

        elif seg.kind == "turn":
            angle = seg.angle_rad
            if angle is None:
                return
            self.expected_heading = wrap_angle(h + angle)

        elif seg.kind == "follow_line":
            # Approximate as straight-forward motion; heading unchanged.
            dist = seg.distance_m
            if dist is None:
                return  # condition-based, can't predict
            self.expected_x += dist * math.cos(h)
            self.expected_y += dist * math.sin(h)

        elif seg.kind == "spline":
            pass  # endpoint unknown in world frame; tracker reset after spline

        elif seg.kind == "arc":
            angle = seg.arc_angle_rad
            if angle is None:
                return
            r = seg.radius_m or 0.0
            # Arc geometry: displacement in body frame then rotate to world.
            # Sign convention: dy_body > 0 = LEFT (world +y = left).
            # arc_angle_rad: positive = CCW (left), negative = CW (right).
            if seg.lateral:
                # Lateral arc: primary motion is lateral (y), forward is drift (x)
                dx_body = math.copysign(r * (1 - math.cos(angle)), angle)
                dy_body = r * math.sin(abs(angle))
            else:
                # Forward arc: primary motion is forward (x), lateral is drift (y)
                dx_body = r * math.sin(abs(angle))
                dy_body = math.copysign(r * (1 - math.cos(angle)), angle)
            cos_h = math.cos(h)
            sin_h = math.sin(h)
            self.expected_x += dx_body * cos_h - dy_body * sin_h
            self.expected_y += dx_body * sin_h + dy_body * cos_h
            self.expected_heading = wrap_angle(h + angle)

    # -- Actual pose reading ----------------------------------------------

    def snapshot_before_cold_start(self, robot: "GenericRobot") -> None:
        """Accumulate world position before an odometry reset (cold start)."""
        pose = robot.odometry.get_pose()
        self._accumulated_x += float(pose.position[0])
        self._accumulated_y += float(pose.position[1])

    def get_actual_xy(self, robot: "GenericRobot") -> tuple[float, float]:
        """Get actual world-frame position (accumulated across resets)."""
        pose = robot.odometry.get_pose()
        x = self._accumulated_x + float(pose.position[0])
        y = self._accumulated_y + float(pose.position[1])
        return x, y

    def get_actual_heading(self, robot: "GenericRobot") -> float:
        """Get actual absolute heading."""
        return robot.odometry.get_absolute_heading()

    # -- Reset expected to actual (for condition-based segments) -----------

    def reset_expected_to_actual(self, robot: "GenericRobot") -> None:
        """Set expected = actual (when we can't predict the endpoint)."""
        self.expected_x, self.expected_y = self.get_actual_xy(robot)
        self.expected_heading = self.get_actual_heading(robot)

    # -- Correction computation -------------------------------------------

    def compute_correction(
        self, robot: "GenericRobot", next_seg: Segment,
    ) -> Correction:
        """Compute parameter adjustments for the next segment."""
        actual_x, actual_y = self.get_actual_xy(robot)
        actual_heading = self.get_actual_heading(robot)

        # Position error in world frame
        dx = actual_x - self.expected_x
        dy = actual_y - self.expected_y

        # Decompose into along-track / cross-track relative to expected heading
        cos_h = math.cos(self.expected_heading)
        sin_h = math.sin(self.expected_heading)
        along_track_err = dx * cos_h + dy * sin_h   # positive = overshot
        cross_track_err = -dx * sin_h + dy * cos_h  # positive = drifted left

        # Heading error
        heading_err = wrap_angle(actual_heading - self.expected_heading)

        _log.debug(
            "WorldPoseTracker: expected=(%.4f, %.4f, %.1f°) actual=(%.4f, %.4f, %.1f°) "
            "along=%.4fm cross=%.4fm hdg=%.2f°",
            self.expected_x, self.expected_y, math.degrees(self.expected_heading),
            actual_x, actual_y, math.degrees(actual_heading),
            along_track_err, cross_track_err, math.degrees(heading_err),
        )

        correction = Correction()

        if next_seg.kind == "linear":
            # Along-track: adjust distance
            if next_seg.distance_m is not None:
                adj = max(-self.MAX_DISTANCE_CORRECTION_M,
                          min(self.MAX_DISTANCE_CORRECTION_M, along_track_err))
                correction.distance_adjust_m = adj

            # Cross-track: bias heading to steer back
            remaining = abs(next_seg.distance_m or 0.3)
            remaining = max(remaining, 0.05)  # avoid division by tiny number
            heading_bias = math.atan2(-cross_track_err, remaining)
            heading_bias = max(-self.MAX_HEADING_CORRECTION_RAD,
                               min(self.MAX_HEADING_CORRECTION_RAD, heading_bias))

            # Target heading = expected heading + cross-track bias
            correction.heading_target_rad = self.expected_heading + heading_bias

        elif next_seg.kind in ("turn", "arc"):
            # Correct heading error by adjusting turn/arc angle
            adj = max(-self.MAX_TURN_CORRECTION_RAD,
                      min(self.MAX_TURN_CORRECTION_RAD, heading_err))
            correction.angle_adjust_rad = adj

        _log.debug(
            "WorldPoseTracker: correction dist=%.4fm hdg=%s angle=%.2f°",
            correction.distance_adjust_m,
            f"{math.degrees(correction.heading_target_rad):.1f}°"
            if correction.heading_target_rad is not None else "None",
            math.degrees(correction.angle_adjust_rad),
        )

        return correction


class WorldCorrectionMiddleware:
    """Closes the world-frame error loop across segment transitions.

    Maintains an internal ``WorldPoseTracker`` initialized at path start,
    advances it after each completed segment, and emits a ``Correction``
    hint before each non-first segment so the motion factory can offset
    the next motion's distance/heading/angle parameters.
    """

    def __init__(self) -> None:
        self._tracker: Optional[WorldPoseTracker] = None
        self._is_first: bool = True

    def on_path_start(self, robot: "GenericRobot") -> None:
        # Initialize after the first odometry reset, which the executor
        # performs before calling on_path_start for the very first segment.
        self._tracker = WorldPoseTracker(
            start_heading=robot.odometry.get_absolute_heading(),
        )
        self._is_first = True

    def on_segment_start(
        self, seg: Segment, is_first: bool, robot: "GenericRobot",
    ) -> Optional[Correction]:
        if is_first or self._tracker is None:
            return None
        return self._tracker.compute_correction(robot, seg)

    def on_cold_start(self, seg: Segment, robot: "GenericRobot") -> None:
        if self._tracker is not None:
            self._tracker.snapshot_before_cold_start(robot)

    def on_segment_end(self, seg: Segment, robot: "GenericRobot") -> None:
        if self._tracker is None:
            return
        self._tracker.advance_expected(seg)
        # Condition-based and opaque (follow_line/spline) segments may have
        # deviated from the ideal predicted geometry, so reset the expected
        # pose to whatever the robot actually achieved rather than stacking
        # corrections on stale predictions.
        was_condition_based = (
            not seg.has_known_endpoint
            or (seg.condition is not None and not seg.has_known_endpoint)
        )
        was_opaque = seg.kind in ("follow_line", "spline")
        if was_condition_based or was_opaque:
            self._tracker.reset_expected_to_actual(robot)

    def on_path_end(self, robot: "GenericRobot") -> None:
        pass

    @property
    def tracker(self) -> Optional[WorldPoseTracker]:
        """Expose the underlying tracker for diagnostics/inspection."""
        return self._tracker
