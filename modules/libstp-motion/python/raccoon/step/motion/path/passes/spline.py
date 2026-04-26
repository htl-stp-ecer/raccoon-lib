"""Spline-conversion pass — translate a linear/turn sequence into a SplinePath.

Unlike merge / corner-cut (pure node-list transforms), this pass *replaces*
the entire path with a single spline-driven step.  It is therefore a
"terminal" transform — typically the last entry in a pipeline that opts in
to spline mode.
"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

from raccoon.motion import LinearAxis

from ..ir import PathNode, Segment, SideAction

if TYPE_CHECKING:
    from ...spline_path import SplinePath


def segments_to_spline_waypoints(
    segments: list[Segment],
) -> list[tuple[float, float]]:
    """Compute ``(forward_cm, left_cm)`` waypoints from a linear/turn sequence.

    Turns update the running heading but do not produce waypoints; each
    linear segment appends its endpoint.  The robot's start position
    ``(0, 0)`` is *not* included — it is the implicit origin for ``spline()``.

    Coordinate system: +forward is the robot's initial heading, +left is
    90° CCW of that (same convention as ``spline()`` waypoints).
    """
    x = 0.0  # meters, forward from start
    y = 0.0  # meters, left from start
    heading = 0.0  # radians, CCW positive

    waypoints: list[tuple[float, float]] = []

    for seg in segments:
        if seg.kind == "linear":
            d = seg.distance_m or 0.0
            if seg.axis == LinearAxis.Forward:
                x += d * math.cos(heading)
                y += d * math.sin(heading)
            else:  # Lateral
                x += d * (-math.sin(heading))
                y += d * math.cos(heading)
            waypoints.append((x * 100.0, y * 100.0))
        elif seg.kind == "turn":
            heading += seg.angle_rad or 0.0
        # Arc segments are rejected before this function is called.

    return waypoints


def build_spline_step(nodes: list[PathNode | None]) -> "SplinePath":
    """Build a SplinePath from a fully-splinifiable node list.

    Raises ``ValueError`` for any node that cannot be represented as a
    waypoint: deferred placeholders, side actions, arc segments, condition-
    based segments, or segments with unknown endpoints.  A minimum of 2
    linear segments is required so the spline has at least 2 explicit
    waypoints.
    """
    from ...spline_path import SplinePath  # deferred to avoid circular import

    for node in nodes:
        if node is None:
            msg = (
                "smooth_path(spline=True) cannot contain Defer steps — "
                "waypoints must be fully known at construction time"
            )
            raise ValueError(msg)
        if isinstance(node, SideAction):
            msg = (
                "smooth_path(spline=True) cannot contain side actions "
                "(background(), Run, or non-drive steps)"
            )
            raise ValueError(msg)
        if isinstance(node, Segment):
            if node.condition is not None:
                msg = (
                    "smooth_path(spline=True) cannot contain condition-based "
                    "segments (.until()) — endpoint must be known"
                )
                raise ValueError(msg)
            if not node.has_known_endpoint:
                msg = "smooth_path(spline=True) requires all segments to have " "known endpoints"
                raise ValueError(msg)
            if node.kind == "arc":
                msg = (
                    "smooth_path(spline=True) cannot contain arc segments — "
                    "use corner_cut_cm instead, or remove the arc"
                )
                raise ValueError(msg)
            if node.kind in ("follow_line", "spline"):
                msg = (
                    f"smooth_path(spline=True) cannot contain "
                    f"{node.kind} segments — endpoint must be a simple "
                    f"linear/turn sequence"
                )
                raise ValueError(msg)

    segs = [n for n in nodes if isinstance(n, Segment)]
    linear_count = sum(1 for s in segs if s.kind == "linear")
    if linear_count < 2:
        msg = (
            "smooth_path(spline=True) requires at least 2 linear segments "
            f"to form a valid spline (found {linear_count})"
        )
        raise ValueError(msg)

    waypoints = segments_to_spline_waypoints(segs)
    speed = min((s.speed_scale for s in segs if s.kind == "linear"), default=1.0)
    return SplinePath(waypoints, speed=speed)
