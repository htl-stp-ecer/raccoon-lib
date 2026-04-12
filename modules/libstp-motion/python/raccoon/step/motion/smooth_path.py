"""Smooth multi-segment motion path with zero-stop transitions.

Supports bare motion steps, nested ``seq()``, ``parallel()`` with a motion
spine, ``background()`` steps, and ``Run`` actions.  Composite steps are
flattened into a linear path of motion segments and side actions at
construction time, with deferred steps resolved at runtime.
"""

import asyncio
import logging
import math
from dataclasses import dataclass, field, replace
from typing import TYPE_CHECKING, Optional, Union

from raccoon.motion import (
    LinearMotion, LinearMotionConfig, LinearAxis,
    TurnMotion, TurnConfig,
    ArcMotion, ArcMotionConfig,
)

from .. import Step, dsl
from ..condition import StopCondition
from ..logic.defer import Defer
from .motion_step import MotionStep

if TYPE_CHECKING:
    from raccoon.robot.api import GenericRobot


# ---------------------------------------------------------------------------
# Segment descriptor
# ---------------------------------------------------------------------------

@dataclass
class _Segment:
    """Internal representation of one motion segment extracted from a Step."""

    kind: str  # "linear" | "turn" | "arc"
    # Linear params
    axis: Optional[LinearAxis] = None
    sign: float = 1.0
    distance_m: Optional[float] = None  # None = condition-only (sentinel)
    speed_scale: float = 1.0
    heading_deg: Optional[float] = None
    # Turn params
    angle_rad: Optional[float] = None
    # Arc params
    radius_m: Optional[float] = None
    arc_angle_rad: Optional[float] = None
    lateral: bool = False
    # Common
    condition: Optional[StopCondition] = None
    has_known_endpoint: bool = True

    _SENTINEL_DISTANCE_M = 100.0


@dataclass
class _SideAction:
    """A non-motion step to execute at a transition point in the path."""

    step: Step
    is_background: bool  # True = fire-and-forget async task; False = await inline


# Union type for flattened path nodes
_PathNode = Union[_Segment, _SideAction]

_log = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# World-frame error correction
# ---------------------------------------------------------------------------

@dataclass
class _Correction:
    """Parameter adjustments for the next segment to correct accumulated error."""

    distance_adjust_m: float = 0.0
    """Subtract from linear distance target (positive = overshot, drive less)."""

    heading_target_rad: Optional[float] = None
    """Absolute heading to hold during the next linear segment (includes cross-track bias)."""

    angle_adjust_rad: float = 0.0
    """Subtract from turn/arc angle target (positive = over-rotated, turn less)."""


def _wrap_angle(a: float) -> float:
    """Wrap angle to [-pi, pi]."""
    return (a + math.pi) % (2 * math.pi) - math.pi


class _WorldPoseTracker:
    """Track expected vs actual world-frame pose across segment transitions.

    Position is tracked by accumulating ``getPose().position`` snapshots
    across odometry resets (cold starts).  Heading uses
    ``getAbsoluteHeading()`` which is reset-immune.
    """

    _MAX_DISTANCE_CORRECTION_M = 0.03
    _MAX_HEADING_CORRECTION_RAD = 0.087  # ~5 deg
    _MAX_TURN_CORRECTION_RAD = 0.087     # ~5 deg

    def __init__(self, start_heading: float) -> None:
        # Expected cumulative world pose (from geometry)
        self.expected_x = 0.0
        self.expected_y = 0.0
        self.expected_heading = start_heading

        # Accumulated actual world position across odometry resets
        self._accumulated_x = 0.0
        self._accumulated_y = 0.0

    # -- Expected pose projection -----------------------------------------

    def advance_expected(self, seg: "_Segment") -> None:
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
            self.expected_heading = _wrap_angle(h + angle)

        elif seg.kind == "arc":
            angle = seg.arc_angle_rad
            if angle is None:
                return
            r = seg.radius_m or 0.0
            # Arc geometry: displacement in body frame then rotate to world.
            #
            # Sign convention: dy_body > 0 = LEFT (the WorldPoseTracker
            # world-frame convention: +y = left).  arc_angle_rad is signed:
            # positive = CCW (left turn), negative = CW (right turn).
            #
            # For a forward arc turning CW (angle < 0):
            #   - Primary forward motion: r·sin(|angle|)  (always positive)
            #   - Lateral drift: negative (RIGHT) for CW, positive (LEFT) for CCW
            # Same logic applies to lateral arcs with forward/lateral swapped.
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
            self.expected_heading = _wrap_angle(h + angle)

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
        self, robot: "GenericRobot", next_seg: "_Segment",
    ) -> _Correction:
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
        heading_err = _wrap_angle(actual_heading - self.expected_heading)

        _log.debug(
            "WorldPoseTracker: expected=(%.4f, %.4f, %.1f°) actual=(%.4f, %.4f, %.1f°) "
            "along=%.4fm cross=%.4fm hdg=%.2f°",
            self.expected_x, self.expected_y, math.degrees(self.expected_heading),
            actual_x, actual_y, math.degrees(actual_heading),
            along_track_err, cross_track_err, math.degrees(heading_err),
        )

        correction = _Correction()

        if next_seg.kind == "linear":
            # Along-track: adjust distance
            if next_seg.distance_m is not None:
                adj = max(-self._MAX_DISTANCE_CORRECTION_M,
                          min(self._MAX_DISTANCE_CORRECTION_M, along_track_err))
                correction.distance_adjust_m = adj

            # Cross-track: bias heading to steer back
            remaining = abs(next_seg.distance_m or 0.3)
            remaining = max(remaining, 0.05)  # avoid division by tiny number
            heading_bias = math.atan2(-cross_track_err, remaining)
            heading_bias = max(-self._MAX_HEADING_CORRECTION_RAD,
                               min(self._MAX_HEADING_CORRECTION_RAD, heading_bias))

            # Target heading = expected heading + cross-track bias
            correction.heading_target_rad = self.expected_heading + heading_bias

        elif next_seg.kind in ("turn", "arc"):
            # Correct heading error by adjusting turn/arc angle
            adj = max(-self._MAX_TURN_CORRECTION_RAD,
                      min(self._MAX_TURN_CORRECTION_RAD, heading_err))
            correction.angle_adjust_rad = adj

        _log.debug(
            "WorldPoseTracker: correction dist=%.4fm hdg=%s angle=%.2f°",
            correction.distance_adjust_m,
            f"{math.degrees(correction.heading_target_rad):.1f}°"
            if correction.heading_target_rad is not None else "None",
            math.degrees(correction.angle_adjust_rad),
        )

        return correction


# ---------------------------------------------------------------------------
# Segment extraction
# ---------------------------------------------------------------------------

def _extract_segment(step: Step) -> _Segment:
    """Extract motion parameters from a resolved step into a _Segment."""
    # Import here to avoid circular imports at module level
    from .drive import _ConditionalDrive
    from .turn import _ConditionalTurn
    from .arc import Arc

    if isinstance(step, _ConditionalDrive):
        cm = step._cm
        return _Segment(
            kind="linear",
            axis=step._axis,
            sign=step._sign,
            distance_m=step._sign * cm / 100.0 if cm is not None else None,
            speed_scale=step._speed,
            heading_deg=step._heading_deg,
            condition=step._until,
            has_known_endpoint=cm is not None,
        )

    if isinstance(step, _ConditionalTurn):
        degrees = step._degrees
        return _Segment(
            kind="turn",
            sign=step._sign,
            angle_rad=step._sign * math.radians(degrees) if degrees is not None else None,
            speed_scale=step._speed,
            condition=step._until,
            has_known_endpoint=degrees is not None,
        )

    if isinstance(step, Arc):
        cfg = step.config
        return _Segment(
            kind="arc",
            radius_m=cfg.radius_m,
            arc_angle_rad=cfg.arc_angle_rad,
            speed_scale=cfg.speed_scale,
            lateral=cfg.lateral,
            has_known_endpoint=True,
        )

    raise TypeError(
        f"smooth_path() does not support {type(step).__name__} as a motion "
        f"step. Only drive, turn, and arc steps can form the motion spine."
    )


def _resolve_step(step) -> Step:
    """Resolve a builder to a concrete Step instance."""
    if hasattr(step, "resolve"):
        return step.resolve()
    return step


def _is_same_type(a: _Segment, b: _Segment) -> bool:
    """Check if two segments can use warm-start (same motion type)."""
    if a.kind != b.kind:
        return False
    if a.kind == "linear":
        return a.axis == b.axis
    return True


# ---------------------------------------------------------------------------
# Recursive step flattening
# ---------------------------------------------------------------------------

def _flatten_one(
    step,
    nodes: list[Optional[_PathNode]],
    deferred: list[tuple[int, Defer]],
) -> None:
    """Recursively flatten *step* into path nodes.

    Appends to *nodes* in-place.  Deferred steps become ``None``
    placeholders with entries in *deferred* for runtime resolution.
    """
    from ..sequential import Sequential
    from ..parallel import Parallel
    from ..logic.background import Background
    from ..logic.defer import Run

    # 1. Defer — placeholder for runtime resolution
    if isinstance(step, Defer):
        deferred.append((len(nodes), step))
        nodes.append(None)
        return

    # 2. Resolve builder (e.g., drive_forward(30) returns a builder)
    step = _resolve_step(step)

    # 3. Sequential — flatten children recursively
    if isinstance(step, Sequential):
        for child in step.steps:
            _flatten_one(child, nodes, deferred)
        return

    # 4. Parallel — find motion spine, side-effect branches
    if isinstance(step, Parallel):
        _flatten_parallel(step, nodes, deferred)
        return

    # 5. Background — non-blocking side action
    if isinstance(step, Background):
        nodes.append(_SideAction(step=step._step, is_background=True))
        return

    # 6. Run — inline side action (quick callable)
    if isinstance(step, Run):
        nodes.append(_SideAction(step=step, is_background=False))
        return

    # 7. Try to extract as a motion segment
    try:
        seg = _extract_segment(step)
        nodes.append(seg)
        return
    except TypeError:
        pass

    # 8. Non-motion step — check if it uses the drive resource
    resources = step.collected_resources()
    if "drive" in resources:
        raise TypeError(
            f"smooth_path() does not support {type(step).__name__} — "
            f"it uses the drive resource but is not a supported motion step. "
            f"Only drive, turn, and arc steps can form the motion spine."
        )

    # 9. Non-drive step — treat as inline side action
    nodes.append(_SideAction(step=step, is_background=False))


def _flatten_parallel(
    par,
    nodes: list[Optional[_PathNode]],
    deferred: list[tuple[int, Defer]],
) -> None:
    """Flatten a Parallel step, identifying the motion spine branch."""
    from ..sequential import Sequential

    spine_idx = None
    for i, branch in enumerate(par.steps):
        branch_resources = branch.collected_resources()
        if "drive" in branch_resources:
            if spine_idx is not None:
                raise TypeError(
                    "smooth_path(): parallel() has multiple branches using "
                    "the drive resource — only one motion spine is allowed"
                )
            spine_idx = i

    if spine_idx is None:
        # No motion branch — entire parallel is a side-effect
        nodes.append(_SideAction(step=par, is_background=False))
        return

    # Side-effect branches: launch as background tasks at this point
    for i, branch in enumerate(par.steps):
        if i != spine_idx:
            nodes.append(_SideAction(step=branch, is_background=True))

    # Flatten the motion spine branch
    spine_branch = par.steps[spine_idx]
    if isinstance(spine_branch, Sequential):
        for child in spine_branch.steps:
            _flatten_one(child, nodes, deferred)
    else:
        _flatten_one(spine_branch, nodes, deferred)


def _flatten_steps(
    steps: list,
) -> tuple[list[Optional[_PathNode]], list[tuple[int, Defer]]]:
    """Flatten a list of steps into a linear path.

    Returns:
        nodes: Flat list of ``_Segment``, ``_SideAction``, or ``None``
            (deferred placeholder).
        deferred: List of ``(index, Defer)`` pairs for runtime resolution.
    """
    nodes: list[Optional[_PathNode]] = []
    deferred: list[tuple[int, Defer]] = []
    for step in steps:
        _flatten_one(step, nodes, deferred)
    return nodes, deferred


# ---------------------------------------------------------------------------
# Path optimizer
# ---------------------------------------------------------------------------

def _can_merge(a: _Segment, b: _Segment) -> bool:
    """Return True if two adjacent known-endpoint segments can be merged."""
    if a.condition is not None or b.condition is not None:
        return False
    if not a.has_known_endpoint or not b.has_known_endpoint:
        return False
    if a.kind == "linear" and b.kind == "linear":
        return (
            a.axis == b.axis
            and a.sign == b.sign
            and a.heading_deg == b.heading_deg
        )
    if a.kind == "turn" and b.kind == "turn":
        # Only merge if the combined angle stays non-zero
        combined = (a.angle_rad or 0.0) + (b.angle_rad or 0.0)
        return abs(combined) > 1e-6
    return False


def _merge_two(a: _Segment, b: _Segment) -> _Segment:
    """Merge two compatible segments. Precondition: _can_merge(a, b)."""
    if a.kind == "linear":
        return replace(
            a,
            distance_m=(a.distance_m or 0.0) + (b.distance_m or 0.0),
            speed_scale=min(a.speed_scale, b.speed_scale),
        )
    # turn + turn
    combined = (a.angle_rad or 0.0) + (b.angle_rad or 0.0)
    return replace(
        a,
        angle_rad=combined,
        sign=1.0 if combined >= 0 else -1.0,
        speed_scale=min(a.speed_scale, b.speed_scale),
    )


def _pass_merge(nodes: list) -> list:
    """Collapse adjacent same-type/same-direction segments with no conditions."""
    result: list = []
    i = 0
    while i < len(nodes):
        node = nodes[i]
        if isinstance(node, _Segment):
            seg = node
            j = i + 1
            while j < len(nodes) and isinstance(nodes[j], _Segment):
                if _can_merge(seg, nodes[j]):
                    seg = _merge_two(seg, nodes[j])
                    j += 1
                else:
                    break
            result.append(seg)
            i = j
        else:
            result.append(node)
            i += 1
    return result


def _try_corner_arc(
    lin1: _Segment, turn: _Segment, lin2: _Segment, cut_m: float,
) -> Optional[tuple[_Segment, _Segment, _Segment]]:
    """Return (new_lin1, arc, new_lin2) for a corner cut, or None if not applicable.

    Geometry: cutting *cut_m* from each leg at the corner requires an arc of
    radius ``R = cut_m / tan(|θ| / 2)``.  Both linears must be the same axis
    and direction and have enough distance to accommodate the cut.
    """
    if (turn.kind != "turn"
            or not turn.has_known_endpoint
            or turn.condition is not None
            or turn.angle_rad is None):
        return None
    if (lin1.kind != "linear"
            or not lin1.has_known_endpoint
            or lin1.condition is not None
            or lin1.distance_m is None):
        return None
    if (lin2.kind != "linear"
            or not lin2.has_known_endpoint
            or lin2.condition is not None
            or lin2.distance_m is None):
        return None
    if lin1.axis != lin2.axis or lin1.sign != lin2.sign:
        return None

    theta = turn.angle_rad
    if abs(theta) < 1e-4:
        return None

    # Both linears must be long enough to accommodate the cut
    if abs(lin1.distance_m) <= cut_m or abs(lin2.distance_m) <= cut_m:
        return None

    radius_m = cut_m / math.tan(abs(theta) / 2.0)

    # Trim each linear: new_d = signed_distance - sign * cut_m
    new_d1 = lin1.distance_m - lin1.sign * cut_m
    new_d2 = lin2.distance_m - lin2.sign * cut_m

    new_lin1 = replace(lin1, distance_m=new_d1)
    new_lin2 = replace(lin2, distance_m=new_d2)
    arc_seg = _Segment(
        kind="arc",
        radius_m=radius_m,
        arc_angle_rad=theta,
        speed_scale=min(lin1.speed_scale, turn.speed_scale, lin2.speed_scale),
        lateral=(lin1.axis == LinearAxis.Lateral),
        has_known_endpoint=True,
    )
    return new_lin1, arc_seg, new_lin2


def _pass_corner_cut(nodes: list, cut_m: float) -> list:
    """Replace linear+turn+linear triples with linear+arc+linear corner cuts."""
    result: list = []
    i = 0
    while i < len(nodes):
        # Peek ahead for a three-segment pattern
        if (i + 2 < len(nodes)
                and isinstance(nodes[i], _Segment)
                and isinstance(nodes[i + 1], _Segment)
                and isinstance(nodes[i + 2], _Segment)):
            cut = _try_corner_arc(nodes[i], nodes[i + 1], nodes[i + 2], cut_m)
            if cut is not None:
                new_lin1, arc_seg, new_lin2 = cut
                if abs(new_lin1.distance_m) > 1e-4:
                    result.append(new_lin1)
                result.append(arc_seg)
                if abs(new_lin2.distance_m) > 1e-4:
                    result.append(new_lin2)
                i += 3
                _log.debug(
                    "path optimizer: corner cut at index %d "
                    "(cut=%.3fm, radius=%.3fm, angle=%.1f°)",
                    i - 3, cut_m,
                    arc_seg.radius_m or 0,
                    math.degrees(arc_seg.arc_angle_rad or 0),
                )
                continue
        result.append(nodes[i])
        i += 1
    return result


def _segments_to_spline_waypoints(
    segments: list[_Segment],
) -> list[tuple[float, float]]:
    """Compute ``(forward_cm, left_cm)`` waypoints from a linear/turn sequence.

    Turns update the running heading but do not produce waypoints; each
    linear segment appends its endpoint.  The robot's start position (0, 0)
    is *not* included — it is the implicit origin for ``spline()``.

    Coordinate system: +forward is the robot's initial heading, +left is
    90° CCW of that (same convention as ``spline()`` waypoints).
    """
    x = 0.0      # meters, forward from start
    y = 0.0      # meters, left from start
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


def _build_spline_step(nodes: list) -> "SplinePath":
    """Build a SplinePath from a fully-splinifiable node list.

    Raises ``ValueError`` for any node that cannot be represented as a
    waypoint: deferred placeholders, side actions, arc segments,
    condition-based segments, or segments with unknown endpoints.
    A minimum of 2 linear segments is required so the spline has at
    least 2 explicit waypoints.
    """
    from .spline_path import SplinePath  # deferred to avoid circular import

    for node in nodes:
        if node is None:
            raise ValueError(
                "smooth_path(spline=True) cannot contain Defer steps — "
                "waypoints must be fully known at construction time"
            )
        if isinstance(node, _SideAction):
            raise ValueError(
                "smooth_path(spline=True) cannot contain side actions "
                "(background(), Run, or non-drive steps)"
            )
        if isinstance(node, _Segment):
            if node.condition is not None:
                raise ValueError(
                    "smooth_path(spline=True) cannot contain condition-based "
                    "segments (.until()) — endpoint must be known"
                )
            if not node.has_known_endpoint:
                raise ValueError(
                    "smooth_path(spline=True) requires all segments to have "
                    "known endpoints"
                )
            if node.kind == "arc":
                raise ValueError(
                    "smooth_path(spline=True) cannot contain arc segments — "
                    "use corner_cut_cm instead, or remove the arc"
                )

    segs = [n for n in nodes if isinstance(n, _Segment)]
    linear_count = sum(1 for s in segs if s.kind == "linear")
    if linear_count < 2:
        raise ValueError(
            "smooth_path(spline=True) requires at least 2 linear segments "
            f"to form a valid spline (found {linear_count})"
        )

    waypoints = _segments_to_spline_waypoints(segs)
    speed = min((s.speed_scale for s in segs if s.kind == "linear"), default=1.0)
    return SplinePath(waypoints, speed=speed)


def _optimize_nodes(
    nodes: list,
    *,
    merge: bool,
    corner_cut_m: float,
) -> list:
    """Apply optimization passes to the flattened path node list.

    ``_SideAction`` and ``None`` (deferred) barriers are preserved and
    never optimized across — they pin the execution point of side effects
    and runtime-resolved steps.

    Passes (applied in order when enabled):

    1. **Merge** (``merge=True``): collapse adjacent same-type segments that
       have no conditions and known endpoints.  Same-axis/same-direction
       linears sum their distances; consecutive turns sum their angles.

    2. **Corner cut** (``corner_cut_m > 0``): replace ``linear + turn + linear``
       triples with ``linear + arc + linear``.  The arc radius is derived from
       the cut distance via ``R = cut_m / tan(|θ| / 2)``.
    """
    if not merge and corner_cut_m <= 0.0:
        return nodes

    result = list(nodes)
    if merge:
        result = _pass_merge(result)
    if corner_cut_m > 0.0:
        result = _pass_corner_cut(result, corner_cut_m)
    return result


# ---------------------------------------------------------------------------
# SmoothPath Step
# ---------------------------------------------------------------------------

@dsl(hidden=True)
class SmoothPath(Step):
    """Execute a sequence of motion segments with smooth velocity transitions.

    Instead of decelerating to zero between each step, carries velocity
    across segment boundaries using warm-started motion controllers.

    Supports composite steps: nested ``seq()`` is flattened, ``parallel()``
    with a motion spine runs side-effect branches concurrently,
    ``background()`` steps are launched without blocking, and non-drive
    steps (servos, Run actions, etc.) execute at transition points.
    """

    hz: int = 100

    def __init__(
        self,
        steps: list,
        correct: bool = True,
        optimize: bool = False,
        corner_cut_m: float = 0.0,
        spline: bool = False,
    ) -> None:
        super().__init__()
        if not steps:
            raise ValueError("smooth_path() requires at least one step")
        if corner_cut_m > 0.0 and spline:
            raise ValueError(
                "smooth_path(): corner_cut_cm and spline=True are mutually exclusive — "
                "use one strategy for handling turns"
            )
        self._raw_steps = steps
        self._correct = correct
        self._optimize = optimize
        self._corner_cut_m = corner_cut_m
        self._spline_flag = spline
        self._nodes, self._deferred = _flatten_steps(steps)

        if optimize or corner_cut_m > 0.0:
            self._nodes = _optimize_nodes(
                self._nodes,
                merge=optimize,
                corner_cut_m=corner_cut_m,
            )

        # Build spline step at construction time so errors surface immediately.
        self._spline_step = _build_spline_step(self._nodes) if spline else None

        # Construction-time validation: at least one motion segment
        # (or a deferred that might resolve to one)
        has_segment = any(isinstance(n, _Segment) for n in self._nodes)
        has_deferred = len(self._deferred) > 0
        if not has_segment and not has_deferred:
            raise ValueError(
                "smooth_path() requires at least one motion step "
                "(drive, turn, or arc), but none were found"
            )

    _composite = True

    def required_resources(self) -> frozenset[str]:
        return frozenset({"drive"})

    def collected_resources(self) -> frozenset[str]:
        result: set[str] = {"drive"}
        for node in self._nodes:
            if isinstance(node, _SideAction):
                result |= node.step.collected_resources()
        return frozenset(result)

    def _generate_signature(self) -> str:
        parts = []
        for node in self._nodes:
            if node is None:
                parts.append("Defer(?)")
            elif isinstance(node, _Segment):
                seg = node
                if seg.kind == "linear":
                    d = f"{abs(seg.distance_m or 0) * 100:.0f}cm" if seg.distance_m else "until"
                    parts.append(f"drive({d})")
                elif seg.kind == "turn":
                    a = f"{abs(math.degrees(seg.angle_rad or 0)):.0f}\u00b0" if seg.angle_rad else "until"
                    parts.append(f"turn({a})")
                elif seg.kind == "arc":
                    parts.append(f"arc({abs(math.degrees(seg.arc_angle_rad or 0)):.0f}\u00b0)")
            elif isinstance(node, _SideAction):
                label = type(node.step).__name__
                mode = "bg" if node.is_background else "inline"
                parts.append(f"{label}({mode})")
        flags = []
        if self._spline_flag:
            flags.append("spline")
        elif self._optimize:
            flags.append("opt")
        if self._corner_cut_m > 0.0:
            flags.append(f"cut={self._corner_cut_m * 100:.0f}cm")
        suffix = f" [{', '.join(flags)}]" if flags else ""
        return f"SmoothPath([{', '.join(parts)}]{suffix})"

    # ------------------------------------------------------------------
    # Profile overshoot for non-terminal segments
    # ------------------------------------------------------------------
    # For intermediate segments, inflate the profile target so the
    # trapezoidal velocity profile stays in cruise phase at the actual
    # endpoint. The Python loop checks distance/angle manually and
    # triggers the transition while the robot is still at cruise speed.
    _OVERSHOOT_M = 1.0     # 1m overshoot (decel zone ~0.2m at typical params)
    _OVERSHOOT_RAD = 3.0   # ~172° overshoot (decel zone ~1.5rad at typical params)
    _DISTANCE_TOL_M = 0.005   # 5mm tolerance for manual distance check
    _ANGLE_TOL_RAD = 0.035    # ~2° tolerance for manual angle check

    # ------------------------------------------------------------------
    # Motion factory
    # ------------------------------------------------------------------

    def _create_linear_motion(
        self, robot: "GenericRobot", seg: _Segment, is_last: bool,
        correction: Optional[_Correction] = None,
    ) -> LinearMotion:
        config = LinearMotionConfig()
        config.axis = seg.axis
        actual = (
            seg.distance_m
            if seg.distance_m is not None
            else seg.sign * _Segment._SENTINEL_DISTANCE_M
        )

        # Apply along-track correction
        if correction and seg.distance_m is not None:
            # Positive distance_adjust_m = overshot = drive less
            # For negative distances (backward), the sign is already in actual
            actual -= math.copysign(correction.distance_adjust_m, actual)

        if not is_last and seg.distance_m is not None:
            # Inflate target so profile cruises through the actual endpoint
            config.distance_m = actual + math.copysign(self._OVERSHOOT_M, actual)
        else:
            config.distance_m = actual
        config.speed_scale = seg.speed_scale

        # Heading: correction target takes priority, then user-specified heading
        if correction and correction.heading_target_rad is not None:
            config.target_heading_rad = correction.heading_target_rad
        elif seg.heading_deg is not None:
            from raccoon.robot.heading_reference import HeadingReferenceService
            ref_svc = robot.get_service(HeadingReferenceService)
            sign = 1.0 if ref_svc._positive_direction == "left" else -1.0
            config.target_heading_rad = (
                ref_svc._reference_rad + sign * math.radians(seg.heading_deg)
            )

        motion = LinearMotion(
            robot.drive, robot.odometry,
            robot.motion_pid_config, config,
        )
        if not is_last:
            motion.set_suppress_hard_stop(True)
        return motion

    def _create_turn_motion(
        self, robot: "GenericRobot", seg: _Segment, is_last: bool,
        correction: Optional[_Correction] = None,
    ) -> TurnMotion:
        config = TurnConfig()
        actual = (
            seg.angle_rad
            if seg.angle_rad is not None
            else seg.sign * math.radians(180)  # sentinel for condition-based
        )

        # Apply heading correction
        if correction and seg.angle_rad is not None:
            actual -= correction.angle_adjust_rad

        if not is_last and seg.angle_rad is not None:
            config.target_angle_rad = actual + math.copysign(self._OVERSHOOT_RAD, actual)
        else:
            config.target_angle_rad = actual
        config.speed_scale = seg.speed_scale

        motion = TurnMotion(
            robot.drive, robot.odometry,
            robot.motion_pid_config, config,
        )
        if not is_last:
            motion.set_suppress_hard_stop(True)
        return motion

    def _create_arc_motion(
        self, robot: "GenericRobot", seg: _Segment, is_last: bool,
        correction: Optional[_Correction] = None,
    ) -> ArcMotion:
        config = ArcMotionConfig()
        config.radius_m = seg.radius_m
        actual = seg.arc_angle_rad

        # Apply heading correction
        if correction and actual is not None:
            actual -= correction.angle_adjust_rad

        if not is_last and actual is not None:
            config.arc_angle_rad = actual + math.copysign(self._OVERSHOOT_RAD, actual)
        else:
            config.arc_angle_rad = actual
        config.speed_scale = seg.speed_scale
        config.lateral = seg.lateral

        motion = ArcMotion(
            robot.drive, robot.odometry,
            robot.motion_pid_config, config,
        )
        if not is_last:
            motion.set_suppress_hard_stop(True)
        return motion

    def _create_motion(
        self, robot: "GenericRobot", seg: _Segment, is_last: bool,
        correction: Optional[_Correction] = None,
    ):
        if seg.kind == "linear":
            return self._create_linear_motion(robot, seg, is_last, correction)
        elif seg.kind == "turn":
            return self._create_turn_motion(robot, seg, is_last, correction)
        elif seg.kind == "arc":
            return self._create_arc_motion(robot, seg, is_last, correction)
        raise ValueError(f"Unknown segment kind: {seg.kind}")

    # ------------------------------------------------------------------
    # Segment completion check
    # ------------------------------------------------------------------

    @staticmethod
    def _has_reached_target(motion, seg: _Segment) -> bool:
        """Check if the motion has reached its distance/angle target.

        Used only for the **last** segment where the profile naturally
        decelerates to zero at the target.
        """
        if seg.kind == "linear":
            return motion.has_reached_distance()
        elif seg.kind in ("turn", "arc"):
            return motion.has_reached_angle()
        return motion.is_finished()

    def _check_segment_reached(
        self, robot: "GenericRobot", seg: _Segment, seg_origin: float,
        correction: Optional[_Correction] = None,
    ) -> bool:
        """Manual position check for non-terminal segments.

        Compares odometry-based distance traveled against the (possibly
        corrected) segment target. Used instead of the C++ check when
        the profile target is inflated with overshoot.
        """
        current = self._get_position_offset(robot, seg)
        traveled = current - seg_origin

        if seg.kind == "linear":
            target = seg.distance_m
            if correction and target is not None:
                target -= math.copysign(correction.distance_adjust_m, target)
            tol = self._DISTANCE_TOL_M
        elif seg.kind == "turn":
            target = seg.angle_rad
            if correction and target is not None:
                target -= correction.angle_adjust_rad
            tol = self._ANGLE_TOL_RAD
        elif seg.kind == "arc":
            target = seg.arc_angle_rad
            if correction and target is not None:
                target -= correction.angle_adjust_rad
            tol = self._ANGLE_TOL_RAD
        else:
            return False

        if target is None:
            return False
        if target >= 0:
            return traveled >= target - tol
        else:
            return traveled <= target + tol

    # ------------------------------------------------------------------
    # Velocity reading
    # ------------------------------------------------------------------

    @staticmethod
    def _get_current_velocity(motion, seg: _Segment) -> float:
        """Read the current filtered velocity from the active motion."""
        return motion.get_filtered_velocity()

    # ------------------------------------------------------------------
    # Odometry offset for warm start
    # ------------------------------------------------------------------

    @staticmethod
    def _get_position_offset(robot: "GenericRobot", seg: _Segment) -> float:
        """Get the current odometry position to use as offset for the next segment."""
        if seg.kind == "linear":
            info = robot.odometry.get_distance_from_origin()
            if seg.axis == LinearAxis.Forward:
                return info.forward
            else:
                return info.lateral
        elif seg.kind in ("turn", "arc"):
            return robot.odometry.get_heading()
        return 0.0

    # ------------------------------------------------------------------
    # Lazy deferred resolution
    # ------------------------------------------------------------------

    @staticmethod
    def _resolve_node_at(
        robot: "GenericRobot",
        nodes: list[Optional[_PathNode]],
        idx: int,
        deferred_map: dict[int, Defer],
    ) -> None:
        """Resolve a Defer placeholder at *idx* in-place.

        The Defer factory is called with the robot's **current** state
        (heading, position, etc.), so heading-dependent steps like
        ``turn_to_heading_right`` compute the correct angle at the
        actual transition point — not at the start of the path.

        If the Defer resolves to a composite (seq, parallel), the
        resulting nodes are spliced into *nodes* replacing the single
        ``None`` placeholder.
        """
        defer = deferred_map.pop(idx)
        resolved_step = defer.factory(robot)
        sub_nodes, sub_deferred = _flatten_steps([resolved_step])
        if sub_deferred:
            raise TypeError(
                "smooth_path(): a Defer step resolved to another "
                "Defer — nested deferral is not supported"
            )
        # Filter out any None sub-nodes (shouldn't happen, but be safe)
        resolved = [n for n in sub_nodes if n is not None]
        # Splice: replace the single None with the resolved node(s)
        nodes[idx:idx + 1] = resolved
        # Shift deferred_map keys that are after idx
        if resolved:
            shift = len(resolved) - 1  # we replaced 1 entry with N
            if shift != 0:
                new_map = {}
                for k, v in deferred_map.items():
                    new_map[k + shift if k > idx else k] = v
                deferred_map.clear()
                deferred_map.update(new_map)

    # ------------------------------------------------------------------
    # Side action execution
    # ------------------------------------------------------------------

    @staticmethod
    async def _advance_past_side_actions(
        robot: "GenericRobot",
        nodes: list[Optional[_PathNode]],
        start_idx: int,
        bg_tasks: list[asyncio.Task],
        deferred_map: dict[int, Defer],
    ) -> int:
        """Execute consecutive side actions from *start_idx*.

        Resolves any Defer placeholders encountered along the way.
        Returns the index of the next _Segment node (or len(nodes)).
        """
        idx = start_idx
        while idx < len(nodes):
            # Resolve deferred node if needed
            if nodes[idx] is None and idx in deferred_map:
                SmoothPath._resolve_node_at(robot, nodes, idx, deferred_map)
            node = nodes[idx]
            if not isinstance(node, _SideAction):
                break  # found a _Segment (or unexpected None)
            if node.is_background:
                task = asyncio.create_task(node.step.run_step(robot))
                bg_tasks.append(task)
            else:
                await node.step.run_step(robot)
            idx += 1
        return idx

    @staticmethod
    def _is_last_segment(
        nodes: list[Optional[_PathNode]], current_idx: int,
    ) -> bool:
        """Check if there are no more _Segment nodes after *current_idx*.

        Unresolved Defer placeholders (``None``) are treated as potential
        segments (conservative — no overshoot, profile decelerates).
        """
        for i in range(current_idx + 1, len(nodes)):
            if isinstance(nodes[i], _Segment) or nodes[i] is None:
                return False
        return True

    # ------------------------------------------------------------------
    # Main execution loop
    # ------------------------------------------------------------------

    async def _execute_step(self, robot: "GenericRobot") -> None:
        # Spline mode: delegate entirely to the pre-built SplinePath step.
        if self._spline_step is not None:
            await self._spline_step.run_step(robot)
            return

        # 1. Build mutable working copy of nodes and deferred map
        nodes: list[Optional[_PathNode]] = list(self._nodes)
        deferred_map: dict[int, Defer] = dict(self._deferred)

        # 2. Calibration check (for segments known at construction time)
        has_calibrated_drive = any(
            isinstance(n, _Segment) and n.kind == "linear" and n.has_known_endpoint
            for n in nodes
        )
        if has_calibrated_drive:
            from raccoon.step.calibration import check_distance_calibration
            check_distance_calibration()

        # 3. Execute the path
        bg_tasks: list[asyncio.Task] = []

        try:
            node_idx = 0

            # Execute any leading side actions (resolves Defers lazily)
            node_idx = await self._advance_past_side_actions(
                robot, nodes, node_idx, bg_tasks, deferred_map,
            )

            if node_idx >= len(nodes):
                return  # only side actions

            # Resolve deferred first segment if needed
            if nodes[node_idx] is None and node_idx in deferred_map:
                self._resolve_node_at(robot, nodes, node_idx, deferred_map)

            # Runtime validation: first node must be a segment
            if not isinstance(nodes[node_idx], _Segment):
                raise ValueError(
                    "smooth_path() resolved to no motion segments — "
                    "at least one drive, turn, or arc step is required"
                )

            # Start first motion segment (cold start)
            seg: _Segment = nodes[node_idx]  # type: ignore[assignment]
            is_last = self._is_last_segment(nodes, node_idx)
            motion = self._create_motion(robot, seg, is_last)
            motion.start()
            seg_origin = self._get_position_offset(robot, seg)

            # Initialize world-frame pose tracker (after first start() resets odom)
            tracker: Optional[_WorldPoseTracker] = None
            seg_correction: Optional[_Correction] = None
            if self._correct:
                tracker = _WorldPoseTracker(
                    start_heading=robot.odometry.get_absolute_heading(),
                )

            if seg.condition is not None:
                seg.condition.start(robot)

            # 4. Unified control loop
            update_rate = 1.0 / self.hz
            loop = asyncio.get_event_loop()
            last_time = loop.time() - update_rate  # seed first dt

            while True:
                current_time = loop.time()
                dt = max(current_time - last_time, 0.0)
                last_time = current_time

                if dt < 1e-4:
                    await asyncio.sleep(update_rate)
                    continue

                transition = False

                # Check condition-based termination
                if seg.condition is not None and seg.condition.check(robot):
                    transition = True

                # Always update the motion controller this cycle
                motion.update(dt)

                # Check distance/angle completion
                if not transition and seg.has_known_endpoint:
                    if is_last:
                        # Last segment: use C++ check (profile decelerates
                        # naturally to the real target)
                        transition = self._has_reached_target(motion, seg)
                    else:
                        # Non-terminal: manual check against actual target
                        # while the profile's inflated target keeps the
                        # robot cruising at full speed
                        transition = self._check_segment_reached(
                            robot, seg, seg_origin, seg_correction,
                        )

                if transition:
                    # Read current velocity before transitioning
                    current_vel = self._get_current_velocity(motion, seg)
                    prev_seg = seg

                    # -- World-frame correction: update tracker at transition --
                    was_condition_based = (
                        not prev_seg.has_known_endpoint
                        or prev_seg.condition is not None and not prev_seg.has_known_endpoint
                    )
                    if tracker is not None:
                        tracker.advance_expected(prev_seg)
                        if was_condition_based:
                            tracker.reset_expected_to_actual(robot)

                    node_idx += 1

                    # Execute side actions at transition point
                    # (also lazily resolves any Defer side-actions)
                    node_idx = await self._advance_past_side_actions(
                        robot, nodes, node_idx, bg_tasks, deferred_map,
                    )

                    if node_idx >= len(nodes):
                        break  # path complete

                    # Track whether next segment came from a Defer
                    was_deferred = (
                        nodes[node_idx] is None and node_idx in deferred_map
                    )

                    # Lazily resolve Defer segment at transition point
                    # so heading-dependent steps see the current heading
                    if was_deferred:
                        self._resolve_node_at(
                            robot, nodes, node_idx, deferred_map,
                        )

                    # After resolution, skip any new side actions
                    node_idx = await self._advance_past_side_actions(
                        robot, nodes, node_idx, bg_tasks, deferred_map,
                    )

                    if node_idx >= len(nodes):
                        break

                    seg = nodes[node_idx]  # type: ignore[assignment]
                    is_last = self._is_last_segment(nodes, node_idx)

                    # Compute correction for the upcoming segment
                    seg_correction = None
                    if tracker is not None:
                        seg_correction = tracker.compute_correction(robot, seg)
                        # Deferred heading turns already computed the
                        # correct angle from current heading — zero out
                        # angle correction to avoid double-correcting
                        if was_deferred and seg.kind in ("turn", "arc"):
                            seg_correction.angle_adjust_rad = 0.0

                    if _is_same_type(prev_seg, seg):
                        # Same type: warm start — carry velocity seamlessly
                        offset = self._get_position_offset(robot, prev_seg)
                        motion = self._create_motion(
                            robot, seg, is_last, seg_correction,
                        )
                        motion.start_warm(offset, current_vel)
                    else:
                        # Cross-type: cold start next (no soft_stop — let
                        # the new controller take over immediately while
                        # the old axis velocity coasts down naturally)
                        if tracker is not None:
                            tracker.snapshot_before_cold_start(robot)
                        motion = self._create_motion(
                            robot, seg, is_last, seg_correction,
                        )
                        motion.start()

                    seg_origin = self._get_position_offset(robot, seg)

                    if seg.condition is not None:
                        seg.condition.start(robot)

                await asyncio.sleep(update_rate)
        finally:
            robot.drive.hard_stop()
            # Clean up any background side-action tasks
            for task in bg_tasks:
                if not task.done():
                    task.cancel()
            if bg_tasks:
                await asyncio.gather(*bg_tasks, return_exceptions=True)


# ---------------------------------------------------------------------------
# DSL factory function
# ---------------------------------------------------------------------------

@dsl(tags=["motion", "path"])
def smooth_path(
    *steps,
    correct: bool = True,
    optimize: bool = False,
    corner_cut_cm: float | None = None,
    spline: bool = False,
) -> SmoothPath:
    """Execute motion steps with smooth velocity transitions between them.

    Eliminates the velocity drops that occur when steps are chained via
    ``seq()``. Instead of decelerating to zero and hard-stopping between
    each step, ``smooth_path`` carries velocity across segment boundaries,
    producing continuous fluid motion.

    For same-type transitions (e.g., drive to drive), the velocity is
    carried seamlessly via warm-started motion controllers with zero
    discontinuity. For cross-type transitions (e.g., drive to turn),
    a soft stop is used which smoothly decelerates without resetting
    PID state.

    When ``correct`` is enabled (default), the path tracks the robot's
    world-frame pose and corrects accumulated position and heading
    errors at each segment transition. Along-track errors adjust the
    next linear segment's distance, cross-track errors bias the heading
    hold, and heading errors adjust turn/arc angles. All corrections
    are clamped to safe limits (3 cm distance, 5 deg heading).

    Supports composite steps:

    - **Nested seq()**: Automatically flattened into the motion spine.
    - **parallel()**: The branch containing drive steps becomes the
      motion spine; other branches run concurrently as side effects.
    - **background()**: Launched without blocking the motion flow.
    - **Non-drive steps** (servos, ``Run`` actions, etc.): Execute at
      transition points between motion segments.

    Steps with ``.until()`` conditions are fully supported. When a
    condition fires, the current velocity is carried into the next
    segment immediately.

    Supports ``Defer`` steps (e.g., ``turn_to_heading_right``), which
    are resolved **lazily** at the actual transition point so that
    heading-dependent steps see the robot's current heading, not the
    heading at the start of the path.

    **Optimization** (all opt-in):

    ``optimize=True`` applies algebraic simplification passes at
    construction time, before execution:

    - *Merge*: adjacent same-type, same-direction, same-axis segments
      with no conditions are collapsed into one.  ``drive(30) + drive(20)``
      becomes ``drive(50)``; ``turn(30°) + turn(20°)`` becomes ``turn(50°)``.
      Only same-direction merges are performed (``drive(30) + drive(-20)``
      is not merged — intent may differ from net displacement).
      ``_SideAction`` and deferred nodes act as merge barriers.

    ``corner_cut_cm=N`` (implies algebraic merge as well) replaces
    ``linear + turn + linear`` triples with ``linear + arc + linear`` by
    cutting *N* cm from each straight leg and inserting a circular arc
    of radius ``R = N / tan(|θ| / 2)`` at the corner.  The robot never
    slows for the turn — it follows the arc at cruise speed.  Only
    segments with known endpoints and no conditions are eligible.

    Note on ``_SideAction`` barriers: a ``background()`` or non-drive step
    between two segments pins that step to the transition point.  Optimization
    never reorders across side actions, preserving the execution contract.

    Prerequisites:
        ``calibrate_distance()`` if any segment uses distance-based mode.
        ``mark_heading_reference()`` if any segment uses heading hold.

    Args:
        *steps: Motion steps to execute smoothly. Accepts
            ``drive_forward``, ``drive_backward``, ``strafe_left``,
            ``strafe_right``, ``turn_left``, ``turn_right``,
            ``turn_to_heading_left``, ``turn_to_heading_right``,
            ``drive_arc_left``, ``drive_arc_right``, nested ``seq()``,
            ``parallel()`` with a motion spine, ``background()``,
            and their builder variants with ``.until()`` and ``.speed()``.
        correct: Enable world-frame error correction across segment
            transitions. Default ``True``. Set to ``False`` for the
            legacy uncorrected behavior.
        optimize: Merge adjacent same-type segments algebraically.
            Default ``False`` (opt-in).
        corner_cut_cm: If set, replace right-angle drive+turn+drive
            corners with arcs of radius ``corner_cut_cm / tan(|θ|/2)``,
            cutting *corner_cut_cm* cm from each straight leg.  Also
            enables the merge pass.  Mutually exclusive with ``spline``.
            Default ``None`` (disabled).
        spline: If True, convert the drive/turn sequence into waypoints
            and follow them with a Catmull-Rom spline (``SplinePath``).
            The entire path executes as a single continuous curve —
            no segment transitions, no velocity drops.  Requires all
            segments to have known endpoints and no conditions or side
            actions.  Mutually exclusive with ``corner_cut_cm``.
            Default ``False`` (disabled).

    Returns:
        A SmoothPath step that executes the segments as one fluid motion.

    Example::

        from raccoon.step.motion import smooth_path, drive_forward, turn_to_heading_right
        from raccoon.step.condition import on_black

        # Three drives with no stops between them
        smooth_path(
            drive_forward(50),
            drive_forward(30),
            drive_forward(20),
        )

        # Drive, turn, drive — soft transition at type boundaries
        smooth_path(
            drive_forward(50),
            turn_to_heading_right(90),
            drive_forward(30),
        )

        # Merge adjacent drives and cut corners with 5 cm arcs
        smooth_path(
            drive_forward(30),
            drive_forward(20),
            turn_left(90),
            drive_forward(40),
            optimize=True,
            corner_cut_cm=5.0,
        )
        # → drive(50) + arc(R=5cm, 90°) + drive(35)

        # Condition-based segment carries velocity into the next
        smooth_path(
            drive_forward(speed=0.8).until(on_black(sensor)),
            drive_forward(20),
        )

        # Parallel: servo moves while driving continuously
        smooth_path(
            drive_forward(30),
            parallel(
                [drive_forward(20), drive_forward(10)],
                [servo_move(0, 1500)],
            ),
        )

        # Background: non-blocking step alongside motion
        smooth_path(
            drive_forward(30),
            background(servo_move(0, 1500)),
            drive_forward(20),
        )

        # Disable correction for legacy behavior
        smooth_path(
            drive_forward(50),
            turn_to_heading_right(90),
            drive_forward(30),
            correct=False,
        )

        # Spline mode: drive+turn+drive → smooth curved path via Catmull-Rom.
        # The robot follows the spline tangent; the drive and turn steps provide
        # the waypoints and heading changes.  Requires at least 2 linear
        # segments; raises ValueError for condition-based steps, side actions,
        # arc segments, or fewer than 2 waypoints.
        smooth_path(
            drive_forward(50),
            turn_right(90),
            drive_forward(30),
            spline=True,
        )
    """
    cut_m = (corner_cut_cm / 100.0) if corner_cut_cm is not None else 0.0
    do_merge = optimize or (corner_cut_cm is not None)
    return SmoothPath(
        list(steps),
        correct=correct,
        optimize=do_merge,
        corner_cut_m=cut_m,
        spline=spline,
    )
