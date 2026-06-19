"""Spline-conversion pass — translate a linear/turn sequence into a SplinePath.

Unlike merge / corner-cut (pure node-list transforms), this pass *replaces*
the entire path with a single ABSOLUTE spline-driven step.  It is therefore a
"terminal" transform — typically the last entry in a pipeline that opts in
to spline mode.

The pass densely samples the SAME centripetal Catmull-Rom curve used by the
relative ``SplinePath`` / C++ ``SplineMotion`` (see ``catmull_rom_spline.hpp``)
and emits ONE :class:`~raccoon.step.motion.goto.GotoWaypoints` side-action that
follows those dense samples CLOSED-LOOP on the localization particle filter —
so drift is corrected along the whole curve.  The relative ``SplineMotion`` /
``smooth_path(spline=True)`` path (which uses :func:`build_spline_step`
directly) is UNAFFECTED.
"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

from raccoon.motion import LinearAxis

from ..ir import PathNode, Segment, SideAction
from .contract import Representation

if TYPE_CHECKING:
    from ...spline_path import SplinePath


# Dense-sample spacing along the absolute spline (metres).  ~3 cm is fine
# enough that GotoWaypoints' proportional controller flows continuously through
# the samples (pursuit-like) rather than stopping at each.
_SAMPLE_SPACING_M = 0.03


def _catmull_rom_segment(
    p0: tuple[float, float],
    p1: tuple[float, float],
    p2: tuple[float, float],
    p3: tuple[float, float],
    t: float,
) -> tuple[float, float]:
    """Evaluate one centripetal Catmull-Rom segment ``P1→P2`` at local ``t``.

    Barry-Goldman evaluation with centripetal (alpha=0.5) knot spacing —
    a 1:1 port of ``CatmullRomSpline::evaluateSegment`` in
    ``catmull_rom_spline.hpp`` so the absolute spline traces the SAME geometry
    as the relative ``SplineMotion``.
    """

    def knot(a: tuple[float, float], b: tuple[float, float], t_prev: float) -> float:
        d = math.hypot(b[0] - a[0], b[1] - a[1])
        return t_prev + math.sqrt(max(d, 1e-12))

    t0 = 0.0
    t1 = knot(p0, p1, t0)
    t2 = knot(p1, p2, t1)
    t3 = knot(p2, p3, t2)

    u = t1 + t * (t2 - t1)

    def lerp(
        a: tuple[float, float],
        b: tuple[float, float],
        ta: float,
        tb: float,
        u_: float,
    ) -> tuple[float, float]:
        denom = tb - ta
        if abs(denom) < 1e-12:
            return a
        alpha = (u_ - ta) / denom
        return (
            (1.0 - alpha) * a[0] + alpha * b[0],
            (1.0 - alpha) * a[1] + alpha * b[1],
        )

    a1 = lerp(p0, p1, t0, t1, u)
    a2 = lerp(p1, p2, t1, t2, u)
    a3 = lerp(p2, p3, t2, t3, u)

    b1 = lerp(a1, a2, t0, t2, u)
    b2 = lerp(a2, a3, t1, t3, u)

    return lerp(b1, b2, t1, t2, u)


def sample_centripetal_catmull_rom(
    control_points_m: list[tuple[float, float]],
    spacing_m: float = _SAMPLE_SPACING_M,
) -> list[tuple[float, float]]:
    """Densely sample a centripetal Catmull-Rom curve through ``control_points_m``.

    Matches ``CatmullRomSpline`` in ``catmull_rom_spline.hpp`` exactly:

    - Centripetal parameterisation (alpha=0.5): knot spacing is the square root
      of the chord length between consecutive control points.
    - Virtual endpoints via REFLECTION (not duplication) so the curve passes
      through every control point: ``P[-1] = 2*P[0] - P[1]`` and
      ``P[N] = 2*P[N-1] - P[N-2]``.
    - Barry-Goldman segment evaluation.

    Returns a flat list of ``(x_m, y_m)`` points, including the first and last
    control points, with consecutive points roughly ``spacing_m`` apart so the
    full path from the first to the last control point is covered.

    Args:
        control_points_m: At least 2 control points ``(x_m, y_m)``.
        spacing_m: Target arc-length spacing between samples (metres).

    Returns:
        Dense ``[(x_m, y_m), ...]`` samples along the curve.
    """
    if len(control_points_m) < 2:
        msg = "sample_centripetal_catmull_rom requires at least 2 control points"
        raise ValueError(msg)

    first = control_points_m[0]
    second = control_points_m[1]
    last = control_points_m[-1]
    second_last = control_points_m[-2]

    virtual_start = (2.0 * first[0] - second[0], 2.0 * first[1] - second[1])
    virtual_end = (2.0 * last[0] - second_last[0], 2.0 * last[1] - second_last[1])

    points = [virtual_start, *control_points_m, virtual_end]
    num_segments = len(points) - 3

    # Estimate the per-segment sample count from a coarse chord-length pass so
    # the average spacing is ~spacing_m (the proportional controller flows
    # through the samples, so exact arc-length parameterisation is unnecessary).
    samples: list[tuple[float, float]] = [
        _catmull_rom_segment(points[0], points[1], points[2], points[3], 0.0)
    ]
    coarse = 16
    for seg in range(num_segments):
        p0, p1, p2, p3 = points[seg], points[seg + 1], points[seg + 2], points[seg + 3]
        seg_len = 0.0
        prev = _catmull_rom_segment(p0, p1, p2, p3, 0.0)
        for i in range(1, coarse + 1):
            cur = _catmull_rom_segment(p0, p1, p2, p3, i / coarse)
            seg_len += math.hypot(cur[0] - prev[0], cur[1] - prev[1])
            prev = cur
        n = max(1, int(math.ceil(seg_len / spacing_m)))
        for i in range(1, n + 1):
            samples.append(_catmull_rom_segment(p0, p1, p2, p3, i / n))

    return samples


# Sample an arc roughly every this many radians (~18°) so the Catmull-Rom
# traces the arc's curvature rather than chording it.  At least 2 samples per
# arc are emitted regardless (the endpoint and one intermediate).
_ARC_SAMPLE_STEP_RAD = math.radians(18.0)


def _arc_samples(
    x: float,
    y: float,
    heading: float,
    radius_m: float,
    arc_angle_rad: float,
    lateral: bool,
) -> tuple[list[tuple[float, float]], float, float, float]:
    """Sample one arc into intermediate ``(x_m, y_m)`` control points.

    Reproduces the ``ArcMotion`` (``arc_motion.cpp``) geometry exactly:

    - The arc has signed sweep ``arc_angle_rad`` (>0 = CCW/left, <0 = CW/right);
      the robot's heading rotates by this amount over the arc.
    - ``lateral=False`` (drive arc): the body-frame velocity is ``+vx`` (forward,
      along ``heading``).  ``lateral=True`` (strafe arc): the velocity is ``+vy``
      with sign matching ``arc_angle`` (``+`` = left for CCW), i.e. along
      ``heading + sign(arc_angle)·90°``.
    - Centre of the circle is 90° to the LEFT of the velocity for CCW motion and
      90° to the RIGHT for CW motion, i.e. at ``velocity_dir + sign·90°`` at
      ``radius_m``.  The centre→robot vector rotates rigidly with the heading, so
      sampling at heading-sweep fraction ``f`` places the robot at
      ``centre + rotate(robot - centre, arc_angle·f)``.

    Returns ``(samples, x_end, y_end, heading_end)`` where ``samples`` is the
    list of ``(x_m, y_m)`` control points (excluding the arc start, including the
    endpoint) and the end pose is the exact arc endpoint so subsequent segments
    integrate from the true position/heading.
    """
    radius = radius_m or 0.0
    sign = 1.0 if arc_angle_rad >= 0.0 else -1.0

    # Velocity direction in world frame at the arc start.
    velocity_dir = heading if not lateral else heading + sign * (math.pi / 2.0)
    # Centre is 90° toward the inside of the turn from the velocity.
    centre_dir = velocity_dir + sign * (math.pi / 2.0)
    cx = x + radius * math.cos(centre_dir)
    cy = y + radius * math.sin(centre_dir)

    # Centre→robot vector at the arc start; it rotates by the heading sweep.
    r0x = x - cx
    r0y = y - cy

    sweep = abs(arc_angle_rad)
    num = max(2, int(math.ceil(sweep / _ARC_SAMPLE_STEP_RAD)))

    samples: list[tuple[float, float]] = []
    for i in range(1, num + 1):
        f = i / num
        ang = arc_angle_rad * f
        ca, sa = math.cos(ang), math.sin(ang)
        px = cx + (r0x * ca - r0y * sa)
        py = cy + (r0x * sa + r0y * ca)
        samples.append((px, py))

    # Final pose: position is the last sample, heading sweeps by arc_angle.
    x_end, y_end = samples[-1]
    return samples, x_end, y_end, heading + arc_angle_rad


def segments_to_spline_waypoints(
    segments: list[Segment],
) -> list[tuple[float, float]]:
    """Compute ``(forward_cm, left_cm)`` control waypoints from a segment sequence.

    Walks the SAME body-frame pose integration as ``to_absolute`` (+forward is
    the robot's initial heading, +left is 90° CCW, heading CCW-positive):

    - ``linear`` advances along its axis and emits the endpoint.
    - ``diagonal`` holds heading and advances by its known body-frame
      displacement ``(forward_m, left_m)`` rotated into world, emitting the
      endpoint.
    - ``arc`` is SAMPLED into several intermediate control points (≈ one every
      18° of sweep, min 2) so the Catmull-Rom traces the arc's curvature; the
      running pose advances to the arc's exact endpoint/heading.
    - ``turn`` only updates the running heading (no waypoint) — the corner is
      rounded by the Catmull-Rom through the neighbouring waypoints.

    The robot's start position ``(0, 0)`` is *not* included — it is the implicit
    origin for ``spline()``.
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
        elif seg.kind == "diagonal":
            fwd = seg.forward_m or 0.0
            left = seg.left_m or 0.0
            x += fwd * math.cos(heading) - left * math.sin(heading)
            y += fwd * math.sin(heading) + left * math.cos(heading)
            waypoints.append((x * 100.0, y * 100.0))
        elif seg.kind == "arc":
            samples, x, y, heading = _arc_samples(
                x,
                y,
                heading,
                seg.radius_m or 0.0,
                seg.arc_angle_rad or 0.0,
                seg.lateral,
            )
            for sx, sy in samples:
                waypoints.append((sx * 100.0, sy * 100.0))
        elif seg.kind == "turn":
            heading += seg.angle_rad or 0.0

    return waypoints


def build_spline_step(nodes: list[PathNode | None]) -> "SplinePath":
    """Build a SplinePath from a fully-splinifiable node list.

    Raises ``ValueError`` for any node that cannot be represented as a
    waypoint: deferred placeholders, side actions, condition-based segments,
    ``follow_line`` / ``spline`` segments, or segments with unknown endpoints.
    Arcs and diagonals ARE accepted — they are integrated into control
    waypoints the Catmull-Rom traces (arcs are sampled along their curvature).
    A minimum of 2 control waypoints (after sampling) is required so the spline
    has at least 2 explicit control points.
    """
    from ...spline_path import SplinePath  # deferred to avoid circular import

    for node in nodes:
        if node is None:
            msg = (
                "splinify() cannot contain Defer steps — "
                "waypoints must be fully known at construction time"
            )
            raise ValueError(msg)
        if isinstance(node, SideAction):
            msg = (
                "splinify() cannot contain side actions " "(background(), Run, or non-drive steps)"
            )
            raise ValueError(msg)
        if isinstance(node, Segment):
            if node.condition is not None:
                msg = (
                    "splinify() cannot contain condition-based "
                    "segments (.until()) — endpoint must be known"
                )
                raise ValueError(msg)
            if not node.has_known_endpoint:
                msg = "splinify() requires all segments to have " "known endpoints"
                raise ValueError(msg)
            if node.kind in ("follow_line", "spline"):
                msg = (
                    f"splinify() cannot contain "
                    f"{node.kind} segments — endpoint must be a simple "
                    f"linear/turn sequence"
                )
                raise ValueError(msg)
            if node.kind == "turn" and (node.opaque_step is not None or node.angle_rad is None):
                # A heading turn (TurnToHeading) targets an ABSOLUTE reference
                # heading, not a relative delta, so it cannot be folded into the
                # curve (``heading += angle_rad`` would crash on a None angle).
                msg = (
                    "splinify() cannot contain turn_to_heading_* turns — they "
                    "target an absolute reference heading, not a relative angle, "
                    "so they cannot be folded into a spline"
                )
                raise ValueError(msg)

    segs = [n for n in nodes if isinstance(n, Segment)]
    waypoints = segments_to_spline_waypoints(segs)
    if len(waypoints) < 2:
        msg = (
            "splinify() requires at least 2 control waypoints "
            f"to form a valid spline (found {len(waypoints)})"
        )
        raise ValueError(msg)

    geom_kinds = ("linear", "diagonal", "arc")
    speed = min((s.speed_scale for s in segs if s.kind in geom_kinds), default=1.0)
    return SplinePath(waypoints, speed=speed)


class SplinifyPass:
    """Terminal pass that collapses the whole path into ONE continuous spline.

    Validates the node list via :func:`build_spline_step` (raises ``ValueError``
    for anything unsplinifiable — defers, side actions, conditions, or fewer than
    2 control waypoints; arcs/diagonals ARE accepted and become control
    waypoints — "everything is one spline") and reuses its control waypoints.

    Two render modes, chosen by the builder from whether ``to_absolute()`` is on:

    - **relative** (default) → ``Segment(kind="spline")``: the continuous,
      odometry-driven C++ ``SplineMotion`` follows the centripetal Catmull-Rom.
    - **absolute** → ``SideAction(SplineFollow)``: a continuous pure-pursuit
      follower rides the SAME curve closed-loop on the localization particle
      filter, correcting drift along the whole curve. Holonomic — heading held,
      independent of the path tangent.

    Terminal: it replaces the path wholesale, so nothing may run after it.
    """

    name = "splinify"
    requires = Representation.RELATIVE
    terminal = True

    def __init__(self, absolute: bool = False) -> None:
        # ``absolute`` is set by the builder from whether to_absolute() is on.
        self.absolute = absolute

    def run(self, nodes: list[PathNode | None]) -> list[PathNode | None]:
        # Validate (propagating ValueErrors) and build the spline control points
        # (arcs/diagonals already sampled into waypoints by build_spline_step).
        spline_path = build_spline_step(nodes)

        if not self.absolute:
            # Relative (default): the continuous odometry-driven C++ SplineMotion
            # segment. Already smooth — no per-sample stepping.
            return spline_path.lower_to_segments()

        # Absolute: a continuous pure-pursuit follower rides the SAME centripetal
        # Catmull-Rom curve closed-loop on the localization particle filter,
        # correcting drift along the whole curve. Holonomic (heading held).
        from ...goto import SplineFollow  # deferred to avoid import cycle

        control_points_m = [
            (fwd_cm / 100.0, left_cm / 100.0) for (fwd_cm, left_cm) in spline_path._waypoints
        ]
        step = SplineFollow(
            waypoints=control_points_m,
            speed=spline_path._speed,
            heading_mode="hold",
        )
        return [SideAction(step=step, is_background=False)]
