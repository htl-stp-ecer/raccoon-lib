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
            if node.kind == "arc":
                msg = (
                    "splinify() cannot contain arc segments — "
                    "use corner_cut_cm instead, or remove the arc"
                )
                raise ValueError(msg)
            if node.kind in ("follow_line", "spline"):
                msg = (
                    f"splinify() cannot contain "
                    f"{node.kind} segments — endpoint must be a simple "
                    f"linear/turn sequence"
                )
                raise ValueError(msg)

    segs = [n for n in nodes if isinstance(n, Segment)]
    linear_count = sum(1 for s in segs if s.kind == "linear")
    if linear_count < 2:
        msg = (
            "splinify() requires at least 2 linear segments "
            f"to form a valid spline (found {linear_count})"
        )
        raise ValueError(msg)

    waypoints = segments_to_spline_waypoints(segs)
    speed = min((s.speed_scale for s in segs if s.kind == "linear"), default=1.0)
    return SplinePath(waypoints, speed=speed)


class SplinifyPass:
    """Terminal pass that collapses a relative path into one ABSOLUTE spline.

    Validates the entire node list via :func:`build_spline_step` (which raises
    ``ValueError`` for anything that can't be splinified — defers, side actions,
    arcs, conditions, or fewer than 2 linear segments) and reuses its control
    waypoints.  Instead of emitting the relative odometry-driven
    ``Segment(kind="spline")``, it densely samples the SAME centripetal
    Catmull-Rom curve (see :func:`sample_centripetal_catmull_rom`) through those
    control points and replaces the whole run with ONE inline
    ``SideAction(GotoWaypoints)`` that follows the dense samples CLOSED-LOOP on
    the localization particle filter — correcting drift along the whole curve
    and composing with the rest of the absolute pipeline.

    Heading: the dense waypoints use ``dtheta=None`` (HOLD the anchor heading),
    matching ``GotoWaypoints``' translation-along-the-curve semantics. The
    relative ``SplinePath`` orients along the spline TANGENT, but tangent-follow
    requires the robot to slew its heading at every sample and assumes an omni
    base; holding heading lets a differential or omni base translate along the
    SAME absolute curve geometry without per-sample rotation. The traced path
    (the ``(x, y)`` samples) is identical to the relative spline's curve.

    This is a *terminal* pass: it replaces the path wholesale, so nothing
    may run after it in a pipeline.
    """

    name = "splinify"
    requires = Representation.RELATIVE
    terminal = True

    def run(self, nodes: list[PathNode | None]) -> list[PathNode | None]:
        from ...goto import GotoWaypoints  # deferred to avoid import cycle

        # Validate (propagating ValueErrors) and obtain the control waypoints +
        # speed.  ``build_spline_step`` returns a SplinePath carrying the control
        # waypoints (centimetres, +forward / +left body frame).
        spline_path = build_spline_step(nodes)
        control_points_m = [
            (fwd_cm / 100.0, left_cm / 100.0) for (fwd_cm, left_cm) in spline_path._waypoints
        ]

        # Same control points as the relative spline (the implicit (0, 0)
        # run-start is NOT a control point — the relative SplineMotion likewise
        # builds its curve only through these explicit waypoints), so the
        # absolute curve traces the SAME geometry.
        dense = sample_centripetal_catmull_rom(control_points_m)

        # HOLD heading (dtheta=None): translate along the absolute curve holding
        # the anchor heading. Body-frame +forward / +left, all relative to the
        # single anchor GotoWaypoints captures at on_start.
        waypoints = [(x_m, y_m, None) for (x_m, y_m) in dense]

        speed = spline_path._speed
        step = GotoWaypoints(waypoints=waypoints, speed=speed)
        return [SideAction(step=step, is_background=False)]
