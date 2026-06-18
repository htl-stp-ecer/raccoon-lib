"""``to_absolute`` pass — turn dead-reckoning relative runs into ONE navigate-to-pose.

A contiguous run of consecutive known-endpoint ``linear`` / ``turn`` /
``diagonal`` segments (no live condition) is converted into a SINGLE
``GotoWaypoints`` navigate-to-pose
move.  ``GotoWaypoints`` captures the localization pose ONCE at the run start
(its ``on_start``) as a single anchor, precomputes the ABSOLUTE world target for
each of the run's waypoints (fixed geometry, known at compile time), then
regulates onto each target in sequence using the particle filter as feedback —
so the open-loop dead-reckoning legs become one closed-loop run that shrugs off
odometry drift.  Anchor scope is PER RUN: each contiguous run captures its own
anchor at its first leg, and every waypoint within the run is relative to that
ONE anchor (not chained leg-to-leg), so drift does NOT accumulate across legs.

Crucially this requires ZERO executor changes: the replacement node is an inline
(blocking) ``SideAction``, and the executor already runs inline side actions via
``await step.run_step(robot)``.  ``GotoWaypoints`` is a ``MotionStep``, whose
``run_step`` drives its full ``on_start`` / ``on_update`` / ``on_stop`` loop to
completion — so the run drives its closed-loop controller through every target.

Run-detection: a MAXIMAL run is a stretch of consecutive ``Segment`` nodes that
are ALL ``kind in ("linear", "turn", "diagonal")``, ``has_known_endpoint is
True`` and whose
condition is "baked" — i.e. ``None``, or a bare relative ``after_cm`` whose
distance is folded into ``distance_m`` at lowering time (the runtime odometer
stop is kept but the geometric endpoint is exact).  Anything else — a ``SideAction``, a
``None`` deferred placeholder, an ``arc`` / ``follow_line`` / ``spline``
segment, a segment with a live (early-stopping / sensor / absolute) condition or
an unknown endpoint — BREAKS the run and passes through unchanged.  Body-frame pose is integrated from ``(x=0, y=0, heading=0)`` at the
run start, reusing the SAME forward/left integration as
``segments_to_spline_waypoints`` (in ``spline.py``).  ONE waypoint is
collected per LINEAR segment endpoint (preserving the path shape — no corner
cutting); turns between linears fold into the following waypoint's target
heading.  All the run's waypoints feed ONE emitted ``GotoWaypoints``.

Best-effort: segments that don't qualify are left untouched; the pass never
crashes.  ``.until(after_cm)`` legs qualify automatically — their distance is
recovered at lowering time.

Representation declaration is intentionally left undeclared (defaults to
``EITHER`` / ``SAME`` / non-terminal) for v1 — see ``optimize.py`` for the
contract.  Declaring ``requires = Representation.RELATIVE`` from here would
introduce a circular import with ``optimize.py``, so it is omitted.
"""

from __future__ import annotations

import math

from raccoon.motion import LinearAxis

from ..ir import PathNode, Segment, SideAction

_RUN_KINDS = ("linear", "turn", "diagonal")


def _condition_is_baked(seg: Segment) -> bool:
    """Is ``seg``'s condition (if any) already captured by ``distance_m``?

    A ``None`` condition is trivially fine.  A bare relative ``after_cm`` is
    the one condition whose distance is baked into ``distance_m`` at lowering
    time while left in place as the runtime odometer stop — for a known-endpoint
    segment that distance IS the true endpoint, so the leg is convertible.
    Any other condition (combined ``_Then`` / ``_AnyOf`` / sensor stop, or an
    absolute ``after_cm``) may stop early and BREAKS the run.
    """
    if seg.condition is None:
        return True
    from ....condition import after_cm  # deferred to avoid import cycle

    cond = seg.condition
    return isinstance(cond, after_cm) and not cond._absolute


def _segment_qualifies(node) -> bool:
    """Is ``node`` a run-eligible known-endpoint linear/turn?

    Requires a known endpoint and either no condition or a bare relative
    ``after_cm`` whose distance is already baked into ``distance_m`` (so the
    geometric endpoint is exact).
    """
    return (
        isinstance(node, Segment)
        and node.kind in _RUN_KINDS
        and node.has_known_endpoint
        and _condition_is_baked(node)
    )


def _run_to_waypoints(run: list[Segment]) -> tuple[list[tuple[float, float, float]], float]:
    """Integrate a run into one body-frame waypoint per linear segment.

    Body-frame pose starts at ``(x=0, y=0, heading=0)``; ``+x`` is forward
    (run-start heading), ``+y`` is left (90° CCW), heading is CCW-positive —
    the SAME convention as ``segments_to_spline_waypoints``.  Each ``linear``
    advances ``(x, y)`` and emits a waypoint carrying the body-frame delta from
    the run start to that point plus the heading there; each ``diagonal`` holds
    heading and advances ``(x, y)`` by its known body-frame displacement,
    emitting a waypoint; each ``turn`` only updates the running heading (folding
    into the next waypoint's heading).

    All waypoints are expressed relative to the SINGLE run-start frame (NOT
    chained leg-to-leg), so a downstream ``GotoWaypoints`` resolves them against
    one anchor.  Returns ``(waypoints, speed)`` where each waypoint is
    ``(forward_m, left_m, dtheta_rad)`` and ``speed`` is the run speed scale
    (the first linear segment's, since the single step carries one speed).
    """
    x = 0.0  # metres, forward from run start
    y = 0.0  # metres, left from run start
    heading = 0.0  # radians, CCW positive

    waypoints: list[tuple[float, float, float]] = []
    speed = 1.0
    speed_set = False
    for seg in run:
        if seg.kind == "linear":
            d = seg.distance_m or 0.0
            if seg.axis == LinearAxis.Forward:
                x += d * math.cos(heading)
                y += d * math.sin(heading)
            else:  # Lateral
                x += d * (-math.sin(heading))
                y += d * math.cos(heading)
            waypoints.append((x, y, heading))
            if not speed_set:
                speed = seg.speed_scale
                speed_set = True
        elif seg.kind == "diagonal":
            # A diagonal holds heading and translates by a known body-frame
            # displacement (+forward / +left). Rotate it into the world frame.
            fwd = seg.forward_m or 0.0
            left = seg.left_m or 0.0
            x += fwd * math.cos(heading) - left * math.sin(heading)
            y += fwd * math.sin(heading) + left * math.cos(heading)
            waypoints.append((x, y, heading))
            if not speed_set:
                speed = seg.speed_scale
                speed_set = True
        elif seg.kind == "turn":
            heading += seg.angle_rad or 0.0
    return waypoints, speed


class ToAbsolutePass:
    """Convert each known-endpoint relative run into ONE closed-loop ``GotoWaypoints``.

    Pure node→node pass.  Replaces each maximal run of qualifying
    ``linear`` / ``turn`` segments with a SINGLE inline
    ``SideAction(GotoWaypoints)`` node carrying all the run's body-frame
    waypoints — one per linear endpoint.  The ``GotoWaypoints`` captures one
    anchor per run at start and drives to the resulting ABSOLUTE world targets
    in sequence (so drift does not chain leg-to-leg).  Non-qualifying nodes pass
    through unchanged.  ``.until(after_cm())`` legs qualify automatically — their
    distance is recovered at lowering time.

    Representation/terminal contract left undeclared (defaults to
    ``EITHER`` / ``SAME`` / non-terminal) for v1.
    """

    name = "to_absolute"

    def run(self, nodes: list[PathNode | None]) -> list[PathNode | None]:
        from ...goto import GotoWaypoints  # deferred to avoid import cycle

        result: list[PathNode | None] = []
        i = 0
        n = len(nodes)
        while i < n:
            if not _segment_qualifies(nodes[i]):
                result.append(nodes[i])
                i += 1
                continue

            # Gather the maximal run of qualifying segments.
            j = i
            run: list[Segment] = []
            while j < n and _segment_qualifies(nodes[j]):
                run.append(nodes[j])  # type: ignore[arg-type]
                j += 1

            waypoints, speed = _run_to_waypoints(run)
            if waypoints:
                step = GotoWaypoints(waypoints=waypoints, speed=speed)
                result.append(SideAction(step=step, is_background=False))
            else:
                # Run had no linear segments (e.g. turn-only) — nothing to
                # convert; pass the original nodes through untouched.
                result.extend(run)
            i = j
        return result
