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

SENSOR-bounded single-axis linear legs.  A ``linear`` segment with NO known
endpoint and a real (non-baked) ``condition`` — e.g. ``strafe_left().until(
on_black)`` — does NOT join a known-endpoint run, but it pins 2 of 3 world DOF
(the cross-axis position + heading) and leaves only the travel axis free until
the sensor fires.  Such a leg is converted into a single inline
``SideAction(AbsoluteHoldMove)``: a closed-loop-on-localization step that holds
the 2 pinned DOF on their absolute targets (correcting drift DURING the move,
read-only on the particle filter) while driving the free axis open-loop until
the original ``.until()`` condition fires.  This keeps the WHOLE to_absolute
path on the particle filter rather than odometry dead reckoning.  Each
``AbsoluteHoldMove`` re-anchors at its own ``on_start``, so the next known run
re-anchors naturally and a sensor leg flushes any in-progress known run.

Best-effort: segments that don't qualify are left untouched; the pass never
crashes.  ``.until(after_cm)`` legs qualify automatically — their distance is
recovered at lowering time, so a BAKED ``after_cm`` leg becomes a
``GotoWaypoints`` waypoint, NOT an ``AbsoluteHoldMove``.  ``follow_line`` /
``spline`` / ``arc`` / ``diagonal`` sensor legs are left untouched.

to_absolute is driven by the builder as a MODE flag (``optimizer._absolute``),
not a representation-changing pass. When ``splinify()`` is also chained the whole
path becomes ONE absolute spline and this per-leg pass is skipped; with only
``to_absolute()`` the builder appends this pass for the per-leg conversion. It
leaves ``produces`` at the default (SAME) so chaining with the terminal
``splinify()`` is allowed.
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


def _is_sensor_linear_leg(node) -> bool:
    """Is ``node`` a SENSOR-bounded single-axis LINEAR leg (no known endpoint)?

    Such a leg (e.g. ``strafe_left().until(on_black)``) pins 2 of 3 world DOF
    (cross-axis position + heading) and leaves the travel axis free until the
    sensor fires. It does NOT qualify for the known-endpoint ``GotoWaypoints``
    run, but to_absolute converts it into an :class:`AbsoluteHoldMove` that
    holds the 2 pinned DOF absolutely while driving the free axis until the
    condition.

    Requires a ``linear`` segment with an UNKNOWN endpoint and a real,
    non-baked condition (a sensor / combined / absolute-after_cm stop). A bare
    relative ``after_cm`` is "baked" — its distance becomes a known endpoint, so
    it stays on the ``GotoWaypoints`` path and is NOT matched here.
    """
    return (
        isinstance(node, Segment)
        and node.kind == "linear"
        and node.has_known_endpoint is False
        and node.condition is not None
        and not _condition_is_baked(node)
    )


def _segment_qualifies(node) -> bool:
    """Is ``node`` a run-eligible known-endpoint linear/turn?

    Requires a known endpoint and either no condition or a bare relative
    ``after_cm`` whose distance is already baked into ``distance_m`` (so the
    geometric endpoint is exact).

    A ``turn`` qualifies ONLY when it is a RELATIVE turn — ``opaque_step is
    None`` and ``angle_rad is not None``.  A heading turn (``TurnToHeading``,
    carried as an ``opaque_step`` with ``angle_rad is None``) targets an
    ABSOLUTE reference heading, not a per-run relative delta, so folding it via
    ``heading += angle_rad`` would corrupt the run (and crash on ``None``).  It
    therefore BREAKS the run and passes through as a plain ``Segment`` that the
    executor runs absolute via the adapter; the GotoWaypoints before/after
    re-anchor.
    """
    if not (
        isinstance(node, Segment)
        and node.kind in _RUN_KINDS
        and node.has_known_endpoint
        and _condition_is_baked(node)
    ):
        return False
    # A heading turn (opaque_step set / angle_rad None) is NOT run-eligible.
    return not (node.kind == "turn" and (node.opaque_step is not None or node.angle_rad is None))


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
    in sequence (so drift does not chain leg-to-leg).

    Additionally converts each SENSOR-bounded single-axis ``linear`` leg (no
    known endpoint, real non-baked condition — e.g. ``strafe_left().until(
    on_black)``) into a single inline ``SideAction(AbsoluteHoldMove)`` that
    holds the cross-axis position + heading absolutely (2 DOF, read-only on
    localization) while driving the free axis until the sensor fires.  Such a
    leg flushes any in-progress known run first.

    Non-qualifying nodes pass through unchanged.  ``.until(after_cm())`` legs
    qualify automatically — their distance is recovered at lowering time, so a
    baked ``after_cm`` leg stays a ``GotoWaypoints`` waypoint (NOT an
    ``AbsoluteHoldMove``).  ``follow_line`` / ``spline`` / ``arc`` / ``diagonal``
    sensor legs pass through untouched.

    to_absolute is driven by the builder as a MODE flag (``optimizer._absolute``),
    not a representation-changing pass: when ``splinify()`` is also chained the
    whole path becomes one absolute spline and this per-leg pass is skipped; when
    it isn't, the builder appends this pass to do the per-leg conversion. It
    therefore leaves ``produces`` at the default (SAME) so chaining with the
    terminal ``splinify()`` is allowed.
    """

    name = "to_absolute"

    def run(self, nodes: list[PathNode | None]) -> list[PathNode | None]:
        from ...goto import AbsoluteHoldMove, GotoWaypoints  # deferred (import cycle)

        result: list[PathNode | None] = []
        i = 0
        n = len(nodes)
        while i < n:
            if not _segment_qualifies(nodes[i]):
                node = nodes[i]
                if _is_sensor_linear_leg(node):
                    # Sensor-bounded single-axis linear leg: hold the cross-axis
                    # position + heading absolutely while driving the free axis
                    # until the sensor fires. (Each AbsoluteHoldMove re-anchors
                    # at its own on_start, so no run flush is needed here — runs
                    # are already flushed before any non-qualifying node.)
                    result.append(
                        SideAction(
                            step=AbsoluteHoldMove(
                                free_axis=node.axis,
                                sign=node.sign,
                                speed=node.speed_scale,
                                condition=node.condition,
                            ),
                            is_background=False,
                        )
                    )
                    i += 1
                    continue
                result.append(node)
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
