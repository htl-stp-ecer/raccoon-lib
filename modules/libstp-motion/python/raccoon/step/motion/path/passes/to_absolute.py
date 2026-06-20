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

import logging
import math

from raccoon.motion import LinearAxis

from ..ir import PathNode, Segment, SideAction

_log = logging.getLogger(__name__)

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
    """Is ``node`` a run-eligible known-endpoint linear/turn/diagonal?

    Requires a known endpoint and either no condition or a bare relative
    ``after_cm`` whose distance is already baked into ``distance_m`` (so the
    geometric endpoint is exact).

    A heading turn (``TurnToHeading``, carried as an ``opaque_step``) and a
    heading-HOLDING drive (a ``linear`` with ``heading_deg``) DO qualify — both
    target the heading reference, an ABSOLUTE frame whose offset to the
    localization frame (the mark offset ``O``) is known at COMPILE time, so
    ``_run_to_waypoints`` integrates them into the same run.
    """
    return (
        isinstance(node, Segment)
        and node.kind in _RUN_KINDS
        and node.has_known_endpoint
        and _condition_is_baked(node)
    )


Waypoint = tuple[float, float, float, float, str, float]
"""One GotoWaypoints leg, fully resolved at COMPILE time:
``(rel_forward_m, rel_left_m, abs_dx_m, abs_dy_m, heading_kind, heading_value)``.

GotoWaypoints composes it against the single run anchor as::

    world = anchor.pos + Rot(anchor.heading)·(rel_forward, rel_left) + (abs_dx, abs_dy)
    theta = anchor.heading + heading_value   if heading_kind == "rel"
            heading_value                     if heading_kind == "abs"
"""


def _abs_heading_rad(r_deg: float, o_rad: float, sign: float) -> float:
    """Reference-relative degrees ``r`` → absolute heading in the LOCALIZATION frame.

    Localization 0° = the start heading; the heading reference sits ``O`` =
    ``mark`` ``origin_offset_deg`` away from it (the compile-time offset).  The
    service computes ``target_absolute_rad(r) = reference_rad + sign·radians(r)``
    in the odometry frame; expressed in the localization frame the runtime start
    heading cancels and only the compile-time constants remain::

        θ_localization = radians(O) + sign·radians(r)

    so a turn_to_heading / heading-holding drive resolves to a fixed heading at
    COMPILE time, with NO runtime service read or frame bridge.
    """
    return o_rad + sign * math.radians(r_deg)


def _run_has_absolute(run: list[Segment]) -> bool:
    """Does the run contain a heading-reference-anchored leg?"""
    return any(
        (s.kind == "turn" and s.opaque_step is not None)
        or (s.kind == "linear" and s.heading_deg is not None)
        for s in run
    )


def _run_to_waypoints(
    run: list[Segment], o_rad: float = 0.0, sign: float = 1.0
) -> tuple[list[Waypoint], float]:
    """Integrate a run into GotoWaypoints legs — entirely at COMPILE time.

    Two frames, both compile-resolved (``o_rad`` / ``sign`` come from the active
    ``mark_heading_reference``):

    - RELATIVE legs (plain drive/turn/strafe, no held heading) integrate in the
      run-start body frame (heading starts at 0); carried as a body-frame delta
      that GotoWaypoints rotates by the runtime anchor heading.
    - ABSOLUTE legs (``turn_to_heading``, or a drive/strafe with ``heading=``)
      target a heading fixed to the reference — known in the localization frame
      as ``radians(O) + sign·r`` (see :func:`_abs_heading_rad`).  Once an
      absolute leg appears, the running heading is known absolutely, so every
      following leg integrates in the localization frame directly.

    A ``turn`` only updates the running heading (folds into the next translating
    leg's heading); a trailing ``turn`` emits a rotate-in-place waypoint so the
    final heading is still regulated.  Returns ``(waypoints, speed)``.
    """
    rel_x = rel_y = 0.0  # body-frame accumulation (relative phase)
    rel_h = 0.0  # heading delta from run start (relative phase)
    abs_x = abs_y = 0.0  # localization-frame accumulation (absolute phase)
    abs_h = 0.0  # localization-frame heading (absolute phase)
    abs_active = False

    waypoints: list[Waypoint] = []
    speed = 1.0
    speed_set = False

    for seg in run:
        if seg.kind == "turn":
            if seg.opaque_step is not None:  # turn_to_heading → absolute target
                abs_h = _abs_heading_rad(seg.opaque_step._target_deg, o_rad, sign)
                abs_active = True
            elif abs_active:
                abs_h += seg.angle_rad or 0.0
            else:
                rel_h += seg.angle_rad or 0.0
            continue  # pure rotation — folds into the next translating leg

        # Translating leg (linear or diagonal): compute its body displacement.
        if seg.kind == "linear":
            d = seg.distance_m or 0.0
            if seg.heading_deg is not None:  # heading hold → absolute target
                abs_h = _abs_heading_rad(seg.heading_deg, o_rad, sign)
                abs_active = True
            if seg.axis == LinearAxis.Forward:
                lf, ll = d, 0.0
            else:  # Lateral (+left), distance_m already signed
                lf, ll = 0.0, d
        else:  # diagonal — holds heading, known body-frame displacement
            lf, ll = seg.forward_m or 0.0, seg.left_m or 0.0

        if abs_active:
            abs_x += lf * math.cos(abs_h) - ll * math.sin(abs_h)
            abs_y += lf * math.sin(abs_h) + ll * math.cos(abs_h)
            waypoints.append((rel_x, rel_y, abs_x, abs_y, "abs", abs_h))
        else:
            rel_x += lf * math.cos(rel_h) - ll * math.sin(rel_h)
            rel_y += lf * math.sin(rel_h) + ll * math.cos(rel_h)
            waypoints.append((rel_x, rel_y, 0.0, 0.0, "rel", rel_h))

        if not speed_set:
            speed = seg.speed_scale
            speed_set = True

    # Trailing pure rotation (run ends on a turn / turn_to_heading): emit a
    # rotate-in-place waypoint so the final heading is still regulated.
    if run and run[-1].kind == "turn":
        if abs_active:
            waypoints.append((rel_x, rel_y, abs_x, abs_y, "abs", abs_h))
        else:
            waypoints.append((rel_x, rel_y, 0.0, 0.0, "rel", rel_h))

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
        from ...heading_reference import MarkHeadingReference  # deferred (import cycle)

        # The active heading reference (compile-time offset + sign) as set by the
        # most recent mark_heading_reference() seen while walking the nodes. Used
        # to resolve turn_to_heading / heading-holding legs into the localization
        # frame at COMPILE time (no runtime service read).
        active_mark: tuple[float, float] | None = None

        result: list[PathNode | None] = []
        i = 0
        n = len(nodes)
        while i < n:
            node_i = nodes[i]
            if isinstance(node_i, SideAction) and isinstance(node_i.step, MarkHeadingReference):
                active_mark = (
                    math.radians(node_i.step._origin_offset_deg),
                    1.0 if node_i.step._positive_direction == "left" else -1.0,
                )
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

            if _run_has_absolute(run):
                if active_mark is None:
                    # A heading-reference-anchored leg (turn_to_heading /
                    # heading-holding drive) needs the mark offset to resolve its
                    # absolute target, but no mark_heading_reference() is in
                    # scope — common when optimize() wraps a single mission whose
                    # mark was set by an earlier mission. Rather than crash the
                    # build, DEGRADE GRACEFULLY: pass this run through unchanged
                    # so its legs run as ordinary relative (heading-holding)
                    # moves. to_absolute is best-effort and must never break a
                    # build; the only cost is no drift-correction on this run.
                    _log.debug(
                        "to_absolute: run with a heading-reference leg has no "
                        "mark_heading_reference() in scope — left relative (%d segs)",
                        len(run),
                    )
                    result.extend(run)
                    i = j
                    continue
                o_rad, sign = active_mark
            else:
                o_rad, sign = 0.0, 1.0  # no absolute legs — values unused
            waypoints, speed = _run_to_waypoints(run, o_rad, sign)
            if waypoints:
                step = GotoWaypoints(waypoints=waypoints, speed=speed)
                result.append(SideAction(step=step, is_background=False))
            else:
                # Run had no linear segments (e.g. turn-only) — nothing to
                # convert; pass the original nodes through untouched.
                result.extend(run)
            i = j
        return result
