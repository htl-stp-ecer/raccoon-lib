"""``to_absolute`` pass — turn dead-reckoning relative runs into closed-loop legs.

A run of consecutive known-endpoint ``linear`` / ``turn`` segments (no live
condition) is converted into a series of ``goto_relative`` navigate-to-pose
moves.  Each ``goto_relative`` leg captures the localization pose at start and
regulates onto ``run_start_pose ⊕ delta`` using the particle filter as
feedback — so the open-loop dead-reckoning legs become closed-loop moves that
shrug off odometry drift.

Crucially this requires ZERO executor changes: the replacement nodes are inline
(blocking) ``SideAction``s, and the executor already runs inline side actions
via ``await step.run_step(robot)``.  ``goto_relative`` is a ``MotionStep``, whose
``run_step`` drives its full ``on_start`` / ``on_update`` / ``on_stop`` loop to
completion — so each leg runs its closed-loop controller to the target.

Run-detection: a MAXIMAL run is a stretch of consecutive ``Segment`` nodes that
are ALL ``kind in ("linear", "turn")``, ``has_known_endpoint is True`` and whose
condition is "baked" — i.e. ``None``, or a bare relative ``after_cm`` already
folded into ``distance_m`` by ``known_distance()`` (the runtime odometer stop is
kept but the geometric endpoint is exact).  Anything else — a ``SideAction``, a
``None`` deferred placeholder, an ``arc`` / ``follow_line`` / ``spline``
segment, a segment with a live (early-stopping / sensor / absolute) condition or
an unknown endpoint — BREAKS the run and passes through unchanged.  Body-frame pose is integrated from ``(x=0, y=0, heading=0)`` at the
run start, reusing the SAME forward/left integration as
``segments_to_spline_waypoints`` (in ``spline.py``).  ONE ``goto_relative`` is
emitted per LINEAR segment endpoint (preserving the path shape — no corner
cutting); turns between linears fold into the following waypoint's target
heading.

Best-effort: segments that don't qualify are left untouched; the pass never
crashes.  It is intended to run AFTER ``known_distance()`` so ``.until(after_cm)``
legs qualify.

Representation declaration is intentionally left undeclared (defaults to
``EITHER`` / ``SAME`` / non-terminal) for v1 — see ``optimize.py`` for the
contract.  Declaring ``requires = Representation.RELATIVE`` from here would
introduce a circular import with ``optimize.py``, so it is omitted.
"""

from __future__ import annotations

import math

from raccoon.motion import LinearAxis

from ..ir import PathNode, Segment, SideAction

_RUN_KINDS = ("linear", "turn")


def _condition_is_baked(seg: Segment) -> bool:
    """Is ``seg``'s condition (if any) already captured by ``distance_m``?

    A ``None`` condition is trivially fine.  A bare relative ``after_cm`` is
    the one condition ``known_distance()`` bakes into ``distance_m`` while
    leaving in place as the runtime odometer stop — for a known-endpoint
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


def _run_to_goto_legs(run: list[Segment]) -> list[Segment]:
    """Integrate a run into one ``goto_relative`` leg per linear segment.

    Body-frame pose starts at ``(x=0, y=0, heading=0)``; ``+x`` is forward
    (run-start heading), ``+y`` is left (90° CCW), heading is CCW-positive —
    the SAME convention as ``segments_to_spline_waypoints``.  Each ``linear``
    advances ``(x, y)`` and emits a leg carrying the body-frame delta from the
    run start to that waypoint plus the heading there; each ``turn`` only
    updates the running heading (folding into the next leg's target heading).
    """
    from ...goto import goto_relative  # deferred to avoid import cycle

    x = 0.0  # metres, forward from run start
    y = 0.0  # metres, left from run start
    heading = 0.0  # radians, CCW positive

    legs: list[Segment] = []
    for seg in run:
        if seg.kind == "linear":
            d = seg.distance_m or 0.0
            if seg.axis == LinearAxis.Forward:
                x += d * math.cos(heading)
                y += d * math.sin(heading)
            else:  # Lateral
                x += d * (-math.sin(heading))
                y += d * math.cos(heading)
            step = goto_relative(
                forward_cm=x * 100.0,
                left_cm=y * 100.0,
                dtheta_deg=math.degrees(heading),
                speed=seg.speed_scale,
            )
            legs.append(step)
        elif seg.kind == "turn":
            heading += seg.angle_rad or 0.0
    return legs


class ToAbsolutePass:
    """Convert known-endpoint relative runs into closed-loop ``goto_relative`` legs.

    Pure node→node pass.  Replaces each maximal run of qualifying
    ``linear`` / ``turn`` segments with inline ``SideAction(goto_relative)``
    nodes — one per linear endpoint.  Non-qualifying nodes pass through
    unchanged.  Use after ``known_distance()`` so ``.until(after_cm())`` legs
    qualify.

    Representation/terminal contract left undeclared (defaults to
    ``EITHER`` / ``SAME`` / non-terminal) for v1.
    """

    name = "to_absolute"

    def run(self, nodes: list[PathNode | None]) -> list[PathNode | None]:
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

            legs = _run_to_goto_legs(run)
            if legs:
                for step in legs:
                    result.append(SideAction(step=step, is_background=False))
            else:
                # Run had no linear segments (e.g. turn-only) — nothing to
                # convert; pass the original nodes through untouched.
                result.extend(run)
            i = j
        return result
