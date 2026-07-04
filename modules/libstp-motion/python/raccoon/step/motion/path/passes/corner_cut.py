"""Corner-cut pass — round a corner between two straight legs with an arc.

Two corner shapes are cut, both trimming ``cut_m`` from each straight leg:

* **Turn corner** — ``linear + turn + linear`` (the legs share an axis and
  direction; the heading rotates by the turn angle). Becomes ``linear + arc +
  linear`` with ``R = cut_m / tan(|θ| / 2)``. Works for forward AND strafe legs
  (a ``strafe + turn + strafe`` corner yields a lateral arc).

* **Crab corner** — ``linear(Forward) + linear(Lateral)`` (or vice-versa) with
  NO turn between: a holonomic base changes travel direction by 90° WITHOUT
  rotating. Becomes ``linear + crab_arc + linear`` — a constant-heading quarter
  circle of radius ``R = cut_m`` (``tan 45° = 1``) that blends the body velocity
  from one leg's direction into the other's.

Only triples/pairs with known endpoints and no conditions are eligible.
``SideAction`` and deferred-placeholder (``None``) entries act as barriers;
background / ephemeral parallel-branch side actions are skipped over (they run
concurrently with the motion) and re-emitted around the arc.
"""

from __future__ import annotations

import logging
import math
from dataclasses import replace

from raccoon.motion import LinearAxis

from ..ir import PathNode, Segment, SideAction

_log = logging.getLogger(__name__)


def _collect_bg(nodes: list[PathNode | None], start: int) -> tuple[list[SideAction], int] | None:
    """Collect leading background side actions from ``start``.

    Returns ``(bgs, idx-of-next-non-background-node)``, or ``None`` if the list
    runs out. A blocking (inline) side action or a deferred ``None`` is NOT
    collected — it is returned as the next node so the caller sees the barrier.
    """
    bgs: list[SideAction] = []
    j = start
    while j < len(nodes):
        node = nodes[j]
        if isinstance(node, SideAction) and node.is_background:
            bgs.append(node)
            j += 1
            continue
        return bgs, j  # next non-background node (may be Segment / barrier)
    return None  # ran off the end


def _match_corner(
    nodes: list[PathNode | None],
    i: int,
) -> tuple[Segment, list[SideAction], Segment, list[SideAction], Segment, int] | None:
    """Match ``lin1 [bg…] turn [bg…] lin2`` starting at index ``i``.

    Background / ephemeral parallel-branch ``SideAction``s run CONCURRENTLY with
    the motion (they don't stop the robot), so they must not break a corner —
    a ``drive + background(arm) + turn + drive`` is still a geometric corner.
    They are collected and re-emitted by the caller. A blocking (inline) side
    action, a deferred placeholder (``None``), or the end of the list is a hard
    barrier. This only does the STRUCTURAL match (three Segments in order);
    ``try_corner_arc`` validates kinds/geometry.
    """
    if not isinstance(nodes[i], Segment):
        return None
    lin1 = nodes[i]

    a = _collect_bg(nodes, i + 1)
    if a is None:
        return None
    bgs_a, turn_idx = a
    if not isinstance(nodes[turn_idx], Segment):
        return None
    turn = nodes[turn_idx]

    b = _collect_bg(nodes, turn_idx + 1)
    if b is None:
        return None
    bgs_b, lin2_idx = b
    if not isinstance(nodes[lin2_idx], Segment):
        return None
    lin2 = nodes[lin2_idx]

    return lin1, bgs_a, turn, bgs_b, lin2, lin2_idx + 1


def _match_crab_corner(
    nodes: list[PathNode | None],
    i: int,
) -> tuple[Segment, list[SideAction], Segment, int] | None:
    """Match ``lin1 [bg…] lin2`` (no turn between) starting at index ``i``.

    Same barrier rules as :func:`_match_corner`. ``try_crab_arc`` validates that
    the two legs are perpendicular (one Forward, one Lateral).
    """
    if not isinstance(nodes[i], Segment):
        return None
    lin1 = nodes[i]

    a = _collect_bg(nodes, i + 1)
    if a is None:
        return None
    bgs, lin2_idx = a
    if not isinstance(nodes[lin2_idx], Segment):
        return None
    lin2 = nodes[lin2_idx]

    return lin1, bgs, lin2, lin2_idx + 1


def _trimmable(lin: Segment, cut_m: float) -> bool:
    """A linear leg with a known, condition-free endpoint long enough to trim."""
    return (
        lin.kind == "linear"
        and lin.has_known_endpoint
        and lin.condition is None
        and lin.distance_m is not None
        and abs(lin.distance_m) > cut_m
    )


def _exit_eligible(lin: Segment, cut_m: float, cut_until: bool) -> bool:
    """May ``lin`` be the EXIT leg of a corner?

    Trimmable known legs always qualify. A sensor-bounded ``.until()`` leg
    qualifies ONLY with ``cut_until`` opted in: its endpoint is unknown so it is
    kept untrimmed — the arc rounds the corner and the leg then runs from the
    arc's end until its condition fires (the sensor still triggers at the same
    world point). Note this is exit-only: a corner's ENTRY leg can never be
    sensor-bounded, because the fillet would have to start curving *before* the
    sensor fires, which can't be anticipated.
    """
    if _trimmable(lin, cut_m):
        return True
    return cut_until and lin.kind == "linear" and lin.condition is not None


def _trim_toward_corner(lin: Segment, cut_m: float) -> Segment:
    """Trim ``cut_m`` off the leg's corner-side end.

    A sensor-bounded leg (no known distance) is returned unchanged — it runs
    until its condition, so there is nothing to trim.
    """
    if lin.condition is not None or lin.distance_m is None:
        return lin
    return replace(lin, distance_m=lin.distance_m - lin.sign * cut_m)


def _body_dir(lin: Segment) -> tuple[float, float]:
    """Body-frame unit travel direction (vx, vy) of a linear leg.

    vx > 0 = forward, vy > 0 = strafe right (matches ``ChassisVelocity``).
    Forward/backward maps to ±vx via ``sign``; strafe right/left to ±vy.
    """
    if lin.axis == LinearAxis.Forward:
        return (lin.sign, 0.0)
    return (0.0, lin.sign)


def try_corner_arc(
    lin1: Segment,
    turn: Segment,
    lin2: Segment,
    cut_m: float,
    cut_until: bool = False,
) -> tuple[Segment, Segment, Segment] | None:
    """Return ``(new_lin1, arc, new_lin2)`` for a turn corner, or ``None``.

    Geometry: cutting ``cut_m`` from each leg at the corner requires an arc of
    radius ``R = cut_m / tan(|θ| / 2)``.  Both linears must be the same axis
    and direction (the heading rotates by the turn angle, so the path deflects
    by exactly ``θ``) and have enough distance to accommodate the cut. A pair of
    lateral (strafe) legs yields a lateral arc; forward legs a drive arc.

    The entry leg must be a known, trimmable distance. With ``cut_until`` the
    EXIT leg may instead be a sensor-bounded ``.until()`` leg (kept untrimmed).
    """
    if (
        turn.kind != "turn"
        or not turn.has_known_endpoint
        or turn.condition is not None
        or turn.angle_rad is None
    ):
        return None
    if not _trimmable(lin1, cut_m) or not _exit_eligible(lin2, cut_m, cut_until):
        return None
    if lin1.axis != lin2.axis or lin1.sign != lin2.sign:
        return None

    theta = turn.angle_rad
    if abs(theta) < 1e-4:
        return None

    radius_m = cut_m / math.tan(abs(theta) / 2.0)

    new_lin1 = _trim_toward_corner(lin1, cut_m)
    new_lin2 = _trim_toward_corner(lin2, cut_m)
    arc_seg = Segment(
        kind="arc",
        radius_m=radius_m,
        arc_angle_rad=theta,
        speed_scale=min(lin1.speed_scale, turn.speed_scale, lin2.speed_scale),
        lateral=(lin1.axis == LinearAxis.Lateral),
        has_known_endpoint=True,
    )
    return new_lin1, arc_seg, new_lin2


def try_crab_arc(
    lin1: Segment,
    lin2: Segment,
    cut_m: float,
    cut_until: bool = False,
) -> tuple[Segment, Segment, Segment] | None:
    """Return ``(new_lin1, crab_arc, new_lin2)`` for a crab corner, or ``None``.

    A crab corner is two perpendicular straight legs (one Forward, one Lateral)
    with NO turn between them: a holonomic base changes travel direction by 90°
    at constant heading. The fillet is a quarter circle of radius
    ``R = cut_m / tan(45°) = cut_m``; the executor's ``CrabArcAdapter`` sweeps
    the body velocity from ``lin1``'s direction into ``lin2``'s.

    The entry leg must be a known, trimmable distance. With ``cut_until`` the
    EXIT leg may instead be a sensor-bounded ``.until()`` leg (kept untrimmed) —
    e.g. ``drive_backward(18) + strafe_left().until(over_line)``.
    """
    if not _trimmable(lin1, cut_m) or not _exit_eligible(lin2, cut_m, cut_until):
        return None
    # Perpendicular legs only: exactly one Forward and one Lateral. Same-axis
    # pairs are either a straight run (merged away) or a reversal — not a corner.
    if lin1.axis == lin2.axis:
        return None

    theta = math.pi / 2.0  # forward↔lateral is always a 90° path deflection
    radius_m = cut_m / math.tan(theta / 2.0)  # == cut_m

    new_lin1 = _trim_toward_corner(lin1, cut_m)
    new_lin2 = _trim_toward_corner(lin2, cut_m)
    crab_seg = Segment(
        kind="crab_arc",
        radius_m=radius_m,
        arc_angle_rad=theta,
        speed_scale=min(lin1.speed_scale, lin2.speed_scale),
        crab_from=_body_dir(lin1),
        crab_to=_body_dir(lin2),
        has_known_endpoint=True,
    )
    return new_lin1, crab_seg, new_lin2


def run_corner_cut(
    nodes: list[PathNode | None],
    cut_m: float,
    cut_until: bool = False,
) -> list[PathNode | None]:
    """Replace turn corners with arcs and crab corners with crab arcs.

    At each position a ``linear+turn+linear`` turn corner is tried first, then a
    ``linear+linear`` crab corner. Non-blocking (background / ephemeral) side
    actions between the legs are skipped over for the match and re-emitted
    concurrently with the arc. Blocking side actions and deferred placeholders
    are barriers.

    With ``cut_until`` the EXIT leg of a corner may be a sensor-bounded
    ``.until()`` leg (kept untrimmed, runs from the arc's end until its
    condition). The entry leg always needs a known, trimmable distance.
    """
    result: list[PathNode | None] = []
    i = 0
    while i < len(nodes):
        # 1. Turn corner: linear + turn + linear.
        match = _match_corner(nodes, i)
        if match is not None:
            lin1, bgs_a, turn, bgs_b, lin2, next_i = match
            cut = try_corner_arc(lin1, turn, lin2, cut_m, cut_until)
            if cut is not None:
                _emit_cut(result, cut, bgs_a + bgs_b)
                _log.debug(
                    "path optimizer: turn corner cut at index %d "
                    "(cut=%.3fm, radius=%.3fm, angle=%.1f°, %d bg actions kept)",
                    i,
                    cut_m,
                    cut[1].radius_m or 0,
                    math.degrees(cut[1].arc_angle_rad or 0),
                    len(bgs_a) + len(bgs_b),
                )
                i = next_i
                continue

        # 2. Crab corner: linear(Forward) + linear(Lateral) — no turn.
        crab_match = _match_crab_corner(nodes, i)
        if crab_match is not None:
            lin1, bgs, lin2, next_i = crab_match
            cut = try_crab_arc(lin1, lin2, cut_m, cut_until)
            if cut is not None:
                _emit_cut(result, cut, bgs)
                _log.debug(
                    "path optimizer: crab corner cut at index %d "
                    "(cut=%.3fm, radius=%.3fm, %d bg actions kept)",
                    i,
                    cut_m,
                    cut[1].radius_m or 0,
                    len(bgs),
                )
                i = next_i
                continue

        result.append(nodes[i])
        i += 1
    return result


def _emit_cut(
    result: list[PathNode | None],
    cut: tuple[Segment, Segment, Segment],
    bgs: list[SideAction],
) -> None:
    """Append a cut corner: trimmed lin1, kept background actions, arc, lin2.

    Background side actions launched around the corner keep running through the
    arc, so they are re-emitted before it (order preserved). A known leg trimmed
    to (near) zero length is dropped; a sensor-bounded leg (no known distance) is
    always kept.
    """

    def _nonzero(seg: Segment) -> bool:
        return seg.distance_m is None or abs(seg.distance_m) > 1e-4

    new_lin1, arc_seg, new_lin2 = cut
    if _nonzero(new_lin1):
        result.append(new_lin1)
    result.extend(bgs)
    result.append(arc_seg)
    if _nonzero(new_lin2):
        result.append(new_lin2)


class CornerCutPass:
    """Compiler pass that rounds turn corners (arcs) and crab corners (crab arcs)."""

    name = "corner_cut"

    def __init__(self, cut_m: float, cut_until: bool = False):
        if cut_m <= 0.0:
            msg = f"CornerCutPass: cut_m must be positive, got {cut_m}"
            raise ValueError(msg)
        self.cut_m = cut_m
        self.cut_until = cut_until

    def run(self, nodes: list[PathNode | None]) -> list[PathNode | None]:
        return run_corner_cut(nodes, self.cut_m, self.cut_until)
