"""Corner-cut pass — replace ``linear + turn + linear`` with ``linear + arc + linear``.

Cuts ``cut_m`` from each straight leg at a known-angle corner and inserts a
circular arc of radius ``R = cut_m / tan(|θ| / 2)``.  Only triples with known
endpoints, no conditions, and matching axis/direction on the linears are
eligible.  ``SideAction`` and deferred-placeholder (``None``) entries act as
barriers.
"""

from __future__ import annotations

import logging
import math
from dataclasses import replace

from raccoon.motion import LinearAxis

from ..ir import PathNode, Segment

_log = logging.getLogger(__name__)


def try_corner_arc(
    lin1: Segment,
    turn: Segment,
    lin2: Segment,
    cut_m: float,
) -> tuple[Segment, Segment, Segment] | None:
    """Return ``(new_lin1, arc, new_lin2)`` for a corner cut, or ``None``.

    Geometry: cutting ``cut_m`` from each leg at the corner requires an arc of
    radius ``R = cut_m / tan(|θ| / 2)``.  Both linears must be the same axis
    and direction and have enough distance to accommodate the cut.
    """
    if (
        turn.kind != "turn"
        or not turn.has_known_endpoint
        or turn.condition is not None
        or turn.angle_rad is None
    ):
        return None
    if (
        lin1.kind != "linear"
        or not lin1.has_known_endpoint
        or lin1.condition is not None
        or lin1.distance_m is None
    ):
        return None
    if (
        lin2.kind != "linear"
        or not lin2.has_known_endpoint
        or lin2.condition is not None
        or lin2.distance_m is None
    ):
        return None
    if lin1.axis != lin2.axis or lin1.sign != lin2.sign:
        return None

    theta = turn.angle_rad
    if abs(theta) < 1e-4:
        return None

    # Both linears must be long enough to accommodate the cut.
    if abs(lin1.distance_m) <= cut_m or abs(lin2.distance_m) <= cut_m:
        return None

    radius_m = cut_m / math.tan(abs(theta) / 2.0)

    # Trim each linear: new_d = signed_distance - sign * cut_m
    new_d1 = lin1.distance_m - lin1.sign * cut_m
    new_d2 = lin2.distance_m - lin2.sign * cut_m

    new_lin1 = replace(lin1, distance_m=new_d1)
    new_lin2 = replace(lin2, distance_m=new_d2)
    arc_seg = Segment(
        kind="arc",
        radius_m=radius_m,
        arc_angle_rad=theta,
        speed_scale=min(lin1.speed_scale, turn.speed_scale, lin2.speed_scale),
        lateral=(lin1.axis == LinearAxis.Lateral),
        has_known_endpoint=True,
    )
    return new_lin1, arc_seg, new_lin2


def run_corner_cut(
    nodes: list[PathNode | None],
    cut_m: float,
) -> list[PathNode | None]:
    """Replace ``linear+turn+linear`` triples with ``linear+arc+linear``."""
    result: list[PathNode | None] = []
    i = 0
    while i < len(nodes):
        # Peek ahead for a three-segment pattern.
        if (
            i + 2 < len(nodes)
            and isinstance(nodes[i], Segment)
            and isinstance(nodes[i + 1], Segment)
            and isinstance(nodes[i + 2], Segment)
        ):
            cut = try_corner_arc(nodes[i], nodes[i + 1], nodes[i + 2], cut_m)  # type: ignore[arg-type]
            if cut is not None:
                new_lin1, arc_seg, new_lin2 = cut
                if abs(new_lin1.distance_m) > 1e-4:  # type: ignore[arg-type]
                    result.append(new_lin1)
                result.append(arc_seg)
                if abs(new_lin2.distance_m) > 1e-4:  # type: ignore[arg-type]
                    result.append(new_lin2)
                i += 3
                _log.debug(
                    "path optimizer: corner cut at index %d "
                    "(cut=%.3fm, radius=%.3fm, angle=%.1f°)",
                    i - 3,
                    cut_m,
                    arc_seg.radius_m or 0,
                    math.degrees(arc_seg.arc_angle_rad or 0),
                )
                continue
        result.append(nodes[i])
        i += 1
    return result


class CornerCutPass:
    """Compiler pass that replaces straight-turn-straight corners with arcs."""

    name = "corner_cut"

    def __init__(self, cut_m: float):
        if cut_m <= 0.0:
            msg = f"CornerCutPass: cut_m must be positive, got {cut_m}"
            raise ValueError(msg)
        self.cut_m = cut_m

    def run(self, nodes: list[PathNode | None]) -> list[PathNode | None]:
        return run_corner_cut(nodes, self.cut_m)
