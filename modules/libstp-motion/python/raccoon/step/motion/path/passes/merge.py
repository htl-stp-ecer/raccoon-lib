"""Merge pass — collapse adjacent same-type segments.

Two segments are merge-eligible if they share kind/axis/direction, both have
known endpoints, and neither has a stop condition.  ``SideAction`` and
deferred-placeholder (``None``) entries act as merge barriers.
"""

from __future__ import annotations

import math
from dataclasses import replace

from ..ir import PathNode, Segment


def _same_angle(a: float | None, b: float | None, *, tol: float = 1e-9) -> bool:
    if a is None and b is None:
        return True
    if a is None or b is None:
        return False
    return abs(math.atan2(math.sin(a - b), math.cos(a - b))) <= tol


def can_merge(a: Segment, b: Segment) -> bool:
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
            and _same_angle(a.target_heading_rad, b.target_heading_rad)
        )
    if a.kind == "turn" and b.kind == "turn":
        # An opaque heading turn (TurnToHeading) executes via its own step —
        # merging would keep only one opaque_step and silently drop the other
        # target. resolve_heading stamps angle_rad on these, so without this
        # guard they would suddenly become mergeable.
        if a.opaque_step is not None or b.opaque_step is not None:
            return False
        # Only merge if the combined angle stays non-zero.
        combined = (a.angle_rad or 0.0) + (b.angle_rad or 0.0)
        return abs(combined) > 1e-6
    return False


def merge_two(a: Segment, b: Segment) -> Segment:
    """Merge two compatible segments. Precondition: ``can_merge(a, b)``."""
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
        target_heading_rad=b.target_heading_rad,
    )


def run_merge(nodes: list[PathNode | None]) -> list[PathNode | None]:
    """Collapse adjacent same-type/same-direction segments with no conditions."""
    result: list[PathNode | None] = []
    i = 0
    while i < len(nodes):
        node = nodes[i]
        if isinstance(node, Segment):
            seg = node
            j = i + 1
            while j < len(nodes) and isinstance(nodes[j], Segment):
                if can_merge(seg, nodes[j]):  # type: ignore[arg-type]
                    seg = merge_two(seg, nodes[j])  # type: ignore[arg-type]
                    j += 1
                else:
                    break
            result.append(seg)
            i = j
        else:
            result.append(node)
            i += 1
    return result


class MergePass:
    """Compiler pass that runs the merge transform."""

    name = "merge"

    def run(self, nodes: list[PathNode | None]) -> list[PathNode | None]:
        return run_merge(nodes)
