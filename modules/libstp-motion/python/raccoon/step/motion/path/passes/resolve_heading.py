"""``resolve_heading`` pass — stamp compile-time angles onto heading turns.

``TurnToHeading`` lowers to an OPAQUE turn segment (``angle_rad=None``): its
relative angle is only known at ``on_start``, when the ``HeadingReferenceService``
computes the shortest-path delta from the live heading. That opacity blocks
every geometry pass — most importantly ``corner_cut``, which needs a known
``angle_rad`` to compute the arc radius — so a mission written in the
recommended drift-robust style (``turn_to_heading_* `` + ``heading=`` pins)
silently gets ZERO benefit from ``optimize(...).cut_corners(...)``.

Path compilation happens inside ``Optimizer._execute_step`` — immediately
before the path runs — so the robot's CURRENT heading is the path-start
heading. This pass integrates a running absolute heading through the node
list (exactly like ``absolute_heading`` / ``to_absolute``) and, wherever the
running heading is provably known, computes the ``TurnToHeading`` delta at
compile time and stamps it onto the segment as ``angle_rad``.

The segment KEEPS its ``opaque_step``: a stamped turn that no later pass
consumes still executes as the original drift-corrected ``TurnToHeading``
(the executor prefers the opaque adapter). Only a pass that geometrically
REPLACES the turn — ``corner_cut`` folding it into an arc — uses the stamp,
which is exactly the trade corner cutting always makes (open-loop arc,
downstream ``heading=`` pins re-correct).

Running-heading bookkeeping (absolute radians, CCW-positive):

* start: live heading at compile time (known)
* ``turn`` with known ``angle_rad`` — advance
* ``TurnToHeading`` — after it, heading == its absolute target, so it
  RE-ESTABLISHES a known heading even when the current one is unknown
  (the turn itself is only stamped when the ENTRY heading is known)
* ``arc`` with known ``arc_angle_rad`` — advance; ``crab_arc`` — unchanged
* ``linear`` / ``diagonal`` with ``heading_deg`` pin — heading := pin (known)
* ``linear`` / ``diagonal`` without pin — heading held, unchanged
* ``follow_line`` without pin, ``spline`` — unknown
* ``SideAction`` / deferred ``None`` — unknown (a side action may drive)

If no heading reference is marked (or no robot is available, e.g. in
``explain()``), the pass is a no-op.
"""

from __future__ import annotations

import logging
import math
from dataclasses import replace
from typing import TYPE_CHECKING

from ..ir import PathNode, Segment, SideAction

if TYPE_CHECKING:
    from raccoon.robot.api import GenericRobot

_log = logging.getLogger(__name__)

_TWO_PI = 2.0 * math.pi


def _normalize_angle(angle: float) -> float:
    """Normalize angle to [-pi, pi]."""
    while angle > math.pi:
        angle -= _TWO_PI
    while angle < -math.pi:
        angle += _TWO_PI
    return angle


class ResolveHeadingTurnsPass:
    """Stamp known relative angles onto ``TurnToHeading`` segments."""

    name = "resolve_heading"

    def __init__(self, robot: "GenericRobot | None" = None) -> None:
        self._robot = robot

    def run(self, nodes: list[PathNode | None]) -> list[PathNode | None]:
        if self._robot is None:
            return nodes

        from raccoon.robot.heading_reference import (
            HeadingReferenceService,
            TurnDirection,
            _world_heading,
        )

        from ...heading_reference import TurnToHeading

        service = self._robot.get_service(HeadingReferenceService)
        if service.reference_deg is None:
            return nodes

        current_abs = _world_heading(self._robot)
        known = True
        resolved = 0

        out: list[PathNode | None] = []
        for node in nodes:
            if node is None or isinstance(node, SideAction):
                known = False
                out.append(node)
                continue
            if not isinstance(node, Segment):
                out.append(node)
                continue

            seg = node

            if (
                seg.kind == "turn"
                and isinstance(seg.opaque_step, TurnToHeading)
                and seg.angle_rad is None
                and seg.condition is None
            ):
                step = seg.opaque_step
                target_abs = service.target_absolute_rad(step._target_deg)
                if known:
                    rel = _normalize_angle(target_abs - current_abs)
                    direction = TurnDirection.coerce(step._force_direction)
                    if direction is TurnDirection.LEFT and rel < 0:
                        rel += _TWO_PI
                    elif direction is TurnDirection.RIGHT and rel > 0:
                        rel -= _TWO_PI
                    seg = replace(seg, angle_rad=rel, sign=1.0 if rel >= 0 else -1.0)
                    resolved += 1
                    _log.debug(
                        "resolve_heading: TurnToHeading(target=%.1f°) resolved to "
                        "%.1f° relative at compile time",
                        step._target_deg,
                        math.degrees(rel),
                    )
                # Whether or not the entry heading was known, AFTER this turn
                # the robot faces the absolute target.
                current_abs = target_abs
                known = True
                out.append(seg)
                continue

            if seg.kind == "turn":
                if seg.angle_rad is not None and seg.condition is None:
                    current_abs += seg.angle_rad
                else:
                    known = False
            elif seg.kind == "arc":
                if seg.arc_angle_rad is not None:
                    current_abs += seg.arc_angle_rad
                else:
                    known = False
            elif seg.kind == "crab_arc":
                pass  # constant-heading blend
            elif seg.kind in ("linear", "diagonal", "follow_line"):
                if seg.heading_deg is not None:
                    current_abs = service.target_absolute_rad(seg.heading_deg)
                    known = True
                elif seg.kind == "follow_line":
                    # A line follow may rotate the chassis arbitrarily.
                    known = False
            else:  # spline / unknown kinds
                known = False

            out.append(seg)

        if resolved:
            _log.debug("resolve_heading: %d heading turn(s) resolved", resolved)
        return out
