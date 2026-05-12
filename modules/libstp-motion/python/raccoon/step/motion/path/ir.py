"""Intermediate representation for the motion path pipeline.

Compiler passes operate on ``list[PathNode]``:

- ``Segment``    — one motion primitive (linear / turn / arc / follow_line / spline).
- ``SideAction`` — a non-motion step pinned to a transition point.
- ``None``       — placeholder for a deferred step resolved at runtime.

These types are intentionally pure data — they hold no robot reference and
no runtime state.  Anything stateful (current motion, world tracker) lives
in the executor / middleware layer.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import TYPE_CHECKING

from raccoon.motion import LinearAxis

if TYPE_CHECKING:
    # Path: step/motion/path/ir.py — `...` = step/.
    from ... import Step
    from ...condition import StopCondition


# Sentinel distance used by condition-based linear segments where the actual
# stopping distance isn't known until the condition fires.
SENTINEL_DISTANCE_M: float = 100.0


@dataclass
class Segment:
    """One motion primitive extracted from a Step into the pipeline IR.

    Field semantics depend on ``kind``:

    - ``"linear"``      — ``axis``, ``sign``, ``distance_m``, ``speed_scale``,
                          optional ``heading_deg`` and ``condition``.
    - ``"turn"``        — ``sign``, ``angle_rad``, ``speed_scale``,
                          optional ``condition``.
    - ``"arc"``         — ``radius_m``, ``arc_angle_rad``, ``speed_scale``,
                          ``lateral``.
    - ``"follow_line"`` — same shape as ``"linear"`` plus ``opaque_step``
                          (the original LineFollow step). Adapter delegates
                          lifecycle to the step's ``on_start``/``on_update``.
    - ``"spline"``      — ``opaque_step`` (the SplinePath step). No warm-start.
    """

    kind: str  # "linear" | "turn" | "arc" | "follow_line" | "spline"

    # Linear params
    axis: LinearAxis | None = None
    sign: float = 1.0
    distance_m: float | None = None  # None = condition-only (sentinel)
    speed_scale: float = 1.0
    heading_deg: float | None = None
    target_heading_rad: float | None = None

    # Turn params
    angle_rad: float | None = None

    # Arc params
    radius_m: float | None = None
    arc_angle_rad: float | None = None
    lateral: bool = False

    # Common
    condition: "StopCondition" | None = None
    has_known_endpoint: bool = True

    # Opaque steps (follow_line, spline): store the step for adapter creation.
    # Excluded from equality so two segments with equivalent geometry compare
    # equal even if they originated from different opaque step instances.
    opaque_step: "Step" | None = field(default=None, compare=False)


@dataclass
class SideAction:
    """A non-motion step pinned to a transition point in the path.

    ``is_background=True``  → fired as an asyncio task and not awaited inline.
    ``is_background=False`` → awaited inline before the next segment starts.
    """

    step: "Step"
    is_background: bool


# A concrete node in the compiled path.  Deferred steps are modelled as
# ``Optional[PathNode]`` (i.e. ``None``) at the list level rather than baked
# into ``PathNode`` itself, so type-narrowing on a non-None entry yields a
# concrete Segment-or-SideAction.
PathNode = Segment | SideAction
