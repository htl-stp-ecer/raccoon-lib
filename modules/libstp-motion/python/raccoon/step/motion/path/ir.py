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
from typing import Optional, Union, TYPE_CHECKING

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
    axis: Optional[LinearAxis] = None
    sign: float = 1.0
    distance_m: Optional[float] = None  # None = condition-only (sentinel)
    speed_scale: float = 1.0
    heading_deg: Optional[float] = None

    # Turn params
    angle_rad: Optional[float] = None

    # Arc params
    radius_m: Optional[float] = None
    arc_angle_rad: Optional[float] = None
    lateral: bool = False

    # Common
    condition: Optional["StopCondition"] = None
    has_known_endpoint: bool = True

    # Opaque steps (follow_line, spline): store the step for adapter creation.
    # Excluded from equality so two segments with equivalent geometry compare
    # equal even if they originated from different opaque step instances.
    opaque_step: Optional["Step"] = field(default=None, compare=False)


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
PathNode = Union[Segment, SideAction]


@dataclass
class Correction:
    """World-frame parameter adjustment for the next segment.

    Produced by middleware (e.g. ``WorldCorrectionMiddleware``) and consumed
    by the motion factory when constructing the next segment's controller.
    """

    distance_adjust_m: float = 0.0
    """Subtract from linear distance target (positive = overshot, drive less)."""

    heading_target_rad: Optional[float] = None
    """Absolute heading to hold during the next linear segment (cross-track bias)."""

    angle_adjust_rad: float = 0.0
    """Subtract from turn/arc angle target (positive = over-rotated, turn less)."""
