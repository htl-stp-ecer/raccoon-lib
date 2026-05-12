"""Optimizer passes for the absolute motion-plan IR."""

from __future__ import annotations

import math
from collections.abc import Sequence

from .abs_ir import AbsoluteNode, Goto, TurnTo


class CompileError(ValueError):
    """Raised when an absolute plan cannot be compiled safely."""


def _same_angle(a: float | None, b: float | None, *, tol: float = 1e-9) -> bool:
    if a is None or b is None:
        return False
    return abs(math.atan2(math.sin(a - b), math.cos(a - b))) <= tol


def fold_implicit_turns(nodes: Sequence[AbsoluteNode]) -> tuple[AbsoluteNode, ...]:
    """Drop ``TurnTo`` nodes immediately implied by the following ``Goto``.

    Side actions and resyncs are barriers because they pin observable work to
    a specific transition point. The pass is deliberately local and auditable.
    """
    folded: list[AbsoluteNode] = []
    index = 0
    while index < len(nodes):
        node = nodes[index]
        next_node = nodes[index + 1] if index + 1 < len(nodes) else None
        if (
            isinstance(node, TurnTo)
            and isinstance(next_node, Goto)
            and _same_angle(node.theta_rad, next_node.theta_rad)
        ):
            index += 1
            continue
        folded.append(node)
        index += 1
    return tuple(folded)


def validate_reachable(
    nodes: Sequence[AbsoluteNode],
    *,
    world_map=None,
) -> tuple[AbsoluteNode, ...]:
    """Validate that absolute targets stay inside the known table bounds.

    ``world_map`` is optional so pure unit tests and non-sim builds can still
    compile plans. When provided, it must expose centimetre bounds via either
    ``width_cm``/``height_cm`` or ``table_width_cm``/``table_height_cm``.
    """
    if world_map is None:
        return tuple(nodes)

    width_cm = getattr(world_map, "width_cm", None)
    height_cm = getattr(world_map, "height_cm", None)
    if width_cm is None:
        width_cm = getattr(world_map, "table_width_cm", None)
    if height_cm is None:
        height_cm = getattr(world_map, "table_height_cm", None)

    if width_cm is None or height_cm is None:
        msg = (
            "validate_reachable: world_map must expose width_cm/height_cm "
            "or table_width_cm/table_height_cm"
        )
        raise CompileError(msg)

    width_m = float(width_cm) / 100.0
    height_m = float(height_cm) / 100.0
    for index, node in enumerate(nodes):
        if not isinstance(node, Goto):
            continue
        if not (0.0 <= node.x_m <= width_m and 0.0 <= node.y_m <= height_m):
            msg = (
                "validate_reachable: Goto at index "
                f"{index} targets ({node.x_m:.3f}, {node.y_m:.3f}) m, "
                f"outside table bounds 0..{width_m:.3f} x 0..{height_m:.3f} m"
            )
            raise CompileError(msg)

    return tuple(nodes)


__all__ = [
    "CompileError",
    "fold_implicit_turns",
    "validate_reachable",
]
