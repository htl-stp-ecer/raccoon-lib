"""Compiler passes for the motion path pipeline.

Each pass is a callable transform from one IR node list to another.  The
default pipeline runs lowering → merge → corner-cut, with spline as an
opt-in terminal transform.

Public API:
    LoweringPass, MergePass, CornerCutPass, SplineConversionPass — pass classes
    optimize_nodes(nodes, *, merge, corner_cut_m)                — legacy helper
"""

from __future__ import annotations

from typing import Optional

from ..ir import PathNode
from .lowering import (
    extract_segment,
    resolve_step,
    is_same_type,
    flatten_one,
    flatten_parallel,
    flatten_steps,
)
from .merge import can_merge, merge_two, run_merge, MergePass
from .corner_cut import try_corner_arc, run_corner_cut, CornerCutPass
from .spline import segments_to_spline_waypoints, build_spline_step

__all__ = [
    # Lowering
    "extract_segment", "resolve_step", "is_same_type",
    "flatten_one", "flatten_parallel", "flatten_steps",
    # Merge
    "can_merge", "merge_two", "run_merge", "MergePass",
    # Corner cut
    "try_corner_arc", "run_corner_cut", "CornerCutPass",
    # Spline
    "segments_to_spline_waypoints", "build_spline_step",
    # Orchestrator
    "optimize_nodes",
]


def optimize_nodes(
    nodes: list[Optional[PathNode]],
    *,
    merge: bool,
    corner_cut_m: float,
) -> list[Optional[PathNode]]:
    """Apply optimization passes to a flattened path node list.

    ``SideAction`` and ``None`` (deferred) barriers are preserved and never
    optimized across — they pin the execution point of side effects and
    runtime-resolved steps.

    Passes (applied in order when enabled):

    1. **Merge** (``merge=True``): collapse adjacent same-type segments that
       have no conditions and known endpoints.  Same-axis/same-direction
       linears sum their distances; consecutive turns sum their angles.

    2. **Corner cut** (``corner_cut_m > 0``): replace ``linear + turn + linear``
       triples with ``linear + arc + linear``.  The arc radius is derived from
       the cut distance via ``R = cut_m / tan(|θ| / 2)``.
    """
    if not merge and corner_cut_m <= 0.0:
        return nodes

    result: list[Optional[PathNode]] = list(nodes)
    if merge:
        result = run_merge(result)
    if corner_cut_m > 0.0:
        result = run_corner_cut(result, corner_cut_m)
    return result
