"""Motion path compilation pipeline.

The pipeline turns a tree of Step objects into an executable plan through a
sequence of compiler passes (lowering, merge, corner-cut, spline, ...) and
runs it via PathExecutor with optional segment middlewares (world-frame
correction, telemetry, ...).

Public API:
    Segment, SideAction, PathNode, Correction   — IR types
    CompilerPass, CompiledPlan, PathCompiler    — compilation
    PathExecutor                                 — runtime
    PathMiddleware, WorldCorrectionMiddleware   — middleware protocol + builtin
    create_motion                                — motion factory
"""

from __future__ import annotations

from .ir import (
    Segment,
    SideAction,
    PathNode,
    Correction,
    SENTINEL_DISTANCE_M,
)
from .compiler import CompilerPass, CompiledPlan, PathCompiler
from .executor import PathExecutor
from .motion_factory import (
    create_motion,
    LineFollowAdapter,
    SplineAdapter,
    OVERSHOOT_M,
    OVERSHOOT_RAD,
)
from .middleware import (
    PathMiddleware,
    WorldCorrectionMiddleware,
    WorldPoseTracker,
)

__all__ = [
    # IR
    "Segment",
    "SideAction",
    "PathNode",
    "Correction",
    "SENTINEL_DISTANCE_M",
    # Compilation
    "CompilerPass",
    "CompiledPlan",
    "PathCompiler",
    # Executor
    "PathExecutor",
    # Motion factory
    "create_motion",
    "LineFollowAdapter",
    "SplineAdapter",
    "OVERSHOOT_M",
    "OVERSHOOT_RAD",
    # Middleware
    "PathMiddleware",
    "WorldCorrectionMiddleware",
    "WorldPoseTracker",
]
