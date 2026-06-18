"""Motion path compilation pipeline.

The pipeline turns a tree of Step objects into an executable plan through a
sequence of compiler passes (lowering, merge, corner-cut, spline, ...) and
runs it via PathExecutor.

Public API:
    Segment, SideAction, PathNode               — IR types
    CompilerPass, CompiledPlan, PathCompiler    — compilation
    PathExecutor                                 — runtime
    create_motion                                — motion factory
"""

from __future__ import annotations

from .ir import (
    Segment,
    SideAction,
    PathNode,
    SENTINEL_DISTANCE_M,
)
from .compiler import CompilerPass, CompiledPlan, PathCompiler
from .optimize import (
    Optimizer,
    Representation,
    PathBuildError,
    optimize,
)
from .executor import PathExecutor
from .motion_factory import (
    create_motion,
    LineFollowAdapter,
    SplineAdapter,
    OVERSHOOT_M,
    OVERSHOOT_RAD,
)

__all__ = [
    # IR (relative)
    "Segment",
    "SideAction",
    "PathNode",
    "SENTINEL_DISTANCE_M",
    # Compilation
    "CompilerPass",
    "CompiledPlan",
    "PathCompiler",
    # Fluent optimizer builder
    "Optimizer",
    "Representation",
    "PathBuildError",
    "optimize",
    # Executor
    "PathExecutor",
    # Motion factory
    "create_motion",
    "LineFollowAdapter",
    "SplineAdapter",
    "OVERSHOOT_M",
    "OVERSHOOT_RAD",
]
