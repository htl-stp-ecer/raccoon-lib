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
from .abs_ir import (
    Goto,
    TurnTo,
    Resync,
    Action,
    AbsoluteNode,
)
from .abs_factory import (
    goto,
    turn_to,
    resync,
    action,
)
from .compiler import CompilerPass, CompiledPlan, PathCompiler
from .optimize import (
    Optimizer,
    Representation,
    PathBuildError,
    optimize,
)
from .abs_compiler import CompiledAbsolutePlan, compile_plan
from .abs_desugar import (
    IntendedPose,
    absolute_to_relative_nodes,
    compile_relative_to_absolute,
    nodes_to_absolute,
)
from .abs_passes import CompileError, fold_implicit_turns, validate_reachable
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
    # IR (absolute, Phase 3)
    "Goto",
    "TurnTo",
    "Resync",
    "Action",
    "AbsoluteNode",
    # IR factories (absolute, cm/deg)
    "goto",
    "turn_to",
    "resync",
    "action",
    # Compilation
    "CompilerPass",
    "CompiledPlan",
    "PathCompiler",
    # Fluent optimizer builder
    "Optimizer",
    "Representation",
    "PathBuildError",
    "optimize",
    # Compilation (absolute, Phase 3)
    "CompiledAbsolutePlan",
    "compile_plan",
    "CompileError",
    "IntendedPose",
    "absolute_to_relative_nodes",
    "compile_relative_to_absolute",
    "nodes_to_absolute",
    "fold_implicit_turns",
    "validate_reachable",
    # Executor
    "PathExecutor",
    # Motion factory
    "create_motion",
    "LineFollowAdapter",
    "SplineAdapter",
    "OVERSHOOT_M",
    "OVERSHOOT_RAD",
]
