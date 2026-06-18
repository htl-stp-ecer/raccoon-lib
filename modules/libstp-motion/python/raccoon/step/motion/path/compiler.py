"""PathCompiler — turns a list of Steps into a CompiledPlan.

The compiler is a thin orchestrator: it lowers the input Step tree into the
IR via the lowering pass, then runs each subsequent CompilerPass in sequence
on the resulting node list.  The result is a ``CompiledPlan`` that the
``PathExecutor`` knows how to run.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Protocol, runtime_checkable

from .abs_compiler import CompiledAbsolutePlan
from .abs_desugar import compile_relative_to_absolute, nodes_to_absolute
from .abs_passes import CompileError, validate_reachable
from .ir import PathNode
from .passes import flatten_steps

if TYPE_CHECKING:
    from ...logic.defer import Defer


@runtime_checkable
class CompilerPass(Protocol):
    """Pure transform on the path IR.

    A pass takes a node list and returns a (possibly different) node list.
    Passes must preserve the "deferred placeholder" (``None``) entries so
    the executor can resolve them at runtime.
    """

    name: str

    def run(
        self,
        nodes: list[PathNode | None],
    ) -> list[PathNode | None]: ...


@dataclass
class CompiledPlan:
    """Result of compiling a Step tree.

    ``nodes``        — the path IR after all passes ran.  Spline mode lowers
                       to a single ``Segment(kind="spline")`` node here, so the
                       executor walks it through the unified loop like any
                       other opaque segment.
    ``deferred``     — ``(index, Defer)`` pairs for runtime resolution.
    ``passes_applied`` — names of passes that ran, for diagnostics.
    """

    nodes: list[PathNode | None]
    deferred: list[tuple[int, "Defer"]]
    passes_applied: list[str] = field(default_factory=list)
    absolute_plan: "CompiledAbsolutePlan | None" = None
    fallback_reason: str | None = None


class PathCompiler:
    """Orchestrates the compiler pipeline.

    Lowering is always run first (it produces the IR from the Step tree).
    The remaining passes are applied in the order given.

    Example:
        >>> from raccoon.step.motion.path.passes import MergePass, CornerCutPass
        >>> compiler = PathCompiler([MergePass(), CornerCutPass(0.05)])
        >>> plan = compiler.compile([drive(50), turn(90), drive(30)])
    """

    def __init__(self, passes: list[CompilerPass] | None = None) -> None:
        self._passes: list[CompilerPass] = list(passes or [])

    def compile(self, steps: list) -> CompiledPlan:
        """Lower ``steps`` to IR and run all configured passes."""
        nodes, deferred = flatten_steps(steps)
        applied: list[str] = ["lowering"]

        for p in self._passes:
            nodes = p.run(nodes)
            applied.append(p.name)

        return CompiledPlan(
            nodes=nodes,
            deferred=deferred,
            passes_applied=applied,
        )

    def compile_via_absolute_bridge(self, steps: list) -> CompiledPlan:
        """Prefer the Phase-5 absolute runtime path, then fall back explicitly.

        We still compile the legacy node list for backwards-compatible
        diagnostics/signatures and as the explicit fallback path. When the
        absolute compiler succeeds, the executor consumes ``absolute_plan``
        directly instead of bridging back into legacy segments.
        """
        legacy = self.compile(steps)
        if legacy.deferred:
            legacy.fallback_reason = (
                "compile_relative_to_absolute: Defer steps are not supported by the "
                "absolute IR path yet"
            )
            return legacy

        try:
            if self._passes:
                abs_nodes = nodes_to_absolute(legacy.nodes)
                abs_nodes = validate_reachable(abs_nodes)
                abs_plan = CompiledAbsolutePlan(
                    nodes=tuple(abs_nodes),
                    passes_applied=("relative_desugar_from_ir", "validate_reachable"),
                )
            else:
                abs_plan = compile_relative_to_absolute(
                    steps,
                    validate=True,
                    fold_turns=False,
                )
        except CompileError as exc:
            legacy.fallback_reason = str(exc)
            return legacy

        applied = [f"absolute:{name}" for name in abs_plan.passes_applied]
        for name in legacy.passes_applied[1:]:
            applied.append(name)

        return CompiledPlan(
            nodes=legacy.nodes,
            deferred=legacy.deferred,
            passes_applied=applied,
            absolute_plan=abs_plan,
        )
