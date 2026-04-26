"""PathCompiler — turns a list of Steps into a CompiledPlan.

The compiler is a thin orchestrator: it lowers the input Step tree into the
IR via the lowering pass, then runs each subsequent CompilerPass in sequence
on the resulting node list.  The result is a ``CompiledPlan`` that the
``PathExecutor`` knows how to run.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Protocol, runtime_checkable

from .ir import PathNode
from .passes import flatten_steps

if TYPE_CHECKING:
    from ...logic.defer import Defer
    from .. import Step


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

    ``nodes``        — the path IR after all passes ran.
    ``deferred``     — ``(index, Defer)`` pairs for runtime resolution.
    ``spline_step``  — when the spline-conversion pass terminated the
                       pipeline by replacing the path with a single
                       SplinePath, that step lives here.  Otherwise
                       ``None`` and the executor walks ``nodes`` normally.
    ``passes_applied`` — names of passes that ran, for diagnostics.
    """

    nodes: list[PathNode | None]
    deferred: list[tuple[int, "Defer"]]
    spline_step: "Step" | None = None
    passes_applied: list[str] = field(default_factory=list)


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
            spline_step=None,
            passes_applied=applied,
        )
