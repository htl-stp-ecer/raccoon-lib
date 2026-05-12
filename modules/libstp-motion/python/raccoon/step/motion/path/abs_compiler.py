"""Absolute-pose plan compiler — Phase 3, Commit B.

This module implements the **dummes Desugaring 1:1** step described in
``docs/design/absolute-motion-plan.md`` (section "Schritt 1 — dummes
Desugaring"). It takes a sequence of absolute IR nodes
(:class:`Goto` / :class:`TurnTo` / :class:`Resync` / :class:`Action`) and
returns a frozen :class:`CompiledAbsolutePlan` ready for the executor.

What this compiler does
-----------------------

* Validate node types — anything outside the ``AbsoluteNode`` union is a
  :class:`TypeError`.
* Wrap the node sequence into a frozen, hashable container.

What this compiler does **not** do
----------------------------------

* No optimizer passes. Folding implicit turns, validating reachability,
  merging collinear gotos, heading inference, action lifting — all live in
  Phase 5. The Phase-3 plan is deliberate: **dumb desugaring is auditable,
  the optimizer works on a well-defined IR.**
* No execution. The executor lands in Phase-3 Commit C.
* No interplay with the relative :class:`PathCompiler` / :class:`CompiledPlan`
  pipeline in :mod:`compiler` — the absolute pipeline is independent.

The ``passes_applied`` field on :class:`CompiledAbsolutePlan` exists so
Phase 5 can record which optimizer passes ran without changing the public
shape; in Phase 3 it is always the empty tuple.
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass, field

from .abs_ir import AbsoluteNode, Action, Goto, Resync, TurnTo

# Concrete tuple of accepted node classes — used for both isinstance checks
# and clear error messages. Kept in sync with ``AbsoluteNode`` in abs_ir.
_NODE_TYPES: tuple[type, ...] = (Goto, TurnTo, Resync, Action)


@dataclass(frozen=True)
class CompiledAbsolutePlan:
    """Result of compiling an absolute plan.

    Attributes:
        nodes: The validated IR nodes in source order. Stored as a
            ``tuple`` so the whole plan is hashable / immutable.
        passes_applied: Names of optimizer passes that ran, in order.
            Empty in Phase 3 (dumb desugaring); populated in Phase 5.
    """

    nodes: tuple[AbsoluteNode, ...]
    passes_applied: tuple[str, ...] = field(default_factory=tuple)

    def __repr__(self) -> str:  # deterministic, line-per-node, snapshot-friendly
        if not self.nodes:
            return f"CompiledAbsolutePlan(nodes=(), passes_applied={self.passes_applied!r})"
        body = "\n".join(f"  {n!r}," for n in self.nodes)
        return (
            "CompiledAbsolutePlan(\n"
            f"  passes_applied={self.passes_applied!r},\n"
            "  nodes=(\n"
            f"{body}\n"
            "  ),\n"
            ")"
        )


def compile_plan(nodes: Sequence[AbsoluteNode]) -> CompiledAbsolutePlan:
    """Compile a flat absolute IR node list into a :class:`CompiledAbsolutePlan`.

    Phase-3 contract: this is a 1:1 desugaring. The function validates that
    every entry is one of ``Goto`` / ``TurnTo`` / ``Resync`` / ``Action`` and
    wraps them into an immutable container. No transformation, no merging,
    no folding. Optimizer passes are Phase 5.

    Args:
        nodes: Sequence of absolute IR nodes. Empty is allowed (yields an
            empty plan).

    Returns:
        A frozen :class:`CompiledAbsolutePlan` with the nodes stored as a
        tuple in source order and ``passes_applied=()``.

    Raises:
        TypeError: If any element is not an :data:`AbsoluteNode` instance.
    """
    validated: list[AbsoluteNode] = []
    for index, node in enumerate(nodes):
        if not isinstance(node, _NODE_TYPES):
            type_names = ", ".join(t.__name__ for t in _NODE_TYPES)
            msg = (
                f"compile_plan: node at index {index} has type "
                f"{type(node).__name__!r}; expected one of: {type_names}."
            )
            raise TypeError(msg)
        validated.append(node)

    return CompiledAbsolutePlan(nodes=tuple(validated), passes_applied=())


__all__ = [
    "CompiledAbsolutePlan",
    "compile_plan",
]
