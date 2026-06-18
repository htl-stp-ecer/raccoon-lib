"""Lowering pass — turn a tree of Step objects into the path IR.

This is the entry point of the pipeline.  It walks the Step tree, recursively
flattens ``Sequential`` and ``Parallel`` composites into a linear node list,
identifies the motion spine inside parallel branches, and produces:

- ``list[Optional[PathNode]]`` — flat path nodes (Segments, SideActions, or
  ``None`` placeholders for deferred steps).
- ``list[tuple[int, Defer]]`` — deferred entries to resolve at runtime,
  keyed by their index in the node list.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

from raccoon.motion import LinearAxis

from ..ir import PathNode, Segment, SideAction
from .known_distance import recover_known_distance

if TYPE_CHECKING:
    from .... import Step
    from ....logic.defer import Defer


def extract_segment(step: "Step") -> Segment:
    """Extract motion IR from a resolved step via its ``lower_to_segments()``.

    A step that is not optimizer-supported returns ``None`` and is rejected
    here with ``TypeError`` so ``flatten_one`` can treat it as a side action.
    """
    segs = step.lower_to_segments()
    if not segs:
        msg = (
            f"smooth_path() does not support {type(step).__name__} as a motion "
            f"step. Supported motion steps: drive, turn, arc, follow_line, "
            f"follow_line_single, spline."
        )
        raise TypeError(msg)
    if len(segs) != 1:
        # Current pipeline assumes one segment per leaf step; multi-segment
        # lowering is a future extension.
        msg = f"{type(step).__name__}.lower_to_segments() returned {len(segs)} segments; expected 1"
        raise TypeError(msg)
    # Always recover a known travel distance hidden in a bare after_cm
    # condition — there is no reason the optimizer should see it as unknown.
    return recover_known_distance(segs[0])


def resolve_step(step) -> "Step":
    """Resolve a builder to a concrete Step instance."""
    if hasattr(step, "resolve"):
        return step.resolve()
    return step


def is_same_type(a: Segment, b: Segment) -> bool:
    """Check if two segments can use warm-start (same motion type).

    ``follow_line`` is treated as a forward linear for warm-start purposes:
    it runs the same ``LinearMotion`` axis internally, so velocity can be
    carried across ``linear(Forward) ↔ follow_line`` transitions.
    ``spline`` never warm-starts.
    """
    # Spline never warm-starts (SplineMotion has no start_warm).
    if a.kind == "spline" or b.kind == "spline":
        return False

    def _effective(seg: Segment) -> tuple:
        if seg.kind == "follow_line":
            return ("linear", LinearAxis.Forward)
        if seg.kind == "linear":
            return ("linear", seg.axis)
        return (seg.kind, None)

    return _effective(a) == _effective(b)


def flatten_one(
    step,
    nodes: list[PathNode | None],
    deferred: list[tuple[int, "Defer"]],
) -> None:
    """Recursively flatten ``step`` into path nodes.

    Appends to ``nodes`` in-place.  Deferred steps become ``None``
    placeholders with entries in ``deferred`` for runtime resolution.
    """
    # `....` = step/ (parent of motion/).
    from ....logic.background import Background
    from ....logic.defer import Defer, Run
    from ....parallel import Parallel
    from ....sequential import Sequential

    # 1. Resolve builder (e.g., drive_forward(30) or defer(...) returns a
    #    builder). Must happen before the isinstance checks below so that
    #    builder factories such as defer() are matched as their concrete
    #    step type (e.g. Defer) rather than slipping through as side actions.
    step = resolve_step(step)

    # 2. Defer — placeholder for runtime resolution.
    if isinstance(step, Defer):
        deferred.append((len(nodes), step))
        nodes.append(None)
        return

    # 3. Sequential — flatten children recursively.
    if isinstance(step, Sequential):
        for child in step.steps:
            flatten_one(child, nodes, deferred)
        return

    # 4. Parallel — find motion spine, side-effect branches.
    if isinstance(step, Parallel):
        flatten_parallel(step, nodes, deferred)
        return

    # 5. Background — non-blocking side action.
    if isinstance(step, Background):
        nodes.append(SideAction(step=step._step, is_background=True))
        return

    # 6. Run — inline side action (quick callable).
    if isinstance(step, Run):
        nodes.append(SideAction(step=step, is_background=False))
        return

    # 7. Try to extract as a motion segment.
    try:
        seg = extract_segment(step)
        nodes.append(seg)
        return
    except TypeError:
        pass

    # 8. Non-motion step — check if it uses the drive resource.
    resources = step.collected_resources()
    if "drive" in resources:
        msg = (
            f"smooth_path() does not support {type(step).__name__} — "
            f"it uses the drive resource but is not a supported motion step. "
            f"Supported: drive, turn, arc, follow_line, follow_line_single, spline."
        )
        raise TypeError(msg)

    # 9. Non-drive step — treat as inline side action.
    nodes.append(SideAction(step=step, is_background=False))


def flatten_parallel(
    par,
    nodes: list[PathNode | None],
    deferred: list[tuple[int, "Defer"]],
) -> None:
    """Flatten a Parallel step, identifying the motion spine branch."""
    from ....sequential import Sequential

    spine_idx = None
    for i, branch in enumerate(par.steps):
        branch_resources = branch.collected_resources()
        if "drive" in branch_resources:
            if spine_idx is not None:
                msg = (
                    "smooth_path(): parallel() has multiple branches using "
                    "the drive resource — only one motion spine is allowed"
                )
                raise TypeError(msg)
            spine_idx = i

    if spine_idx is None:
        # No motion branch — entire parallel is a side-effect.
        nodes.append(SideAction(step=par, is_background=False))
        return

    # Side-effect branches: launch as background tasks at this point.
    for i, branch in enumerate(par.steps):
        if i != spine_idx:
            nodes.append(SideAction(step=branch, is_background=True))

    # Flatten the motion spine branch.
    spine_branch = par.steps[spine_idx]
    if isinstance(spine_branch, Sequential):
        for child in spine_branch.steps:
            flatten_one(child, nodes, deferred)
    else:
        flatten_one(spine_branch, nodes, deferred)


def flatten_steps(
    steps: list,
) -> tuple[list[PathNode | None], list[tuple[int, "Defer"]]]:
    """Flatten a list of steps into a linear path.

    Returns:
        ``(nodes, deferred)`` where ``nodes`` is a flat list of ``Segment``,
        ``SideAction``, or ``None`` (deferred placeholder), and ``deferred``
        is a list of ``(index, Defer)`` pairs for runtime resolution.
    """
    nodes: list[PathNode | None] = []
    deferred: list[tuple[int, "Defer"]] = []
    for step in steps:
        flatten_one(step, nodes, deferred)
    return nodes, deferred
