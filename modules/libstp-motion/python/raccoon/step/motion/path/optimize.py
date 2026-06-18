"""Fluent path-optimizer builder — ``optimize([...])``.

A thin, fluent wrapper around the existing compiler + executor pipeline.
Where ``smooth_path()`` exposes optimization as boolean flags, ``optimize()``
exposes it as a chain of explicit passes — Java-stream style::

    optimize([drive_forward(50), turn_right(90), drive_forward(30)])
        .merge()
        .cut_corners(5)

The builder is itself a ``Step``: it compiles its raw steps through
``PathCompiler`` and runs the result through ``PathExecutor``, mirroring the
non-absolute, non-spline execution wiring of ``SmoothPath``.

A small compile-time state machine validates pass composition.  Each pass may
optionally declare the segment representation it ``requires`` / ``produces``
and whether it is ``terminal``; passes that don't declare these (the existing
``MergePass`` / ``CornerCutPass``) default to ``EITHER`` / ``SAME`` /
non-terminal and keep working unchanged.
"""

from __future__ import annotations

from enum import Enum
from typing import TYPE_CHECKING

from ... import Step
from .compiler import PathCompiler
from .executor import PathExecutor
from .ir import Segment, SideAction
from .passes import (
    CornerCutPass,
    MergePass,
    ToAbsolutePass,
    flatten_steps,
)

if TYPE_CHECKING:
    from raccoon.robot.api import GenericRobot


# ---------------------------------------------------------------------------
# Pass contract extension
# ---------------------------------------------------------------------------


class Representation(Enum):
    """Segment representation a pass consumes or produces.

    - ``RELATIVE`` — body-frame deltas (the default stream representation).
    - ``ABSOLUTE`` — world-frame waypoints.
    - ``EITHER``   — a pass that accepts whatever representation is current.
    - ``SAME``     — a pass that leaves the representation unchanged.
    """

    RELATIVE = "relative"
    ABSOLUTE = "absolute"
    EITHER = "either"
    SAME = "same"


# Module-level defaults for the (optional) extended pass contract.  A pass may
# declare ``requires`` / ``produces`` / ``terminal`` as class attributes; if it
# doesn't, these defaults apply.  Existing passes therefore behave as
# EITHER / SAME / non-terminal.
DEFAULT_REQUIRES = Representation.EITHER
DEFAULT_PRODUCES = Representation.SAME
DEFAULT_TERMINAL = False


def _pass_requires(p) -> Representation:
    return getattr(p, "requires", DEFAULT_REQUIRES)


def _pass_produces(p) -> Representation:
    return getattr(p, "produces", DEFAULT_PRODUCES)


def _pass_terminal(p) -> bool:
    return getattr(p, "terminal", DEFAULT_TERMINAL)


class PathBuildError(Exception):
    """Raised when passes are composed in an invalid order."""


# ---------------------------------------------------------------------------
# Node rendering for explain()
# ---------------------------------------------------------------------------


def _render_node(idx: int, node) -> str:
    """Compactly render one IR node for ``explain()``."""
    if node is None:
        return f"  [{idx}] None(deferred)"
    if isinstance(node, Segment):
        seg = node
        parts = [f"kind={seg.kind}"]
        if seg.kind in ("linear", "follow_line"):
            axis = getattr(seg.axis, "name", seg.axis)
            parts.append(f"axis={axis}")
            parts.append(f"distance_m={seg.distance_m}")
        elif seg.kind == "turn":
            parts.append(f"angle_rad={seg.angle_rad}")
        elif seg.kind == "arc":
            parts.append(f"radius_m={seg.radius_m}")
            parts.append(f"arc_angle_rad={seg.arc_angle_rad}")
        parts.append(f"has_known_endpoint={seg.has_known_endpoint}")
        return f"  [{idx}] Segment({', '.join(parts)})"
    if isinstance(node, SideAction):
        label = type(node.step).__name__
        mode = "bg" if node.is_background else "inline"
        return f"  [{idx}] SideAction({label}, {mode})"
    return f"  [{idx}] {node!r}"


def _render_nodes(nodes) -> str:
    if not nodes:
        return "  (empty)"
    return "\n".join(_render_node(i, n) for i, n in enumerate(nodes))


# ---------------------------------------------------------------------------
# Optimizer Step
# ---------------------------------------------------------------------------


class Optimizer(Step):
    """Fluent path optimizer — compiles motion steps through opt-in passes.

    Built via the ``optimize()`` factory.  Chain ``.merge()``,
    ``.cut_corners()`` or ``.apply(custom_pass)`` to append compiler passes;
    each returns ``self`` so calls compose Java-stream style.  Execution
    compiles the raw steps through ``PathCompiler`` and runs the result on
    ``PathExecutor`` (relative, non-spline path).
    """

    hz: int = 100

    def __init__(self, steps: list) -> None:
        super().__init__()
        self._raw_steps = steps
        self._passes: list = []
        self._repr = Representation.RELATIVE
        self._terminated_by: str | None = None

    _composite = True

    # -- compile-time state machine ----------------------------------------

    def _add(self, p) -> "Optimizer":
        """Append a pass, validating representation flow and termination."""
        if self._terminated_by is not None:
            msg = f"{p.name}() after terminal {self._terminated_by}()"
            raise PathBuildError(msg)

        req = _pass_requires(p)
        if req not in (self._repr, Representation.EITHER):
            msg = f"{p.name}() needs {req.name} segments, stream is " f"{self._repr.name} — reorder"
            raise PathBuildError(msg)

        self._passes.append(p)

        produces = _pass_produces(p)
        if produces != Representation.SAME:
            self._repr = produces

        if _pass_terminal(p):
            self._terminated_by = p.name

        return self

    # -- public chain methods ----------------------------------------------

    def merge(self) -> "Optimizer":
        """Collapse adjacent same-type/same-direction segments (``MergePass``)."""
        return self._add(MergePass())

    def to_absolute(self) -> "Optimizer":
        """Convert known-endpoint relative runs into closed-loop ``goto_relative`` legs.

        Replaces each maximal run of consecutive known-endpoint ``linear`` /
        ``turn`` segments with inline navigate-to-pose moves (``ToAbsolutePass``)
        — one ``goto_relative`` per linear endpoint, regulated on the
        localization particle filter so they shrug off odometry drift.
        ``.until(after_cm())`` legs qualify automatically: their distance is
        recovered at lowering time.  Non-qualifying nodes pass through untouched.
        """
        return self._add(ToAbsolutePass())

    def cut_corners(self, radius_cm: float) -> "Optimizer":
        """Replace ``linear+turn+linear`` corners with arcs (``CornerCutPass``).

        ``radius_cm`` is the cut distance trimmed from each straight leg, in
        centimeters; converted to meters for ``CornerCutPass`` here.
        """
        return self._add(CornerCutPass(radius_cm / 100.0))

    def apply(self, p) -> "Optimizer":
        """Append a user-supplied compiler pass."""
        return self._add(p)

    # -- Step interface ----------------------------------------------------

    def required_resources(self) -> frozenset[str]:
        return frozenset({"drive"})

    def collected_resources(self) -> frozenset[str]:
        result: set[str] = {"drive"}
        nodes, _ = flatten_steps(self._raw_steps)
        for node in nodes:
            if isinstance(node, SideAction):
                result |= node.step.collected_resources()
        return frozenset(result)

    def _generate_signature(self) -> str:
        return f"Optimize(passes={[p.name for p in self._passes]})"

    async def _execute_step(self, robot: "GenericRobot") -> None:
        """Compile the raw steps and run the plan via the path executor."""
        plan = PathCompiler(self._passes).compile(self._raw_steps)
        executor = PathExecutor(
            nodes=plan.nodes,
            deferred=plan.deferred,
            hz=self.hz,
        )
        await executor.run(robot)

    # -- diagnostics -------------------------------------------------------

    def explain(self) -> str:
        """Render the node list before and after each pass — no execution.

        Lowers the raw steps to IR, then applies each configured pass in turn,
        capturing the node list after each.  Returns a readable multi-line
        report for debugging pass composition.  Does not touch a robot.
        """
        nodes, _deferred = flatten_steps(self._raw_steps)

        lines: list[str] = ["Optimize.explain():", "raw (lowering):", _render_nodes(nodes)]

        for p in self._passes:
            nodes = p.run(nodes)
            lines.append(f"after {p.name}:")
            lines.append(_render_nodes(nodes))

        seg_count = sum(1 for n in nodes if isinstance(n, Segment))
        action_count = sum(1 for n in nodes if isinstance(n, SideAction))
        deferred_count = sum(1 for n in nodes if n is None)
        lines.append(
            f"summary: {len(nodes)} nodes "
            f"({seg_count} segment(s), {action_count} side action(s), "
            f"{deferred_count} deferred); passes=[{', '.join(p.name for p in self._passes)}]"
        )
        return "\n".join(lines)


# ---------------------------------------------------------------------------
# DSL factory function
# ---------------------------------------------------------------------------


def optimize(steps) -> Optimizer:
    """Build a fluent path optimizer over a list of motion steps.

    ``optimize()`` is the explicit-pass counterpart to ``smooth_path()``: it
    wraps the same compiler + executor pipeline, but instead of boolean flags
    you append optimization passes by chaining methods.  Each chain method
    returns the builder, Java-stream style::

        optimize([drive_forward(50), turn_right(90), drive_forward(30)])
            .merge()
            .cut_corners(5)

    With no passes appended it is a drop-in for ``seq(steps)`` — the executor
    runs the lowered segments back-to-back with its normal warm-start loop.

    A compile-time state machine validates pass order: a pass may declare the
    segment ``Representation`` it requires/produces and whether it is terminal.
    Composing passes that disagree (e.g. an absolute-only pass on a relative
    stream, or any pass after a terminal one) raises ``PathBuildError`` at
    build time.

    Available passes:

    - ``.merge()`` — collapse adjacent same-type, same-direction segments
      (``drive(30)+drive(20)`` → ``drive(50)``).
    - ``.cut_corners(radius_cm)`` — replace ``linear+turn+linear`` corners
      with a circular arc, trimming ``radius_cm`` from each straight leg.
    - ``.apply(pass)`` — append any custom ``CompilerPass``.

    Use ``.explain()`` to dump the IR node list after each pass without
    executing anything.

    Prerequisites:
        ``calibrate_distance()`` if any segment uses distance-based mode.
        ``mark_heading_reference()`` if any segment uses heading hold.

    Args:
        steps: A list of motion steps (or a single step / nested ``seq()``).
            Accepts the same step family as ``smooth_path``.

    Returns:
        An ``Optimizer`` step. Chain passes onto it, then run it like any step.

    Example::

        from raccoon.step.motion import optimize, drive_forward, turn_right

        optimize(
            [
                drive_forward(30),
                drive_forward(20),
                turn_right(90),
                drive_forward(40),
            ]
        ).merge().cut_corners(5)
    """
    raw = list(steps) if isinstance(steps, list | tuple) else [steps]
    return Optimizer(raw)
