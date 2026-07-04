"""Fluent path-optimizer builder — ``optimize([...])``.

A thin, fluent wrapper around the existing compiler + executor pipeline.
Where ``smooth_path()`` exposes optimization as boolean flags, ``optimize()``
exposes it as a chain of explicit passes — Java-stream style::

    optimize([drive_forward(50), turn_right(90), drive_forward(30)])
        .cut_corners(5)

The builder is itself a ``Step``: it compiles its raw steps through
``PathCompiler`` and runs the result through ``PathExecutor``, mirroring the
non-absolute, non-spline execution wiring of ``SmoothPath``.

``DecomposePass`` then ``MergePass`` are ALWAYS-ON — prepended to every
compiled pipeline before the user's chained passes — because both are
behavior-preserving.  Use ``seq()`` instead of ``optimize()`` if you want
neither.

A small compile-time state machine validates composition of the CHAINED
passes.  Each pass may optionally declare the segment representation it
``requires`` / ``produces`` and whether it is ``terminal`` (see
``passes/contract.py`` for :class:`Representation` and the defaults); passes
that don't declare these (``MergePass`` / ``CornerCutPass``) default to
``EITHER`` / ``SAME`` / non-terminal and keep working unchanged.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

from ... import Step
from .compiler import PathCompiler
from .executor import PathExecutor
from .ir import Segment, SideAction
from .passes import (
    AbsoluteHeadingPass,
    CornerCutPass,
    DecomposePass,
    MergePass,
    Representation,
    SplinifyPass,
    ToAbsolutePass,
    VelocityProfilePass,
    flatten_steps,
)
from .passes.contract import (
    DEFAULT_PRODUCES,
    DEFAULT_REQUIRES,
    DEFAULT_TERMINAL,
)

if TYPE_CHECKING:
    from raccoon.robot.api import GenericRobot


# ---------------------------------------------------------------------------
# Pass contract extension
# ---------------------------------------------------------------------------
#
# ``Representation`` and the ``DEFAULT_*`` constants live in
# ``path/passes/contract.py`` (a neutral module) so passes can declare
# ``requires`` / ``produces`` / ``terminal`` against them without a circular
# import back into this module.  They are re-imported above.


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
        elif seg.kind == "crab_arc":
            parts.append(f"radius_m={seg.radius_m}")
            parts.append(f"from={seg.crab_from}")
            parts.append(f"to={seg.crab_to}")
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

    Built via the ``optimize()`` factory.  Chain ``.cut_corners()``,
    ``.to_absolute()``, ``.splinify()``, ``.absolute_heading()`` or
    ``.apply(custom_pass)`` to append compiler passes; each returns ``self`` so
    calls compose Java-stream style.  Execution compiles the raw steps through
    ``PathCompiler`` and runs the result on ``PathExecutor`` (relative,
    non-spline path).

    Two passes are ALWAYS-ON and run before any user pass: ``DecomposePass``
    (splits ``after_cm + sensor`` legs in time) then ``MergePass`` (collapses
    adjacent same-type legs).  Both are behavior-preserving RELATIVE→RELATIVE
    transforms, so they are prepended to the compiled pipeline without going
    through ``_add`` (they don't affect the representation state the user's
    passes are validated against).  If you want neither, use ``seq()`` rather
    than ``optimize()``.
    """

    hz: int = 100

    def __init__(self, steps: list) -> None:
        super().__init__()
        self._raw_steps = steps
        self._passes: list = []
        self._repr = Representation.RELATIVE
        self._terminated_by: str | None = None
        # Absolute MODE — set by to_absolute(). Drives whether the compiled
        # pipeline runs the per-leg ToAbsolutePass and whether splinify renders
        # absolute (SplineFollow) vs relative (Segment(kind="spline")).
        self._absolute = False

    _composite = True

    # -- compile-time state machine ----------------------------------------

    def _add(self, p) -> "Optimizer":
        """Append a pass, validating representation flow and termination."""
        if self._terminated_by is not None:
            msg = f"{p.name}() after terminal {self._terminated_by}()"
            raise PathBuildError(msg)

        req = _pass_requires(p)
        if req not in (self._repr, Representation.EITHER):
            msg = f"{p.name}() needs {req.name} segments, stream is {self._repr.name} — reorder"
            raise PathBuildError(msg)

        self._passes.append(p)

        produces = _pass_produces(p)
        if produces != Representation.SAME:
            self._repr = produces

        if _pass_terminal(p):
            self._terminated_by = p.name

        return self

    def _effective_passes(self) -> list:
        """The full compiled pass pipeline: always-on prepends + user passes.

        ``DecomposePass`` then ``MergePass`` always run first (both are
        behavior-preserving RELATIVE→RELATIVE transforms), followed by the
        user's opt-in passes in the order they were chained.  decompose runs
        before merge — and both before user passes — which also yields the
        canonical "merge before cut_corners" ordering for free.

        Absolute-mode coupling: when both ``to_absolute()`` and ``splinify()``
        are chained, the whole path becomes ONE absolute spline — so the per-leg
        ``ToAbsolutePass`` is dropped and ``splinify`` renders absolute.  With
        only ``to_absolute()`` the per-leg pass runs; with only ``splinify()`` it
        renders the relative spline.
        """
        has_splinify = any(isinstance(p, SplinifyPass) for p in self._passes)
        out: list = [DecomposePass(), MergePass()]
        for p in self._passes:
            if isinstance(p, ToAbsolutePass) and has_splinify:
                continue  # subsumed: the whole path is one absolute spline
            if isinstance(p, SplinifyPass):
                out.append(SplinifyPass(absolute=self._absolute))
            else:
                out.append(p)
        return out

    # -- public chain methods ----------------------------------------------

    def to_absolute(self) -> "Optimizer":
        """Drive as ABSOLUTE as possible — closed-loop on the localization filter.

        Turns on absolute mode.  On its own, each maximal run of known-endpoint
        ``linear`` / ``turn`` / ``diagonal`` legs becomes ONE ``GotoWaypoints``
        and each sensor-bounded single-axis leg becomes an ``AbsoluteHoldMove``
        (cross-axis + heading held absolute, free axis until the sensor) — every
        leg regulating on the particle filter so accumulated odometry drift is
        corrected during the move.  Chained with ``.splinify()`` the whole path
        instead becomes one absolute spline (a continuous ``SplineFollow``).

        Reads localization only — never feeds it.
        """
        self._absolute = True
        return self._add(ToAbsolutePass())

    def absolute_heading(self) -> "Optimizer":
        """Pin every straight leg to one integrated heading (``AbsoluteHeadingPass``).

        Integrates a single running heading through the path at compile time
        (start 0; turns/arcs add their signed angle) and stamps it onto each
        ``linear`` / ``follow_line`` leg, so every straight leg regulates
        against ONE coherent heading reference rather than re-zeroing on the
        previous leg's drifted frame — drift-robust over long paths. The pass
        emits headings in the path-start frame; the executor adds the
        path-start world-heading offset H0 (captured once at the first motion
        segment) at runtime so they become absolute world targets.
        """
        return self._add(AbsoluteHeadingPass())

    def cut_corners(self, radius_cm: float, cut_until: bool = False) -> "Optimizer":
        """Round corners between straight legs with arcs (``CornerCutPass``).

        Cuts two corner shapes, trimming ``radius_cm`` from each straight leg:

        - **Turn corners** (``linear+turn+linear``, same-axis legs) become a
          circular arc — forward legs give a drive arc, strafe legs a lateral arc.
        - **Crab corners** (``linear(Forward)+linear(Lateral)``, no turn) become
          a constant-heading ``crab_arc``: a holonomic base blends forward into
          strafe over a 90° quarter circle without rotating.

        By default only corners between known fixed-distance legs are cut. Set
        ``cut_until=True`` to also round a corner whose EXIT leg is a
        sensor-bounded ``.until()`` leg: the entry leg is trimmed, the arc rounds
        the corner, and the ``.until()`` leg then runs from the arc's end until
        its condition fires (it is kept untrimmed — its endpoint is unknown, and
        the sensor still triggers at the same place). The ENTRY leg can never be
        a ``.until()`` leg: the fillet would have to start curving before the
        sensor fires, which can't be anticipated.

        ``radius_cm`` is the cut distance trimmed from each (known) straight leg,
        in centimeters; converted to meters for ``CornerCutPass`` here.
        """
        return self._add(CornerCutPass(radius_cm / 100.0, cut_until=cut_until))

    def splinify(self) -> "Optimizer":
        """Replace the whole relative run with one Catmull-Rom spline (terminal).

        Collapses the linear/turn path into a single ``spline`` segment driven
        through its waypoints (``SplinifyPass``).  Terminal — nothing may follow.
        Raises ``ValueError`` at compile time if the path can't be splinified
        (side actions, arcs, conditions, deferred steps, or < 2 waypoints).
        """
        return self._add(SplinifyPass())

    def time_optimal(
        self,
        max_speed_cmps: float = 100.0,
        accel_cmps2: float = 60.0,
        lateral_accel_cmps2: float = 50.0,
        sensor_carry_cmps: float = float("inf"),
    ) -> "Optimizer":
        """Carry speed through the WHOLE path — the global generalisation of the
        per-seam warm-start (``VelocityProfilePass``).

        Runs a forward/backward sweep that stamps the fastest feasible entry/exit
        speed on every leg, so the robot only decelerates for REAL stops (turns,
        direction reversals, blocking side actions, the final leg) and
        corner/curvature limits — instead of braking to a halt at every segment
        seam.

        ``sensor_carry_cmps`` is the "on steroids" knob for the sensor-driven
        style: a leg that ends on a ``.until()`` SENSOR/time condition normally
        forces a full stop, but when the NEXT leg continues in the same direction
        the robot instead FLOWS THROUGH the sensor boundary rather than braking to
        zero — far cleaner, faster motion on a robot that stops at every sensor.
        It defaults to ``inf`` (the robot keeps its full cruise through the seam —
        it is already at cruise when the sensor fires, so braking would only waste
        time, and the same-direction flow adds no lateral overshoot). Lower it
        (cm/s) to throttle the flow-through where a sensor needs dense sampling;
        set 0 to brake at every sensor as before. Accuracy-preserving by
        construction.

        PATH-PRESERVING: changes only speed, never geometry, so the endpoint
        matches the un-profiled path. Most effective after ``cut_corners()`` (it
        carries speed through the arcs the corner-cut introduced). Limits are in
        cm: ``max_speed_cmps`` cruise cap, ``accel_cmps2`` accel/decel,
        ``lateral_accel_cmps2`` the arc curvature cap (``v <= sqrt(a_lat * R)``).
        """
        return self._add(
            VelocityProfilePass(
                max_speed_mps=max_speed_cmps / 100.0,
                accel_mps2=accel_cmps2 / 100.0,
                lateral_accel_mps2=lateral_accel_cmps2 / 100.0,
                sensor_carry_mps=sensor_carry_cmps / 100.0,
            )
        )

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
        # Show the effective pipeline (always-on decompose+merge prepended)
        # so the signature reflects what actually runs at compile time.
        return f"Optimize(passes={[p.name for p in self._effective_passes()]})"

    async def _execute_step(self, robot: "GenericRobot") -> None:
        """Compile the raw steps and run the plan via the path executor."""
        # Composite steps log only at debug level, so without this the whole
        # optimize() block would run silently except for its inline side actions.
        # Announce the compiled pipeline; the executor then logs each leg.
        self.info(self._generate_signature())
        plan = PathCompiler(self._effective_passes()).compile(self._raw_steps)
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

        effective = self._effective_passes()
        for p in effective:
            nodes = p.run(nodes)
            lines.append(f"after {p.name}:")
            lines.append(_render_nodes(nodes))

        seg_count = sum(1 for n in nodes if isinstance(n, Segment))
        action_count = sum(1 for n in nodes if isinstance(n, SideAction))
        deferred_count = sum(1 for n in nodes if n is None)
        lines.append(
            f"summary: {len(nodes)} nodes "
            f"({seg_count} segment(s), {action_count} side action(s), "
            f"{deferred_count} deferred); passes=[{', '.join(p.name for p in effective)}]"
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
            .cut_corners(5)

    ``decompose`` then ``merge`` ALWAYS run first (before any chained pass):
    both are behavior-preserving, so ``optimize()`` always splits
    ``after_cm + sensor`` legs and collapses adjacent same-type legs.  If you
    want neither, use ``seq(steps)`` instead.

    A compile-time state machine validates the order of the CHAINED passes: a
    pass may declare the segment ``Representation`` it requires/produces and
    whether it is terminal.  Composing passes that disagree (e.g. an
    absolute-only pass on a relative stream, or any pass after a terminal one)
    raises ``PathBuildError`` at build time.

    Available chained passes:

    - ``.cut_corners(radius_cm)`` — replace ``linear+turn+linear`` corners
      with a circular arc, trimming ``radius_cm`` from each straight leg.
    - ``.to_absolute()`` — convert known-endpoint relative runs into one
      closed-loop navigate-to-pose move.
    - ``.absolute_heading()`` — pin every straight leg to one integrated heading.
    - ``.splinify()`` — collapse the relative run into one spline (terminal).
    - ``.apply(pass)`` — append any custom ``CompilerPass``.

    Use ``.explain()`` to dump the IR node list after each pass (including the
    always-on decompose + merge) without executing anything.

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
        ).cut_corners(5)
    """
    raw = list(steps) if isinstance(steps, list | tuple) else [steps]
    return Optimizer(raw)
