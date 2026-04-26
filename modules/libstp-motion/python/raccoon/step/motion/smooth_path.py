"""Smooth multi-segment motion path with zero-stop transitions.

Supports bare motion steps, nested ``seq()``, ``parallel()`` with a motion
spine, ``background()`` steps, and ``Run`` actions.  Composite steps are
flattened into a linear path of motion segments and side actions at
construction time, with deferred steps resolved at runtime.

This module is now a thin DSL wrapper.  The actual pipeline lives in
``raccoon.step.motion.path`` (compiler passes, executor, middleware).
"""

from __future__ import annotations

from typing import TYPE_CHECKING

from .. import Step, dsl
from .path import (
    SENTINEL_DISTANCE_M as _SENTINEL_DISTANCE_M,
)
from .path import (
    CompiledPlan,
    PathCompiler,
    PathExecutor,
    WorldCorrectionMiddleware,
)
from .path import (
    Segment as _Segment,
)
from .path import (
    SideAction as _SideAction,
)
from .path.passes import (
    CornerCutPass,
    MergePass,
)
from .path.passes import (
    build_spline_step as _build_spline_step,
)

if TYPE_CHECKING:
    from raccoon.robot.api import GenericRobot


# Sentinel kept on _Segment for backwards compatibility with code that
# referenced ``_Segment._SENTINEL_DISTANCE_M``.
_Segment._SENTINEL_DISTANCE_M = _SENTINEL_DISTANCE_M


def _build_default_passes(*, optimize: bool, corner_cut_m: float) -> list:
    """Build the default pass list from legacy ``smooth_path()`` flags."""
    passes: list = []
    if optimize or corner_cut_m > 0.0:
        passes.append(MergePass())
    if corner_cut_m > 0.0:
        passes.append(CornerCutPass(corner_cut_m))
    return passes


# ---------------------------------------------------------------------------
# SmoothPath Step
# ---------------------------------------------------------------------------


@dsl(hidden=True)
class SmoothPath(Step):
    """Execute a sequence of motion segments with smooth velocity transitions.

    Instead of decelerating to zero between each step, carries velocity
    across segment boundaries using warm-started motion controllers.

    Supports composite steps: nested ``seq()`` is flattened, ``parallel()``
    with a motion spine runs side-effect branches concurrently,
    ``background()`` steps are launched without blocking, and non-drive
    steps (servos, Run actions, etc.) execute at transition points.
    """

    hz: int = 100

    def __init__(
        self,
        steps: list,
        correct: bool = True,
        optimize: bool = False,
        corner_cut_m: float = 0.0,
        spline: bool = False,
    ) -> None:
        super().__init__()
        if not steps:
            msg = "smooth_path() requires at least one step"
            raise ValueError(msg)
        if corner_cut_m > 0.0 and spline:
            msg = (
                "smooth_path(): corner_cut_cm and spline=True are mutually "
                "exclusive — use one strategy for handling turns"
            )
            raise ValueError(msg)

        self._raw_steps = steps
        self._correct = correct
        self._optimize = optimize
        self._corner_cut_m = corner_cut_m
        self._spline_flag = spline

        # Compile the plan via the standard pipeline.
        compiler = PathCompiler(
            passes=_build_default_passes(
                optimize=optimize,
                corner_cut_m=corner_cut_m,
            ),
        )
        self._plan: CompiledPlan = compiler.compile(steps)

        # Backwards-compat: expose the IR fields via the same attribute names
        # the previous implementation used.
        self._nodes = self._plan.nodes
        self._deferred = self._plan.deferred

        # Spline mode replaces the entire path with a SplinePath step.
        # Built up-front so errors surface at construction time.
        self._spline_step = _build_spline_step(self._nodes) if spline else None

        # Construction-time validation: at least one motion segment, or a
        # deferred placeholder that might resolve to one.
        has_segment = any(isinstance(n, _Segment) for n in self._nodes)
        has_deferred = len(self._deferred) > 0
        if not has_segment and not has_deferred:
            msg = (
                "smooth_path() requires at least one motion step "
                "(drive, turn, or arc), but none were found"
            )
            raise ValueError(msg)

    _composite = True

    def required_resources(self) -> frozenset[str]:
        return frozenset({"drive"})

    def collected_resources(self) -> frozenset[str]:
        result: set[str] = {"drive"}
        for node in self._nodes:
            if isinstance(node, _SideAction):
                result |= node.step.collected_resources()
        return frozenset(result)

    def _generate_signature(self) -> str:
        import math

        parts: list[str] = []
        for node in self._nodes:
            if node is None:
                parts.append("Defer(?)")
            elif isinstance(node, _Segment):
                seg = node
                if seg.kind == "linear":
                    d = f"{abs(seg.distance_m or 0) * 100:.0f}cm" if seg.distance_m else "until"
                    parts.append(f"drive({d})")
                elif seg.kind == "turn":
                    a = (
                        f"{abs(math.degrees(seg.angle_rad or 0)):.0f}°"
                        if seg.angle_rad
                        else "until"
                    )
                    parts.append(f"turn({a})")
                elif seg.kind == "arc":
                    parts.append(f"arc({abs(math.degrees(seg.arc_angle_rad or 0)):.0f}°)")
                elif seg.kind == "follow_line":
                    d = f"{abs(seg.distance_m or 0) * 100:.0f}cm" if seg.distance_m else "until"
                    parts.append(f"follow_line({d})")
                elif seg.kind == "spline":
                    parts.append("spline(...)")
            elif isinstance(node, _SideAction):
                label = type(node.step).__name__
                mode = "bg" if node.is_background else "inline"
                parts.append(f"{label}({mode})")
        flags: list[str] = []
        if self._spline_flag:
            flags.append("spline")
        elif self._optimize:
            flags.append("opt")
        if self._corner_cut_m > 0.0:
            flags.append(f"cut={self._corner_cut_m * 100:.0f}cm")
        suffix = f" [{', '.join(flags)}]" if flags else ""
        return f"SmoothPath([{', '.join(parts)}]{suffix})"

    async def _execute_step(self, robot: "GenericRobot") -> None:
        """Run the compiled plan via the path executor."""
        middlewares: list = []
        if self._correct:
            middlewares.append(WorldCorrectionMiddleware())

        executor = PathExecutor(
            nodes=self._plan.nodes,
            deferred=self._plan.deferred,
            spline_step=self._spline_step,
            middlewares=middlewares,
            hz=self.hz,
        )
        await executor.run(robot)


# ---------------------------------------------------------------------------
# DSL factory function
# ---------------------------------------------------------------------------


@dsl(tags=["motion", "path"])
def smooth_path(
    *steps,
    correct: bool = True,
    optimize: bool = False,
    corner_cut_cm: float | None = None,
    spline: bool = False,
) -> SmoothPath:
    """Execute motion steps with smooth velocity transitions between them.

    Eliminates the velocity drops that occur when steps are chained via
    ``seq()``. Instead of decelerating to zero and hard-stopping between
    each step, ``smooth_path`` carries velocity across segment boundaries,
    producing continuous fluid motion.

    For same-type transitions (e.g., drive to drive), the velocity is
    carried seamlessly via warm-started motion controllers with zero
    discontinuity. For cross-type transitions (e.g., drive to turn),
    a soft stop is used which smoothly decelerates without resetting
    PID state.

    When ``correct`` is enabled (default), the path tracks the robot's
    world-frame pose and corrects accumulated position and heading
    errors at each segment transition. Along-track errors adjust the
    next linear segment's distance, cross-track errors bias the heading
    hold, and heading errors adjust turn/arc angles. All corrections
    are clamped to safe limits (3 cm distance, 5 deg heading).

    Supports composite steps:

    - **Nested seq()**: Automatically flattened into the motion spine.
    - **parallel()**: The branch containing drive steps becomes the
      motion spine; other branches run concurrently as side effects.
    - **background()**: Launched without blocking the motion flow.
    - **Non-drive steps** (servos, ``Run`` actions, etc.): Execute at
      transition points between motion segments.

    Steps with ``.until()`` conditions are fully supported. When a
    condition fires, the current velocity is carried into the next
    segment immediately.

    Supports ``Defer`` steps (e.g., ``turn_to_heading_right``), which
    are resolved **lazily** at the actual transition point so that
    heading-dependent steps see the robot's current heading, not the
    heading at the start of the path.

    **Optimization** (all opt-in):

    ``optimize=True`` applies algebraic simplification passes at
    construction time, before execution:

    - *Merge*: adjacent same-type, same-direction, same-axis segments
      with no conditions are collapsed into one.  ``drive(30) + drive(20)``
      becomes ``drive(50)``; ``turn(30°) + turn(20°)`` becomes ``turn(50°)``.
      Only same-direction merges are performed (``drive(30) + drive(-20)``
      is not merged — intent may differ from net displacement).
      ``_SideAction`` and deferred nodes act as merge barriers.

    ``corner_cut_cm=N`` (implies algebraic merge as well) replaces
    ``linear + turn + linear`` triples with ``linear + arc + linear`` by
    cutting *N* cm from each straight leg and inserting a circular arc
    of radius ``R = N / tan(|θ| / 2)`` at the corner.  The robot never
    slows for the turn — it follows the arc at cruise speed.  Only
    segments with known endpoints and no conditions are eligible.

    Note on ``_SideAction`` barriers: a ``background()`` or non-drive step
    between two segments pins that step to the transition point.  Optimization
    never reorders across side actions, preserving the execution contract.

    Prerequisites:
        ``calibrate_distance()`` if any segment uses distance-based mode.
        ``mark_heading_reference()`` if any segment uses heading hold.

    Args:
        *steps: Motion steps to execute smoothly. Accepts
            ``drive_forward``, ``drive_backward``, ``strafe_left``,
            ``strafe_right``, ``turn_left``, ``turn_right``,
            ``turn_to_heading_left``, ``turn_to_heading_right``,
            ``drive_arc_left``, ``drive_arc_right``,
            ``follow_line``, ``follow_line_single``, ``spline``,
            nested ``seq()``, ``parallel()`` with a motion spine,
            ``background()``, and their builder variants with
            ``.until()`` and ``.speed()``.
        correct: Enable world-frame error correction across segment
            transitions. Default ``True``. Set to ``False`` for the
            legacy uncorrected behavior.
        optimize: Merge adjacent same-type segments algebraically.
            Default ``False`` (opt-in).
        corner_cut_cm: If set, replace right-angle drive+turn+drive
            corners with arcs of radius ``corner_cut_cm / tan(|θ|/2)``,
            cutting *corner_cut_cm* cm from each straight leg.  Also
            enables the merge pass.  Mutually exclusive with ``spline``.
            Default ``None`` (disabled).
        spline: If True, convert the drive/turn sequence into waypoints
            and follow them with a Catmull-Rom spline (``SplinePath``).
            Mutually exclusive with ``corner_cut_cm``.  Default ``False``.

    Returns:
        A SmoothPath step that executes the segments as one fluid motion.

    Example::

        from raccoon.step.motion import smooth_path, drive_forward, turn_to_heading_right

        # Three drives with no stops between them
        smooth_path(
            drive_forward(50),
            drive_forward(30),
            drive_forward(20),
        )

        # Drive, turn, drive — soft transition at type boundaries
        smooth_path(
            drive_forward(50),
            turn_to_heading_right(90),
            drive_forward(30),
        )

        # Merge adjacent drives and cut corners with 5 cm arcs
        smooth_path(
            drive_forward(30),
            drive_forward(20),
            turn_left(90),
            drive_forward(40),
            optimize=True,
            corner_cut_cm=5.0,
        )
        # → drive(50) + arc(R=5cm, 90°) + drive(35)
    """
    cut_m = (corner_cut_cm / 100.0) if corner_cut_cm is not None else 0.0
    do_merge = optimize or (corner_cut_cm is not None)
    return SmoothPath(
        list(steps),
        correct=correct,
        optimize=do_merge,
        corner_cut_m=cut_m,
        spline=spline,
    )
