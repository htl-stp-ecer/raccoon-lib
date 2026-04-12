"""Smooth multi-segment motion path with zero-stop transitions.

Supports bare motion steps, nested ``seq()``, ``parallel()`` with a motion
spine, ``background()`` steps, and ``Run`` actions.  Composite steps are
flattened into a linear path of motion segments and side actions at
construction time, with deferred steps resolved at runtime.
"""

import asyncio
import math
from dataclasses import dataclass
from typing import TYPE_CHECKING, Optional, Union

from raccoon.motion import (
    LinearMotion, LinearMotionConfig, LinearAxis,
    TurnMotion, TurnConfig,
    ArcMotion, ArcMotionConfig,
)

from .. import Step, dsl
from ..condition import StopCondition
from ..logic.defer import Defer
from .motion_step import MotionStep

if TYPE_CHECKING:
    from raccoon.robot.api import GenericRobot


# ---------------------------------------------------------------------------
# Segment descriptor
# ---------------------------------------------------------------------------

@dataclass
class _Segment:
    """Internal representation of one motion segment extracted from a Step."""

    kind: str  # "linear" | "turn" | "arc"
    # Linear params
    axis: Optional[LinearAxis] = None
    sign: float = 1.0
    distance_m: Optional[float] = None  # None = condition-only (sentinel)
    speed_scale: float = 1.0
    heading_deg: Optional[float] = None
    # Turn params
    angle_rad: Optional[float] = None
    # Arc params
    radius_m: Optional[float] = None
    arc_angle_rad: Optional[float] = None
    lateral: bool = False
    # Common
    condition: Optional[StopCondition] = None
    has_known_endpoint: bool = True

    _SENTINEL_DISTANCE_M = 100.0


@dataclass
class _SideAction:
    """A non-motion step to execute at a transition point in the path."""

    step: Step
    is_background: bool  # True = fire-and-forget async task; False = await inline


# Union type for flattened path nodes
_PathNode = Union[_Segment, _SideAction]


# ---------------------------------------------------------------------------
# Segment extraction
# ---------------------------------------------------------------------------

def _extract_segment(step: Step) -> _Segment:
    """Extract motion parameters from a resolved step into a _Segment."""
    # Import here to avoid circular imports at module level
    from .drive import _ConditionalDrive
    from .turn import _ConditionalTurn
    from .arc import Arc

    if isinstance(step, _ConditionalDrive):
        cm = step._cm
        return _Segment(
            kind="linear",
            axis=step._axis,
            sign=step._sign,
            distance_m=step._sign * cm / 100.0 if cm is not None else None,
            speed_scale=step._speed,
            heading_deg=step._heading_deg,
            condition=step._until,
            has_known_endpoint=cm is not None,
        )

    if isinstance(step, _ConditionalTurn):
        degrees = step._degrees
        return _Segment(
            kind="turn",
            sign=step._sign,
            angle_rad=step._sign * math.radians(degrees) if degrees is not None else None,
            speed_scale=step._speed,
            condition=step._until,
            has_known_endpoint=degrees is not None,
        )

    if isinstance(step, Arc):
        cfg = step.config
        return _Segment(
            kind="arc",
            radius_m=cfg.radius_m,
            arc_angle_rad=cfg.arc_angle_rad,
            speed_scale=cfg.speed_scale,
            lateral=cfg.lateral,
            has_known_endpoint=True,
        )

    raise TypeError(
        f"smooth_path() does not support {type(step).__name__} as a motion "
        f"step. Only drive, turn, and arc steps can form the motion spine."
    )


def _resolve_step(step) -> Step:
    """Resolve a builder to a concrete Step instance."""
    if hasattr(step, "resolve"):
        return step.resolve()
    return step


def _is_same_type(a: _Segment, b: _Segment) -> bool:
    """Check if two segments can use warm-start (same motion type)."""
    if a.kind != b.kind:
        return False
    if a.kind == "linear":
        return a.axis == b.axis
    return True


# ---------------------------------------------------------------------------
# Recursive step flattening
# ---------------------------------------------------------------------------

def _flatten_one(
    step,
    nodes: list[Optional[_PathNode]],
    deferred: list[tuple[int, Defer]],
) -> None:
    """Recursively flatten *step* into path nodes.

    Appends to *nodes* in-place.  Deferred steps become ``None``
    placeholders with entries in *deferred* for runtime resolution.
    """
    from ..sequential import Sequential
    from ..parallel import Parallel
    from ..logic.background import Background
    from ..logic.defer import Run

    # 1. Defer — placeholder for runtime resolution
    if isinstance(step, Defer):
        deferred.append((len(nodes), step))
        nodes.append(None)
        return

    # 2. Resolve builder (e.g., drive_forward(30) returns a builder)
    step = _resolve_step(step)

    # 3. Sequential — flatten children recursively
    if isinstance(step, Sequential):
        for child in step.steps:
            _flatten_one(child, nodes, deferred)
        return

    # 4. Parallel — find motion spine, side-effect branches
    if isinstance(step, Parallel):
        _flatten_parallel(step, nodes, deferred)
        return

    # 5. Background — non-blocking side action
    if isinstance(step, Background):
        nodes.append(_SideAction(step=step._step, is_background=True))
        return

    # 6. Run — inline side action (quick callable)
    if isinstance(step, Run):
        nodes.append(_SideAction(step=step, is_background=False))
        return

    # 7. Try to extract as a motion segment
    try:
        seg = _extract_segment(step)
        nodes.append(seg)
        return
    except TypeError:
        pass

    # 8. Non-motion step — check if it uses the drive resource
    resources = step.collected_resources()
    if "drive" in resources:
        raise TypeError(
            f"smooth_path() does not support {type(step).__name__} — "
            f"it uses the drive resource but is not a supported motion step. "
            f"Only drive, turn, and arc steps can form the motion spine."
        )

    # 9. Non-drive step — treat as inline side action
    nodes.append(_SideAction(step=step, is_background=False))


def _flatten_parallel(
    par,
    nodes: list[Optional[_PathNode]],
    deferred: list[tuple[int, Defer]],
) -> None:
    """Flatten a Parallel step, identifying the motion spine branch."""
    from ..sequential import Sequential

    spine_idx = None
    for i, branch in enumerate(par.steps):
        branch_resources = branch.collected_resources()
        if "drive" in branch_resources:
            if spine_idx is not None:
                raise TypeError(
                    "smooth_path(): parallel() has multiple branches using "
                    "the drive resource — only one motion spine is allowed"
                )
            spine_idx = i

    if spine_idx is None:
        # No motion branch — entire parallel is a side-effect
        nodes.append(_SideAction(step=par, is_background=False))
        return

    # Side-effect branches: launch as background tasks at this point
    for i, branch in enumerate(par.steps):
        if i != spine_idx:
            nodes.append(_SideAction(step=branch, is_background=True))

    # Flatten the motion spine branch
    spine_branch = par.steps[spine_idx]
    if isinstance(spine_branch, Sequential):
        for child in spine_branch.steps:
            _flatten_one(child, nodes, deferred)
    else:
        _flatten_one(spine_branch, nodes, deferred)


def _flatten_steps(
    steps: list,
) -> tuple[list[Optional[_PathNode]], list[tuple[int, Defer]]]:
    """Flatten a list of steps into a linear path.

    Returns:
        nodes: Flat list of ``_Segment``, ``_SideAction``, or ``None``
            (deferred placeholder).
        deferred: List of ``(index, Defer)`` pairs for runtime resolution.
    """
    nodes: list[Optional[_PathNode]] = []
    deferred: list[tuple[int, Defer]] = []
    for step in steps:
        _flatten_one(step, nodes, deferred)
    return nodes, deferred


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

    def __init__(self, steps: list) -> None:
        super().__init__()
        if not steps:
            raise ValueError("smooth_path() requires at least one step")
        self._raw_steps = steps
        self._nodes, self._deferred = _flatten_steps(steps)

        # Construction-time validation: at least one motion segment
        # (or a deferred that might resolve to one)
        has_segment = any(isinstance(n, _Segment) for n in self._nodes)
        has_deferred = len(self._deferred) > 0
        if not has_segment and not has_deferred:
            raise ValueError(
                "smooth_path() requires at least one motion step "
                "(drive, turn, or arc), but none were found"
            )

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
        parts = []
        for node in self._nodes:
            if node is None:
                parts.append("Defer(?)")
            elif isinstance(node, _Segment):
                seg = node
                if seg.kind == "linear":
                    d = f"{abs(seg.distance_m or 0) * 100:.0f}cm" if seg.distance_m else "until"
                    parts.append(f"drive({d})")
                elif seg.kind == "turn":
                    a = f"{abs(math.degrees(seg.angle_rad or 0)):.0f}\u00b0" if seg.angle_rad else "until"
                    parts.append(f"turn({a})")
                elif seg.kind == "arc":
                    parts.append(f"arc({abs(math.degrees(seg.arc_angle_rad or 0)):.0f}\u00b0)")
            elif isinstance(node, _SideAction):
                label = type(node.step).__name__
                mode = "bg" if node.is_background else "inline"
                parts.append(f"{label}({mode})")
        return f"SmoothPath([{', '.join(parts)}])"

    # ------------------------------------------------------------------
    # Profile overshoot for non-terminal segments
    # ------------------------------------------------------------------
    # For intermediate segments, inflate the profile target so the
    # trapezoidal velocity profile stays in cruise phase at the actual
    # endpoint. The Python loop checks distance/angle manually and
    # triggers the transition while the robot is still at cruise speed.
    _OVERSHOOT_M = 1.0     # 1m overshoot (decel zone ~0.2m at typical params)
    _OVERSHOOT_RAD = 3.0   # ~172° overshoot (decel zone ~1.5rad at typical params)
    _DISTANCE_TOL_M = 0.005   # 5mm tolerance for manual distance check
    _ANGLE_TOL_RAD = 0.035    # ~2° tolerance for manual angle check

    # ------------------------------------------------------------------
    # Motion factory
    # ------------------------------------------------------------------

    def _create_linear_motion(
        self, robot: "GenericRobot", seg: _Segment, is_last: bool,
    ) -> LinearMotion:
        config = LinearMotionConfig()
        config.axis = seg.axis
        actual = (
            seg.distance_m
            if seg.distance_m is not None
            else seg.sign * _Segment._SENTINEL_DISTANCE_M
        )
        if not is_last and seg.distance_m is not None:
            # Inflate target so profile cruises through the actual endpoint
            config.distance_m = actual + math.copysign(self._OVERSHOOT_M, actual)
        else:
            config.distance_m = actual
        config.speed_scale = seg.speed_scale

        if seg.heading_deg is not None:
            from raccoon.robot.heading_reference import HeadingReferenceService
            ref_svc = robot.get_service(HeadingReferenceService)
            sign = 1.0 if ref_svc._positive_direction == "left" else -1.0
            config.target_heading_rad = (
                ref_svc._reference_rad + sign * math.radians(seg.heading_deg)
            )

        motion = LinearMotion(
            robot.drive, robot.odometry,
            robot.motion_pid_config, config,
        )
        if not is_last:
            motion.set_suppress_hard_stop(True)
        return motion

    def _create_turn_motion(
        self, robot: "GenericRobot", seg: _Segment, is_last: bool,
    ) -> TurnMotion:
        config = TurnConfig()
        actual = (
            seg.angle_rad
            if seg.angle_rad is not None
            else seg.sign * math.radians(180)  # sentinel for condition-based
        )
        if not is_last and seg.angle_rad is not None:
            config.target_angle_rad = actual + math.copysign(self._OVERSHOOT_RAD, actual)
        else:
            config.target_angle_rad = actual
        config.speed_scale = seg.speed_scale

        motion = TurnMotion(
            robot.drive, robot.odometry,
            robot.motion_pid_config, config,
        )
        if not is_last:
            motion.set_suppress_hard_stop(True)
        return motion

    def _create_arc_motion(
        self, robot: "GenericRobot", seg: _Segment, is_last: bool,
    ) -> ArcMotion:
        config = ArcMotionConfig()
        config.radius_m = seg.radius_m
        actual = seg.arc_angle_rad
        if not is_last and actual is not None:
            config.arc_angle_rad = actual + math.copysign(self._OVERSHOOT_RAD, actual)
        else:
            config.arc_angle_rad = actual
        config.speed_scale = seg.speed_scale
        config.lateral = seg.lateral

        motion = ArcMotion(
            robot.drive, robot.odometry,
            robot.motion_pid_config, config,
        )
        if not is_last:
            motion.set_suppress_hard_stop(True)
        return motion

    def _create_motion(self, robot: "GenericRobot", seg: _Segment, is_last: bool):
        if seg.kind == "linear":
            return self._create_linear_motion(robot, seg, is_last)
        elif seg.kind == "turn":
            return self._create_turn_motion(robot, seg, is_last)
        elif seg.kind == "arc":
            return self._create_arc_motion(robot, seg, is_last)
        raise ValueError(f"Unknown segment kind: {seg.kind}")

    # ------------------------------------------------------------------
    # Segment completion check
    # ------------------------------------------------------------------

    @staticmethod
    def _has_reached_target(motion, seg: _Segment) -> bool:
        """Check if the motion has reached its distance/angle target.

        Used only for the **last** segment where the profile naturally
        decelerates to zero at the target.
        """
        if seg.kind == "linear":
            return motion.has_reached_distance()
        elif seg.kind in ("turn", "arc"):
            return motion.has_reached_angle()
        return motion.is_finished()

    def _check_segment_reached(
        self, robot: "GenericRobot", seg: _Segment, seg_origin: float,
    ) -> bool:
        """Manual position check for non-terminal segments.

        Compares odometry-based distance traveled against the actual
        segment target. Used instead of the C++ check when the profile
        target is inflated with overshoot.
        """
        current = self._get_position_offset(robot, seg)
        traveled = current - seg_origin

        if seg.kind == "linear":
            target = seg.distance_m
            tol = self._DISTANCE_TOL_M
        elif seg.kind == "turn":
            target = seg.angle_rad
            tol = self._ANGLE_TOL_RAD
        elif seg.kind == "arc":
            target = seg.arc_angle_rad
            tol = self._ANGLE_TOL_RAD
        else:
            return False

        if target is None:
            return False
        if target >= 0:
            return traveled >= target - tol
        else:
            return traveled <= target + tol

    # ------------------------------------------------------------------
    # Velocity reading
    # ------------------------------------------------------------------

    @staticmethod
    def _get_current_velocity(motion, seg: _Segment) -> float:
        """Read the current filtered velocity from the active motion."""
        return motion.get_filtered_velocity()

    # ------------------------------------------------------------------
    # Odometry offset for warm start
    # ------------------------------------------------------------------

    @staticmethod
    def _get_position_offset(robot: "GenericRobot", seg: _Segment) -> float:
        """Get the current odometry position to use as offset for the next segment."""
        if seg.kind == "linear":
            info = robot.odometry.get_distance_from_origin()
            if seg.axis == LinearAxis.Forward:
                return info.forward
            else:
                return info.lateral
        elif seg.kind in ("turn", "arc"):
            return robot.odometry.get_heading()
        return 0.0

    # ------------------------------------------------------------------
    # Lazy deferred resolution
    # ------------------------------------------------------------------

    @staticmethod
    def _resolve_node_at(
        robot: "GenericRobot",
        nodes: list[Optional[_PathNode]],
        idx: int,
        deferred_map: dict[int, Defer],
    ) -> None:
        """Resolve a Defer placeholder at *idx* in-place.

        The Defer factory is called with the robot's **current** state
        (heading, position, etc.), so heading-dependent steps like
        ``turn_to_heading_right`` compute the correct angle at the
        actual transition point — not at the start of the path.

        If the Defer resolves to a composite (seq, parallel), the
        resulting nodes are spliced into *nodes* replacing the single
        ``None`` placeholder.
        """
        defer = deferred_map.pop(idx)
        resolved_step = defer.factory(robot)
        sub_nodes, sub_deferred = _flatten_steps([resolved_step])
        if sub_deferred:
            raise TypeError(
                "smooth_path(): a Defer step resolved to another "
                "Defer — nested deferral is not supported"
            )
        # Filter out any None sub-nodes (shouldn't happen, but be safe)
        resolved = [n for n in sub_nodes if n is not None]
        # Splice: replace the single None with the resolved node(s)
        nodes[idx:idx + 1] = resolved
        # Shift deferred_map keys that are after idx
        if resolved:
            shift = len(resolved) - 1  # we replaced 1 entry with N
            if shift != 0:
                new_map = {}
                for k, v in deferred_map.items():
                    new_map[k + shift if k > idx else k] = v
                deferred_map.clear()
                deferred_map.update(new_map)

    # ------------------------------------------------------------------
    # Side action execution
    # ------------------------------------------------------------------

    @staticmethod
    async def _advance_past_side_actions(
        robot: "GenericRobot",
        nodes: list[Optional[_PathNode]],
        start_idx: int,
        bg_tasks: list[asyncio.Task],
        deferred_map: dict[int, Defer],
    ) -> int:
        """Execute consecutive side actions from *start_idx*.

        Resolves any Defer placeholders encountered along the way.
        Returns the index of the next _Segment node (or len(nodes)).
        """
        idx = start_idx
        while idx < len(nodes):
            # Resolve deferred node if needed
            if nodes[idx] is None and idx in deferred_map:
                SmoothPath._resolve_node_at(robot, nodes, idx, deferred_map)
            node = nodes[idx]
            if not isinstance(node, _SideAction):
                break  # found a _Segment (or unexpected None)
            if node.is_background:
                task = asyncio.create_task(node.step.run_step(robot))
                bg_tasks.append(task)
            else:
                await node.step.run_step(robot)
            idx += 1
        return idx

    @staticmethod
    def _is_last_segment(
        nodes: list[Optional[_PathNode]], current_idx: int,
    ) -> bool:
        """Check if there are no more _Segment nodes after *current_idx*.

        Unresolved Defer placeholders (``None``) are treated as potential
        segments (conservative — no overshoot, profile decelerates).
        """
        for i in range(current_idx + 1, len(nodes)):
            if isinstance(nodes[i], _Segment) or nodes[i] is None:
                return False
        return True

    # ------------------------------------------------------------------
    # Main execution loop
    # ------------------------------------------------------------------

    async def _execute_step(self, robot: "GenericRobot") -> None:
        # 1. Build mutable working copy of nodes and deferred map
        nodes: list[Optional[_PathNode]] = list(self._nodes)
        deferred_map: dict[int, Defer] = dict(self._deferred)

        # 2. Calibration check (for segments known at construction time)
        has_calibrated_drive = any(
            isinstance(n, _Segment) and n.kind == "linear" and n.has_known_endpoint
            for n in nodes
        )
        if has_calibrated_drive:
            from raccoon.step.calibration import check_distance_calibration
            check_distance_calibration()

        # 3. Execute the path
        bg_tasks: list[asyncio.Task] = []

        try:
            node_idx = 0

            # Execute any leading side actions (resolves Defers lazily)
            node_idx = await self._advance_past_side_actions(
                robot, nodes, node_idx, bg_tasks, deferred_map,
            )

            if node_idx >= len(nodes):
                return  # only side actions

            # Resolve deferred first segment if needed
            if nodes[node_idx] is None and node_idx in deferred_map:
                self._resolve_node_at(robot, nodes, node_idx, deferred_map)

            # Runtime validation: first node must be a segment
            if not isinstance(nodes[node_idx], _Segment):
                raise ValueError(
                    "smooth_path() resolved to no motion segments — "
                    "at least one drive, turn, or arc step is required"
                )

            # Start first motion segment (cold start)
            seg: _Segment = nodes[node_idx]  # type: ignore[assignment]
            is_last = self._is_last_segment(nodes, node_idx)
            motion = self._create_motion(robot, seg, is_last)
            motion.start()
            seg_origin = self._get_position_offset(robot, seg)

            if seg.condition is not None:
                seg.condition.start(robot)

            # 4. Unified control loop
            update_rate = 1.0 / self.hz
            loop = asyncio.get_event_loop()
            last_time = loop.time() - update_rate  # seed first dt

            while True:
                current_time = loop.time()
                dt = max(current_time - last_time, 0.0)
                last_time = current_time

                if dt < 1e-4:
                    await asyncio.sleep(update_rate)
                    continue

                transition = False

                # Check condition-based termination
                if seg.condition is not None and seg.condition.check(robot):
                    transition = True

                # Always update the motion controller this cycle
                motion.update(dt)

                # Check distance/angle completion
                if not transition and seg.has_known_endpoint:
                    if is_last:
                        # Last segment: use C++ check (profile decelerates
                        # naturally to the real target)
                        transition = self._has_reached_target(motion, seg)
                    else:
                        # Non-terminal: manual check against actual target
                        # while the profile's inflated target keeps the
                        # robot cruising at full speed
                        transition = self._check_segment_reached(
                            robot, seg, seg_origin,
                        )

                if transition:
                    # Read current velocity before transitioning
                    current_vel = self._get_current_velocity(motion, seg)
                    prev_seg = seg

                    node_idx += 1

                    # Execute side actions at transition point
                    # (also lazily resolves any Defer side-actions)
                    node_idx = await self._advance_past_side_actions(
                        robot, nodes, node_idx, bg_tasks, deferred_map,
                    )

                    if node_idx >= len(nodes):
                        break  # path complete

                    # Lazily resolve Defer segment at transition point
                    # so heading-dependent steps see the current heading
                    if nodes[node_idx] is None and node_idx in deferred_map:
                        self._resolve_node_at(
                            robot, nodes, node_idx, deferred_map,
                        )

                    # After resolution, skip any new side actions
                    node_idx = await self._advance_past_side_actions(
                        robot, nodes, node_idx, bg_tasks, deferred_map,
                    )

                    if node_idx >= len(nodes):
                        break

                    seg = nodes[node_idx]  # type: ignore[assignment]
                    is_last = self._is_last_segment(nodes, node_idx)

                    if _is_same_type(prev_seg, seg):
                        # Same type: warm start — carry velocity seamlessly
                        offset = self._get_position_offset(robot, prev_seg)
                        motion = self._create_motion(robot, seg, is_last)
                        motion.start_warm(offset, current_vel)
                    else:
                        # Cross-type: cold start next (no soft_stop — let
                        # the new controller take over immediately while
                        # the old axis velocity coasts down naturally)
                        motion = self._create_motion(robot, seg, is_last)
                        motion.start()

                    seg_origin = self._get_position_offset(robot, seg)

                    if seg.condition is not None:
                        seg.condition.start(robot)

                await asyncio.sleep(update_rate)
        finally:
            robot.drive.hard_stop()
            # Clean up any background side-action tasks
            for task in bg_tasks:
                if not task.done():
                    task.cancel()
            if bg_tasks:
                await asyncio.gather(*bg_tasks, return_exceptions=True)


# ---------------------------------------------------------------------------
# DSL factory function
# ---------------------------------------------------------------------------

@dsl(tags=["motion", "path"])
def smooth_path(*steps) -> SmoothPath:
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

    Prerequisites:
        ``calibrate_distance()`` if any segment uses distance-based mode.
        ``mark_heading_reference()`` if any segment uses heading hold.

    Args:
        *steps: Motion steps to execute smoothly. Accepts
            ``drive_forward``, ``drive_backward``, ``strafe_left``,
            ``strafe_right``, ``turn_left``, ``turn_right``,
            ``turn_to_heading_left``, ``turn_to_heading_right``,
            ``drive_arc_left``, ``drive_arc_right``, nested ``seq()``,
            ``parallel()`` with a motion spine, ``background()``,
            and their builder variants with ``.until()`` and ``.speed()``.

    Returns:
        A SmoothPath step that executes the segments as one fluid motion.

    Example::

        from raccoon.step.motion import smooth_path, drive_forward, turn_to_heading_right
        from raccoon.step.condition import on_black

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

        # Condition-based segment carries velocity into the next
        smooth_path(
            drive_forward(speed=0.8).until(on_black(sensor)),
            drive_forward(20),
        )

        # Parallel: servo moves while driving continuously
        smooth_path(
            drive_forward(30),
            parallel(
                [drive_forward(20), drive_forward(10)],
                [servo_move(0, 1500)],
            ),
        )

        # Background: non-blocking step alongside motion
        smooth_path(
            drive_forward(30),
            background(servo_move(0, 1500)),
            drive_forward(20),
        )
    """
    return SmoothPath(list(steps))
