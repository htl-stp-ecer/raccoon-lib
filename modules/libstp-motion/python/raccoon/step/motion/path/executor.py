"""PathExecutor — runs a compiled plan with smooth segment transitions.

Drives the unified control loop: cycle the active motion, watch for
condition / distance / angle / opaque completion, then transition to the
next segment with the appropriate warm-vs-cold start, executing any side
actions in between.

Middlewares are invoked at well-defined hook points (path start/end,
segment start/end, cold start) and may return a ``Correction`` that the
motion factory consumes when constructing the next motion.
"""

from __future__ import annotations

import asyncio
import math
from typing import TYPE_CHECKING

from raccoon.motion import LinearAxis

from .ir import Correction, PathNode, Segment, SideAction
from .motion_factory import create_motion
from .passes import flatten_steps, is_same_type

if TYPE_CHECKING:
    from raccoon.robot.api import GenericRobot

    from ...logic.defer import Defer
    from .. import Step
    from .middleware import PathMiddleware


# ---------------------------------------------------------------------------
# Per-segment manual completion check tolerances
# ---------------------------------------------------------------------------

DISTANCE_TOL_M = 0.005  # 5mm tolerance for manual distance check
ANGLE_TOL_RAD = 0.035  # ~2° tolerance for manual angle check


# ---------------------------------------------------------------------------
# Helpers — odometry reads, completion checks
# ---------------------------------------------------------------------------


def _has_reached_target(motion, seg: Segment) -> bool:
    """Did the motion reach its distance/angle target?

    Used for the **last** segment where the profile naturally decelerates
    to zero at the target.  Opaque segments (follow_line, spline) are
    always checked via ``is_finished()``.
    """
    if seg.kind == "linear":
        return motion.has_reached_distance()
    if seg.kind in ("turn", "arc"):
        return motion.has_reached_angle()
    return motion.is_finished()


def _get_position_offset(robot: "GenericRobot", seg: Segment) -> float:
    """Get the current odometry position to use as offset for warm-start."""
    if seg.kind == "linear":
        info = robot.odometry.get_distance_from_origin()
        return info.forward if seg.axis == LinearAxis.Forward else info.lateral
    if seg.kind in ("turn", "arc"):
        return robot.odometry.get_heading()
    if seg.kind == "follow_line":
        # Always forward-axis, same as LinearAxis.Forward.
        return robot.odometry.get_distance_from_origin().forward
    # "spline": not used for warm-start (splines always cold-start).
    return 0.0


def _check_segment_reached(
    robot: "GenericRobot",
    seg: Segment,
    seg_origin: float,
    correction: Correction | None = None,
) -> bool:
    """Manual position check for non-terminal segments.

    Compares odometry-based distance traveled against the (possibly
    corrected) segment target.  Used instead of the C++ check when the
    profile target is inflated with overshoot.
    """
    current = _get_position_offset(robot, seg)
    traveled = current - seg_origin

    if seg.kind == "linear":
        target = seg.distance_m
        if correction and target is not None:
            target -= math.copysign(correction.distance_adjust_m, target)
        tol = DISTANCE_TOL_M
    elif seg.kind == "turn":
        target = seg.angle_rad
        if correction and target is not None:
            target -= correction.angle_adjust_rad
        tol = ANGLE_TOL_RAD
    elif seg.kind == "arc":
        target = seg.arc_angle_rad
        if correction and target is not None:
            target -= correction.angle_adjust_rad
        tol = ANGLE_TOL_RAD
    else:
        return False

    if target is None:
        return False
    if target >= 0:
        return traveled >= target - tol
    return traveled <= target + tol


# ---------------------------------------------------------------------------
# Lazy deferred resolution & side-action execution
# ---------------------------------------------------------------------------


def _resolve_node_at(
    robot: "GenericRobot",
    nodes: list[PathNode | None],
    idx: int,
    deferred_map: dict[int, "Defer"],
) -> None:
    """Resolve a Defer placeholder at ``idx`` in-place.

    The Defer factory is called with the robot's *current* state, so
    heading-dependent steps like ``turn_to_heading_right`` compute the
    correct angle at the actual transition point — not at path start.

    If the Defer resolves to a composite, the resulting nodes are spliced
    into ``nodes`` replacing the single ``None`` placeholder, and any
    deferred-map indices after ``idx`` are shifted.
    """
    defer = deferred_map.pop(idx)
    resolved_step = defer.factory(robot)
    sub_nodes, sub_deferred = flatten_steps([resolved_step])
    if sub_deferred:
        msg = (
            "smooth_path(): a Defer step resolved to another Defer — "
            "nested deferral is not supported"
        )
        raise TypeError(msg)
    resolved = [n for n in sub_nodes if n is not None]
    nodes[idx : idx + 1] = resolved
    if resolved:
        shift = len(resolved) - 1  # we replaced 1 entry with N
        if shift != 0:
            new_map = {}
            for k, v in deferred_map.items():
                new_map[k + shift if k > idx else k] = v
            deferred_map.clear()
            deferred_map.update(new_map)


async def _advance_past_side_actions(
    robot: "GenericRobot",
    nodes: list[PathNode | None],
    start_idx: int,
    bg_tasks: list[asyncio.Task],
    deferred_map: dict[int, "Defer"],
) -> int:
    """Execute consecutive side actions from ``start_idx``.

    Resolves any Defer placeholders encountered along the way.  Returns
    the index of the next ``Segment`` node (or ``len(nodes)``).
    """
    idx = start_idx
    while idx < len(nodes):
        if nodes[idx] is None and idx in deferred_map:
            _resolve_node_at(robot, nodes, idx, deferred_map)
        node = nodes[idx]
        if not isinstance(node, SideAction):
            break  # found a Segment (or unexpected None)
        if node.is_background:
            task = asyncio.create_task(node.step.run_step(robot))
            bg_tasks.append(task)
        else:
            await node.step.run_step(robot)
        idx += 1
    return idx


def _is_last_segment(
    nodes: list[PathNode | None],
    current_idx: int,
) -> bool:
    """Are there no more Segment nodes after ``current_idx``?

    Unresolved Defer placeholders (``None``) are treated as potential
    segments (conservative — no overshoot, profile decelerates).
    """
    for i in range(current_idx + 1, len(nodes)):
        if isinstance(nodes[i], Segment) or nodes[i] is None:
            return False
    return True


# ---------------------------------------------------------------------------
# Executor
# ---------------------------------------------------------------------------


class PathExecutor:
    """Runs a compiled plan against a robot.

    The executor owns the control loop, manages segment transitions
    (warm/cold start), executes side actions at transition points, and
    fans out hook calls to registered middlewares.
    """

    DEFAULT_HZ = 100

    def __init__(
        self,
        nodes: list[PathNode | None],
        deferred: list[tuple[int, "Defer"]],
        spline_step: "Step" | None = None,
        middlewares: list["PathMiddleware"] | None = None,
        hz: int = DEFAULT_HZ,
    ) -> None:
        self._nodes = nodes
        self._deferred = deferred
        self._spline_step = spline_step
        self._middlewares: list["PathMiddleware"] = list(middlewares or [])
        self._hz = hz

    # -- Middleware fan-out ------------------------------------------------

    def _mw_path_start(self, robot: "GenericRobot") -> None:
        for mw in self._middlewares:
            mw.on_path_start(robot)

    def _mw_segment_start(
        self,
        seg: Segment,
        is_first: bool,
        robot: "GenericRobot",
    ) -> Correction | None:
        # If multiple middlewares produce a correction, the last one wins.
        # (For Phase 1 there's only WorldCorrection; merging strategies for
        # multi-middleware setups belong to the public-API layer.)
        result: Correction | None = None
        for mw in self._middlewares:
            c = mw.on_segment_start(seg, is_first, robot)
            if c is not None:
                result = c
        return result

    def _mw_cold_start(self, seg: Segment, robot: "GenericRobot") -> None:
        for mw in self._middlewares:
            mw.on_cold_start(seg, robot)

    def _mw_segment_end(self, seg: Segment, robot: "GenericRobot") -> None:
        for mw in self._middlewares:
            mw.on_segment_end(seg, robot)

    def _mw_path_end(self, robot: "GenericRobot") -> None:
        for mw in self._middlewares:
            mw.on_path_end(robot)

    # -- Main entrypoint ---------------------------------------------------

    async def run(self, robot: "GenericRobot") -> None:
        # Spline-mode shortcut: a spline-conversion pass replaced the path
        # with a single SplinePath step; just delegate to it.
        if self._spline_step is not None:
            await self._spline_step._execute_step(robot)
            return

        # 1. Mutable working copies (deferreds get popped, nodes get spliced).
        nodes: list[PathNode | None] = list(self._nodes)
        deferred_map: dict[int, "Defer"] = dict(self._deferred)

        # 2. Calibration check: any known-distance linear requires calibration.
        has_calibrated_drive = any(
            isinstance(n, Segment) and n.kind == "linear" and n.has_known_endpoint for n in nodes
        )
        if has_calibrated_drive:
            from raccoon.step.calibration import check_distance_calibration

            check_distance_calibration()

        # 3. Execute the path.
        bg_tasks: list[asyncio.Task] = []

        try:
            node_idx = 0

            # Leading side actions (resolves Defers lazily).
            node_idx = await _advance_past_side_actions(
                robot,
                nodes,
                node_idx,
                bg_tasks,
                deferred_map,
            )
            if node_idx >= len(nodes):
                return  # path was only side actions

            # Resolve deferred first segment if needed.
            if nodes[node_idx] is None and node_idx in deferred_map:
                _resolve_node_at(robot, nodes, node_idx, deferred_map)

            if not isinstance(nodes[node_idx], Segment):
                msg = (
                    "smooth_path() resolved to no motion segments — "
                    "at least one motion step is required"
                )
                raise ValueError(msg)

            # Start first segment (cold start).
            seg: Segment = nodes[node_idx]  # type: ignore[assignment]
            is_last = _is_last_segment(nodes, node_idx)

            # First motion is constructed without a correction (no prior
            # segment to correct against).  start() also resets the odometry,
            # which is why on_path_start must run AFTER start() — middlewares
            # like WorldCorrection initialize their tracker against the
            # post-reset frame.
            seg_correction = None
            motion = create_motion(robot, seg, is_last, seg_correction)
            motion.start()
            seg_origin = _get_position_offset(robot, seg)

            self._mw_path_start(robot)
            # Inform middlewares about the first segment too (so they can
            # update internal "is_first" state); the result is discarded
            # since the first motion was already constructed.
            self._mw_segment_start(seg, is_first=True, robot=robot)

            if seg.condition is not None:
                seg.condition.start(robot)

            # 4. Unified control loop.
            update_rate = 1.0 / self._hz
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

                # Condition-based termination?
                if seg.condition is not None and seg.condition.check(robot):
                    transition = True

                # Always update the motion controller this cycle.
                motion.update(dt)

                # Distance/angle completion?
                if not transition and seg.has_known_endpoint:
                    if is_last:
                        # Profile decelerates naturally to the real target.
                        transition = _has_reached_target(motion, seg)
                    else:
                        # Manual check while the inflated profile keeps
                        # the robot at cruise speed.
                        transition = _check_segment_reached(
                            robot,
                            seg,
                            seg_origin,
                            seg_correction,
                        )

                # Opaque steps: adapter signals completion via is_finished().
                if not transition and seg.kind in ("follow_line", "spline"):
                    transition = motion.is_finished()

                if transition:
                    current_vel = motion.get_filtered_velocity()
                    prev_seg = seg

                    # Notify middlewares the segment ended (advances trackers).
                    self._mw_segment_end(prev_seg, robot)

                    node_idx += 1

                    # Side actions at this transition point.
                    node_idx = await _advance_past_side_actions(
                        robot,
                        nodes,
                        node_idx,
                        bg_tasks,
                        deferred_map,
                    )
                    if node_idx >= len(nodes):
                        break

                    # Was the next node a Defer?  If so, resolve it now —
                    # heading-dependent steps need the current heading.
                    was_deferred = nodes[node_idx] is None and node_idx in deferred_map
                    if was_deferred:
                        _resolve_node_at(
                            robot,
                            nodes,
                            node_idx,
                            deferred_map,
                        )

                    # Skip side actions that may have appeared post-resolution.
                    node_idx = await _advance_past_side_actions(
                        robot,
                        nodes,
                        node_idx,
                        bg_tasks,
                        deferred_map,
                    )
                    if node_idx >= len(nodes):
                        break

                    seg = nodes[node_idx]  # type: ignore[assignment]
                    is_last = _is_last_segment(nodes, node_idx)

                    # Ask middlewares for the next segment's correction.
                    seg_correction = self._mw_segment_start(
                        seg,
                        is_first=False,
                        robot=robot,
                    )
                    # Deferred heading turns already used the current heading
                    # to compute their angle — zero out angle correction so
                    # we don't double-correct.
                    if was_deferred and seg_correction is not None and seg.kind in ("turn", "arc"):
                        seg_correction.angle_adjust_rad = 0.0

                    if is_same_type(prev_seg, seg):
                        # Same type: warm start — carry velocity seamlessly.
                        offset = _get_position_offset(robot, prev_seg)
                        motion = create_motion(
                            robot,
                            seg,
                            is_last,
                            seg_correction,
                        )
                        motion.start_warm(offset, current_vel)
                    else:
                        # Cross-type: cold start.  Notify middlewares first
                        # so they can snapshot pre-reset state.
                        self._mw_cold_start(seg, robot)
                        robot.drive.hard_stop()
                        motion = create_motion(
                            robot,
                            seg,
                            is_last,
                            seg_correction,
                        )
                        motion.start()

                    seg_origin = _get_position_offset(robot, seg)

                    if seg.condition is not None:
                        seg.condition.start(robot)

                await asyncio.sleep(update_rate)
        finally:
            self._mw_path_end(robot)
            robot.drive.hard_stop()
            # Clean up any background side-action tasks.
            for task in bg_tasks:
                if not task.done():
                    task.cancel()
            if bg_tasks:
                await asyncio.gather(*bg_tasks, return_exceptions=True)
