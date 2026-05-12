"""PathExecutor — runs a compiled plan with smooth segment transitions.

Drives the unified control loop: cycle the active motion, watch for
condition / distance / angle / opaque completion, then transition to the
next segment with the appropriate warm-vs-cold start, executing any side
actions in between.
"""

from __future__ import annotations

import asyncio
import math
from typing import TYPE_CHECKING

from raccoon.foundation import Pose
from raccoon.localization import Observation
from raccoon.motion import LinearAxis
from raccoon.step.calibration import check_distance_calibration

from .._heading_utils import get_world_heading_rad
from .abs_ir import Action, Goto, Resync, TurnTo
from .ir import PathNode, Segment, SideAction
from .motion_factory import create_motion
from .passes import flatten_steps, is_same_type


async def _world_heading_for_seg(robot: "GenericRobot", seg: Segment) -> float | None:
    """Read absolute world heading for segment kinds that need it.

    Linear segments require ``target_heading_rad`` on the config; the rest
    (turn/arc are delta-based; follow_line/spline are opaque adapters) do
    not read this argument and we skip the read so executor tests for
    those kinds don't have to mock localization.

    Localization is a background pass-through service. At segment
    boundaries, the just-finished motion has updated odometry in the same
    event-loop tick; yielding briefly lets localization publish that pose
    before the next linear segment freezes its absolute heading target.
    """
    if seg.kind == "linear" or (seg.kind == "turn" and seg.target_heading_rad is not None):
        odom_heading = 0.0
        for _ in range(10):
            world_heading = get_world_heading_rad(robot)
            odom_heading = float(robot.odometry.get_pose().heading)
            if abs(math.remainder(world_heading - odom_heading, 2.0 * math.pi)) < 0.02:
                return world_heading
            await asyncio.sleep(0.005)
        return odom_heading
    return None


if TYPE_CHECKING:
    from raccoon.robot.api import GenericRobot

    from ...logic.defer import Defer
    from .. import Step


# ---------------------------------------------------------------------------
# Per-segment manual completion check tolerances
# ---------------------------------------------------------------------------

DISTANCE_TOL_M = 0.005  # 5mm tolerance for manual distance check
ANGLE_TOL_RAD = 0.035  # ~2° tolerance for manual angle check
ABS_AXIS_TOL_M = 0.02  # absolute runtime accepts 2cm off-axis drift
ABS_ARC_TOL_M = 0.03  # arc reconstruction must fit within 3cm
RESYNC_SIGMA_TIGHT = 1e-3
ZERO_DISTANCE_TOL_M = 0.005


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
    motion,
) -> bool:
    """Manual position check for non-terminal segments.

    Compares odometry-based distance traveled against the segment target.
    Used instead of the C++ check when the profile target is inflated with
    overshoot.
    """
    current = _get_position_offset(robot, seg)
    traveled = current - seg_origin

    if seg.kind == "linear":
        target = seg.distance_m
        tol = DISTANCE_TOL_M
    elif seg.kind == "turn":
        if seg.target_heading_rad is not None:
            return motion.has_reached_angle()
        target = seg.angle_rad
        tol = ANGLE_TOL_RAD
    elif seg.kind == "arc":
        target = seg.arc_angle_rad
        tol = ANGLE_TOL_RAD
    else:
        return False

    if target is None:
        return False
    if target >= 0:
        return traveled >= target - tol
    return traveled <= target + tol


def _wrap_angle(angle_rad: float) -> float:
    return math.remainder(angle_rad, 2.0 * math.pi)


def _world_to_body(dx_m: float, dy_m: float, theta_rad: float) -> tuple[float, float]:
    return (
        dx_m * math.cos(theta_rad) + dy_m * math.sin(theta_rad),
        dx_m * math.sin(theta_rad) - dy_m * math.cos(theta_rad),
    )


def _current_runtime_pose(robot: "GenericRobot"):
    """Pose snapshot for absolute runtime lowering.

    Position is taken from odometry so segment boundaries see the same pose
    source the motion controllers just updated. Heading stays on the Phase-4
    world-frame helper, which already falls back to odometry when localization
    lags a tick behind.
    """
    odom_pose = robot.odometry.get_pose()
    pose = Pose()
    pose.position = odom_pose.position
    pose.heading = get_world_heading_rad(robot)
    return pose


def _infer_arc_geometry(
    forward_m: float,
    strafe_right_m: float,
    angle_rad: float,
) -> tuple[float, bool]:
    theta = abs(angle_rad)
    if theta <= 1e-6:
        msg = "absolute Goto(via='arc') requires a non-zero heading delta"
        raise RuntimeError(msg)

    candidates: list[tuple[float, bool, float]] = []

    sin_theta = math.sin(theta)
    one_minus_cos = 1.0 - math.cos(theta)
    if abs(sin_theta) > 1e-9:
        radius = abs(forward_m) / abs(sin_theta)
        expected_strafe = radius * one_minus_cos * (-1.0 if angle_rad > 0.0 else 1.0)
        err = abs(forward_m - radius * sin_theta) + abs(strafe_right_m - expected_strafe)
        candidates.append((radius, False, err))

        radius = abs(strafe_right_m) / abs(sin_theta)
        expected_forward = radius * one_minus_cos
        expected_strafe = (1.0 if angle_rad < 0.0 else -1.0) * radius * sin_theta
        err = abs(forward_m - expected_forward) + abs(strafe_right_m - expected_strafe)
        candidates.append((radius, True, err))

    if not candidates:
        msg = "absolute Goto(via='arc') could not infer arc geometry"
        raise RuntimeError(msg)

    radius_m, lateral, error_m = min(candidates, key=lambda item: item[2])
    if not math.isfinite(radius_m) or radius_m <= 0.0 or error_m > ABS_ARC_TOL_M:
        msg = "absolute Goto(via='arc') does not match a safe runtime arc reconstruction"
        raise RuntimeError(msg)
    return radius_m, lateral


def _lower_absolute_turn(current_heading_rad: float, node: TurnTo) -> Segment:
    angle_rad = _wrap_angle(node.theta_rad - current_heading_rad)
    return Segment(
        kind="turn",
        sign=1.0 if angle_rad >= 0.0 else -1.0,
        angle_rad=angle_rad,
        target_heading_rad=node.theta_rad,
        has_known_endpoint=True,
    )


def _lower_absolute_goto(current_pose, node: Goto) -> Segment:
    heading_rad = float(current_pose.heading)
    dx_m = float(node.x_m) - float(current_pose.position[0])
    dy_m = float(node.y_m) - float(current_pose.position[1])
    forward_m, strafe_right_m = _world_to_body(dx_m, dy_m, heading_rad)
    target_heading_rad = heading_rad if node.theta_rad is None else node.theta_rad

    if node.via == "forward":
        if abs(strafe_right_m) > ABS_AXIS_TOL_M:
            msg = "absolute Goto(via='forward') is not body-collinear at runtime"
            raise RuntimeError(msg)
        distance_m = math.copysign(math.hypot(forward_m, strafe_right_m), forward_m)
        return Segment(
            kind="linear",
            axis=LinearAxis.Forward,
            sign=1.0 if distance_m >= 0.0 else -1.0,
            distance_m=distance_m,
            speed_scale=node.speed_scale,
            target_heading_rad=target_heading_rad,
            has_known_endpoint=True,
        )

    if node.via == "lateral":
        if abs(forward_m) > ABS_AXIS_TOL_M:
            msg = "absolute Goto(via='lateral') is not body-collinear at runtime"
            raise RuntimeError(msg)
        distance_m = math.copysign(math.hypot(forward_m, strafe_right_m), strafe_right_m)
        return Segment(
            kind="linear",
            axis=LinearAxis.Lateral,
            sign=1.0 if distance_m >= 0.0 else -1.0,
            distance_m=distance_m,
            speed_scale=node.speed_scale,
            target_heading_rad=target_heading_rad,
            has_known_endpoint=True,
        )

    if node.via == "auto":
        if abs(strafe_right_m) <= ABS_AXIS_TOL_M:
            return Segment(
                kind="linear",
                axis=LinearAxis.Forward,
                sign=1.0 if forward_m >= 0.0 else -1.0,
                distance_m=forward_m,
                speed_scale=node.speed_scale,
                target_heading_rad=target_heading_rad,
                has_known_endpoint=True,
            )
        if abs(forward_m) <= ABS_AXIS_TOL_M:
            return Segment(
                kind="linear",
                axis=LinearAxis.Lateral,
                sign=1.0 if strafe_right_m >= 0.0 else -1.0,
                distance_m=strafe_right_m,
                speed_scale=node.speed_scale,
                target_heading_rad=target_heading_rad,
                has_known_endpoint=True,
            )
        msg = "absolute Goto(via='auto') needs a single-axis runtime delta"
        raise RuntimeError(msg)

    if node.via == "arc":
        if node.theta_rad is None:
            msg = "absolute Goto(via='arc') requires theta_rad"
            raise RuntimeError(msg)
        arc_angle_rad = _wrap_angle(node.theta_rad - heading_rad)
        radius_m, lateral = _infer_arc_geometry(forward_m, strafe_right_m, arc_angle_rad)
        return Segment(
            kind="arc",
            radius_m=radius_m,
            arc_angle_rad=arc_angle_rad,
            speed_scale=node.speed_scale,
            lateral=lateral,
            has_known_endpoint=True,
        )

    msg = f"unsupported absolute Goto(via={node.via!r})"
    raise RuntimeError(msg)


def _lower_absolute_goto_segments(current_pose, node: Goto) -> list[Segment]:
    heading_rad = float(current_pose.heading)
    dx_m = float(node.x_m) - float(current_pose.position[0])
    dy_m = float(node.y_m) - float(current_pose.position[1])
    distance_m = math.hypot(dx_m, dy_m)
    final_heading_rad = heading_rad if node.theta_rad is None else node.theta_rad

    if node.via == "forward" and distance_m > ZERO_DISTANCE_TOL_M:
        path_heading_rad = math.atan2(dy_m, dx_m)
        segments: list[Segment] = []
        turn_in_rad = _wrap_angle(path_heading_rad - heading_rad)
        if abs(turn_in_rad) > ANGLE_TOL_RAD:
            segments.append(
                Segment(
                    kind="turn",
                    sign=1.0 if turn_in_rad >= 0.0 else -1.0,
                    angle_rad=turn_in_rad,
                    target_heading_rad=path_heading_rad,
                    has_known_endpoint=True,
                )
            )
        segments.append(
            Segment(
                kind="linear",
                axis=LinearAxis.Forward,
                sign=1.0,
                distance_m=distance_m,
                speed_scale=node.speed_scale,
                target_heading_rad=path_heading_rad,
                has_known_endpoint=True,
            )
        )
        turn_out_rad = _wrap_angle(final_heading_rad - path_heading_rad)
        if abs(turn_out_rad) > ANGLE_TOL_RAD:
            segments.append(
                Segment(
                    kind="turn",
                    sign=1.0 if turn_out_rad >= 0.0 else -1.0,
                    angle_rad=turn_out_rad,
                    target_heading_rad=final_heading_rad,
                    has_known_endpoint=True,
                )
            )
        return segments

    if distance_m <= ZERO_DISTANCE_TOL_M:
        if node.theta_rad is None:
            return []
        turn_rad = _wrap_angle(node.theta_rad - heading_rad)
        if abs(turn_rad) <= ANGLE_TOL_RAD:
            return []
        return [
            Segment(
                kind="turn",
                sign=1.0 if turn_rad >= 0.0 else -1.0,
                angle_rad=turn_rad,
                target_heading_rad=node.theta_rad,
                has_known_endpoint=True,
            )
        ]

    return [_lower_absolute_goto(current_pose, node)]


def _segments_for_absolute_node(robot: "GenericRobot", node: Goto | TurnTo) -> list[Segment]:
    if isinstance(node, TurnTo):
        return [_lower_absolute_turn(get_world_heading_rad(robot), node)]
    return _lower_absolute_goto_segments(_current_runtime_pose(robot), node)


def _resolve_absolute_motion_node(
    robot: "GenericRobot",
    nodes: list[Goto | TurnTo | Resync | Action | Segment],
    idx: int,
) -> None:
    node = nodes[idx]
    if isinstance(node, Segment):
        return
    if not isinstance(node, Goto | TurnTo):
        return
    nodes[idx : idx + 1] = _segments_for_absolute_node(robot, node)


def _resync_sigma(node: Resync) -> tuple[float, float, float]:
    return tuple(RESYNC_SIGMA_TIGHT if snap else math.inf for snap in node.snap_axes)


def _apply_resync(robot: "GenericRobot", node: Resync) -> None:
    loc = getattr(robot, "localization", None)
    if loc is None:
        msg = "absolute Resync requires robot.localization"
        raise RuntimeError(msg)

    pose = loc.get_pose()
    obs_pose = Pose()
    x_m = float(pose.position[0]) if node.expected_x_m is None else node.expected_x_m
    y_m = float(pose.position[1]) if node.expected_y_m is None else node.expected_y_m
    z_m = float(pose.position[2]) if len(pose.position) > 2 else 0.0
    obs_pose.position = (x_m, y_m, z_m)
    obs_pose.heading = pose.heading
    if node.expected_theta_rad is not None:
        obs_pose.heading = node.expected_theta_rad
    loc.observe(Observation(pose=obs_pose, sigma=_resync_sigma(node)))


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
    (warm/cold start), and executes side actions at transition points.
    """

    DEFAULT_HZ = 100

    def __init__(
        self,
        nodes: list[PathNode | None],
        deferred: list[tuple[int, "Defer"]],
        absolute_nodes: tuple[Goto | TurnTo | Resync | Action, ...] | None = None,
        spline_step: "Step" | None = None,
        hz: int = DEFAULT_HZ,
    ) -> None:
        self._nodes = nodes
        self._deferred = deferred
        self._absolute_nodes = absolute_nodes
        self._spline_step = spline_step
        self._hz = hz

    # -- Main entrypoint ---------------------------------------------------

    async def run(self, robot: "GenericRobot") -> None:
        # Spline-mode shortcut: a spline-conversion pass replaced the path
        # with a single SplinePath step; just delegate to it.
        if self._spline_step is not None:
            await self._spline_step._execute_step(robot)
            return
        if self._absolute_nodes is not None:
            await self._run_absolute(robot)
            return

        # 1. Mutable working copies (deferreds get popped, nodes get spliced).
        nodes: list[PathNode | None] = list(self._nodes)
        deferred_map: dict[int, "Defer"] = dict(self._deferred)

        # 2. Calibration check: any known-distance linear requires calibration.
        has_calibrated_drive = any(
            isinstance(n, Segment) and n.kind == "linear" and n.has_known_endpoint for n in nodes
        )
        if has_calibrated_drive:
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

            motion = create_motion(
                robot,
                seg,
                is_last,
                current_world_heading_rad=await _world_heading_for_seg(robot, seg),
            )
            motion.start()
            seg_origin = _get_position_offset(robot, seg)

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
                            motion,
                        )

                # Opaque steps: adapter signals completion via is_finished().
                if not transition and seg.kind in ("follow_line", "spline"):
                    transition = motion.is_finished()

                if transition:
                    current_vel = motion.get_filtered_velocity()
                    prev_seg = seg

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
                    if nodes[node_idx] is None and node_idx in deferred_map:
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

                    next_world_heading = await _world_heading_for_seg(robot, seg)
                    if is_same_type(prev_seg, seg):
                        # Same type: warm start — carry velocity seamlessly.
                        offset = _get_position_offset(robot, prev_seg)
                        motion = create_motion(
                            robot,
                            seg,
                            is_last,
                            current_world_heading_rad=next_world_heading,
                        )
                        motion.start_warm(offset, current_vel)
                    else:
                        # Cross-type: cold start.
                        robot.drive.hard_stop()
                        motion = create_motion(
                            robot,
                            seg,
                            is_last,
                            current_world_heading_rad=next_world_heading,
                        )
                        motion.start()

                    seg_origin = _get_position_offset(robot, seg)

                    if seg.condition is not None:
                        seg.condition.start(robot)

                await asyncio.sleep(update_rate)
        finally:
            robot.drive.hard_stop()
            # Clean up any background side-action tasks.
            for task in bg_tasks:
                if not task.done():
                    task.cancel()
            if bg_tasks:
                await asyncio.gather(*bg_tasks, return_exceptions=True)

    async def _run_absolute(self, robot: "GenericRobot") -> None:
        absolute_nodes: list[Goto | TurnTo | Resync | Action | Segment] = list(
            self._absolute_nodes or ()
        )
        bg_tasks: list[asyncio.Task] = []

        try:
            node_idx = 0
            while node_idx < len(absolute_nodes) and not isinstance(
                absolute_nodes[node_idx], Segment
            ):
                node = absolute_nodes[node_idx]
                _resolve_absolute_motion_node(robot, absolute_nodes, node_idx)
                if node_idx >= len(absolute_nodes):
                    break
                if node_idx < len(absolute_nodes) and isinstance(absolute_nodes[node_idx], Segment):
                    break
                node = absolute_nodes[node_idx]
                if isinstance(node, Action):
                    if node.blocking:
                        await node.step.run_step(robot)
                    else:
                        bg_tasks.append(asyncio.create_task(node.step.run_step(robot)))
                elif isinstance(node, Resync):
                    _apply_resync(robot, node)
                node_idx += 1

            if node_idx >= len(absolute_nodes):
                return

            _resolve_absolute_motion_node(robot, absolute_nodes, node_idx)
            if node_idx >= len(absolute_nodes):
                return
            seg = absolute_nodes[node_idx]
            if not isinstance(seg, Segment):
                msg = "absolute runtime lowered to no executable motion segment"
                raise RuntimeError(msg)
            is_last = not any(
                isinstance(node, Goto | TurnTo | Segment) for node in absolute_nodes[node_idx + 1 :]
            )
            motion = create_motion(
                robot,
                seg,
                is_last,
                current_world_heading_rad=await _world_heading_for_seg(robot, seg),
            )
            motion.start()
            seg_origin = _get_position_offset(robot, seg)

            update_rate = 1.0 / self._hz
            loop = asyncio.get_event_loop()
            last_time = loop.time() - update_rate

            while True:
                current_time = loop.time()
                dt = max(current_time - last_time, 0.0)
                last_time = current_time

                if dt < 1e-4:
                    await asyncio.sleep(update_rate)
                    continue

                motion.update(dt)
                if is_last:
                    transition = _has_reached_target(motion, seg)
                else:
                    transition = _check_segment_reached(robot, seg, seg_origin, motion)

                if not transition and seg.kind in ("follow_line", "spline"):
                    transition = motion.is_finished()

                if transition:
                    current_vel = motion.get_filtered_velocity()
                    prev_seg = seg
                    node_idx += 1

                    while node_idx < len(absolute_nodes) and not isinstance(
                        absolute_nodes[node_idx], Segment
                    ):
                        node = absolute_nodes[node_idx]
                        _resolve_absolute_motion_node(robot, absolute_nodes, node_idx)
                        if node_idx >= len(absolute_nodes):
                            break
                        if node_idx < len(absolute_nodes) and isinstance(
                            absolute_nodes[node_idx], Segment
                        ):
                            break
                        node = absolute_nodes[node_idx]
                        if isinstance(node, Action):
                            if node.blocking:
                                await node.step.run_step(robot)
                            else:
                                bg_tasks.append(asyncio.create_task(node.step.run_step(robot)))
                        elif isinstance(node, Resync):
                            _apply_resync(robot, node)
                        node_idx += 1

                    if node_idx >= len(absolute_nodes):
                        break

                    _resolve_absolute_motion_node(robot, absolute_nodes, node_idx)
                    if node_idx >= len(absolute_nodes):
                        break
                    seg = absolute_nodes[node_idx]
                    if not isinstance(seg, Segment):
                        msg = "absolute runtime lowered to no executable motion segment"
                        raise RuntimeError(msg)
                    is_last = not any(
                        isinstance(node, Goto | TurnTo | Segment)
                        for node in absolute_nodes[node_idx + 1 :]
                    )
                    next_world_heading = await _world_heading_for_seg(robot, seg)
                    if is_same_type(prev_seg, seg):
                        offset = _get_position_offset(robot, prev_seg)
                        motion = create_motion(
                            robot,
                            seg,
                            is_last,
                            current_world_heading_rad=next_world_heading,
                        )
                        motion.start_warm(offset, current_vel)
                    else:
                        robot.drive.hard_stop()
                        motion = create_motion(
                            robot,
                            seg,
                            is_last,
                            current_world_heading_rad=next_world_heading,
                        )
                        motion.start()
                    seg_origin = _get_position_offset(robot, seg)

                await asyncio.sleep(update_rate)
        finally:
            robot.drive.hard_stop()
            for task in bg_tasks:
                if not task.done():
                    task.cancel()
            if bg_tasks:
                await asyncio.gather(*bg_tasks, return_exceptions=True)
