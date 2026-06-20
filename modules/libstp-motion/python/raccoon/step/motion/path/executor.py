"""PathExecutor — runs a compiled plan with smooth segment transitions.

Drives the unified control loop: cycle the active motion, watch for
condition / distance / angle / opaque completion, then transition to the
next segment with the appropriate warm-vs-cold start, executing any side
actions in between.
"""

from __future__ import annotations

import asyncio
import dataclasses
import math
from typing import TYPE_CHECKING

from raccoon.motion import LinearAxis
from raccoon.step.calibration import check_distance_calibration

from .._heading_utils import get_world_heading_rad
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


# ---------------------------------------------------------------------------
# Per-segment manual completion check tolerances
# ---------------------------------------------------------------------------

DISTANCE_TOL_M = 0.005  # 5mm tolerance for manual distance check
ANGLE_TOL_RAD = 0.035  # ~2° tolerance for manual angle check


# ---------------------------------------------------------------------------
# Helpers — odometry reads, completion checks
# ---------------------------------------------------------------------------


def _with_heading_offset(seg: Segment, h0: float) -> Segment:
    """Return ``seg`` with the path-start world heading ``h0`` folded in.

    ``AbsoluteHeadingPass`` stamps ``target_heading_rad`` integrated from 0 at
    PATH START (path-relative). The executor + motion factory consume
    ``target_heading_rad`` as an ABSOLUTE raw-odometry world heading, so each
    leg must be shifted by the world heading captured once at path start.

    Segments with ``target_heading_rad is None`` (the common case — the
    absolute_heading pass was not applied) are returned unchanged, so this is a
    no-op for every existing path. The original segment is never mutated; a
    ``dataclasses.replace`` copy is returned so the pass output (reused across
    runs / explain) stays intact.
    """
    if seg.target_heading_rad is None:
        return seg
    return dataclasses.replace(seg, target_heading_rad=seg.target_heading_rad + h0)


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
    """Get the current odometry position to use as offset for warm-start.

    For linear segments we project the world position onto the segment's
    *travel direction* so the distance check tracks actual progress regardless
    of how the robot is oriented relative to the odometry origin. The travel
    direction is ``target_heading_rad`` when the segment pins an (absolute)
    heading, otherwise the robot's current heading (relative / hold-current
    mode — the motion holds this heading throughout the leg). Falling back to
    ``get_distance_from_origin()`` was wrong whenever the leg drove along a
    heading different from the odometry origin axis: a short fixed-distance
    leg (e.g. a 3 cm grab drive) at a 90°-off heading then never registered as
    reached and the robot ran away. Forward uses the heading axis; Lateral the
    right-positive perpendicular.
    """
    if seg.kind == "linear":
        h = seg.target_heading_rad
        if h is None:
            h = float(robot.odometry.get_heading())
        pos = robot.odometry.get_pose().position
        x, y = float(pos[0]), float(pos[1])
        if seg.axis == LinearAxis.Forward:
            return x * math.cos(h) + y * math.sin(h)
        return -x * math.sin(h) + y * math.cos(h)
    if seg.kind in ("turn", "arc"):
        return robot.odometry.get_heading()
    if seg.kind == "follow_line":
        # Always forward-axis, same as LinearAxis.Forward.
        return robot.odometry.get_distance_from_origin().forward
    # "spline" / "diagonal": not used for warm-start (always cold-start).
    return 0.0


def _check_segment_reached(
    robot: "GenericRobot",
    seg: Segment,
    seg_origin: float,
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
    ephemeral_tasks: list[asyncio.Task],
) -> int:
    """Execute consecutive side actions from ``start_idx``.

    Resolves any Defer placeholders encountered along the way.  Returns
    the index of the next ``Segment`` node (or ``len(nodes)``).

    This is called at every segment transition, so it is also the join point
    for parallel() branches: any EPHEMERAL background task launched for a
    previous spine is awaited here (the spine that scoped it has now ended)
    before the next side actions / segment run — restoring parallel()'s
    await-all semantics so a branch can't leak past its scope.
    """
    # Join the previous spine's ephemeral parallel branches before proceeding.
    if ephemeral_tasks:
        pending = list(ephemeral_tasks)
        ephemeral_tasks.clear()
        await asyncio.gather(*pending, return_exceptions=True)

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
            if node.ephemeral:
                ephemeral_tasks.append(task)
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


def _next_segment_warmstarts(
    nodes: list[PathNode | None],
    current_idx: int,
    seg: Segment,
) -> bool:
    """Will the NEXT motion segment warm-start from ``seg`` (carry its velocity)?

    Only then should ``seg`` cruise through its endpoint (inflated profile);
    otherwise it must decelerate to its real target so a short move doesn't
    overshoot. The next segment cold-starts — so NO warm continuation — when:
      * there are no more segments (``seg`` is last),
      * a blocking inline side action runs before it (robot stops while it runs,
        then the next segment hard-stops + cold-starts),
      * an unresolved Defer (``None``) sits in between (conservative), or
      * the next segment is a different motion type (cross-type → cold start).
    Background / ephemeral parallel-branch side actions do NOT block the spine,
    so they are skipped over.
    """
    for i in range(current_idx + 1, len(nodes)):
        node = nodes[i]
        if node is None:
            return False  # unresolved Defer — be conservative (decelerate)
        if isinstance(node, SideAction):
            if not node.is_background:
                return False  # blocking inline side action → next cold-starts
            continue  # background / ephemeral branch — doesn't stop the spine
        return is_same_type(seg, node)  # next Segment: warm only if same type
    return False  # no further segment


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
        hz: int = DEFAULT_HZ,
    ) -> None:
        self._nodes = nodes
        self._deferred = deferred
        self._hz = hz

    # -- Main entrypoint ---------------------------------------------------

    async def run(self, robot: "GenericRobot") -> None:
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
        # Ephemeral parallel-branch tasks awaiting their join at the next
        # transition (see _advance_past_side_actions).
        ephemeral_tasks: list[asyncio.Task] = []

        try:
            node_idx = 0

            # Leading side actions (resolves Defers lazily).
            node_idx = await _advance_past_side_actions(
                robot,
                nodes,
                node_idx,
                bg_tasks,
                deferred_map,
                ephemeral_tasks,
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
            #
            # Capture the path-start world heading H0 ONCE, here at the first
            # real motion segment (after leading side actions advanced — those
            # may reorient the robot). AbsoluteHeadingPass emits headings in the
            # path-start frame (start = 0); H0 converts them to the absolute
            # raw-odometry world frame the motion factory regulates against.
            h0 = get_world_heading_rad(robot)

            seg: Segment = nodes[node_idx]  # type: ignore[assignment]
            seg = _with_heading_offset(seg, h0)
            is_last = _is_last_segment(nodes, node_idx)

            motion = create_motion(
                robot,
                seg,
                is_last,
                current_world_heading_rad=await _world_heading_for_seg(robot, seg),
                inflate=_next_segment_warmstarts(nodes, node_idx, seg),
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

                # Opaque adapters (follow_line / spline / diagonal, and a
                # heading TurnToHeading carried as an opaque turn) have no
                # measurable distance/angle target — they signal completion only
                # via is_finished(), so the geometric distance/angle checks must
                # NOT run for them (a heading turn has angle_rad=None, which the
                # non-last _check_segment_reached can't evaluate).
                is_opaque = seg.kind in ("follow_line", "spline", "diagonal") or (
                    seg.kind == "turn" and seg.opaque_step is not None
                )

                # Distance/angle completion (geometric segments only).
                if not transition and seg.has_known_endpoint and not is_opaque:
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
                        )

                # Opaque steps: adapter signals completion via is_finished().
                if not transition and is_opaque:
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
                        ephemeral_tasks,
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
                        ephemeral_tasks,
                    )
                    if node_idx >= len(nodes):
                        break

                    seg = nodes[node_idx]  # type: ignore[assignment]
                    seg = _with_heading_offset(seg, h0)
                    is_last = _is_last_segment(nodes, node_idx)

                    next_world_heading = await _world_heading_for_seg(robot, seg)
                    inflate = _next_segment_warmstarts(nodes, node_idx, seg)
                    if is_same_type(prev_seg, seg):
                        # Same type: warm start — carry velocity seamlessly.
                        offset = _get_position_offset(robot, prev_seg)
                        motion = create_motion(
                            robot,
                            seg,
                            is_last,
                            current_world_heading_rad=next_world_heading,
                            inflate=inflate,
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
                            inflate=inflate,
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
