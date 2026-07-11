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

from raccoon.class_name_logger import ClassNameLogger
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

DISTANCE_TOL_M = 0.002  # 2mm — in lockstep with C++ distance_tolerance_m
ANGLE_TOL_RAD = 0.035  # ~2° tolerance for manual angle check

# Stall watchdog for geometric (linear/turn/arc) path segments. A short tactical
# leg (e.g. a 5 cm strafe) that makes NO progress toward its target for this long
# is stuck — a mis-compiled distance/sign, a blocked robot, or odometry that never
# crosses the (possibly wrong) target. Without this guard the segment drives blind
# for tens of seconds (the M050 "30 s freeze": a 5 cm strafe ran ~30 s because its
# odometry completion check never fired). Bail loudly instead.
_SEGMENT_STALL_TIMEOUT_S = 4.0
# 2 mm — meaningful linear progress toward target. Matches the completion band
# (DISTANCE_TOL_M) so the watchdog counts the tighter final approach as progress
# and doesn't trip while the profiled PID is legitimately closing the last few mm.
_PROGRESS_EPS_M = 0.002
_PROGRESS_EPS_RAD = math.radians(1.0)  # ~1° — meaningful angular progress


def _segment_target_and_eps(seg: Segment) -> tuple[float, float] | None:
    """Return ``(target, progress_eps)`` for a geometric segment, else ``None``.

    Opaque segments (follow_line/spline/diagonal/crab_arc, opaque turns) and
    open-ended segments (``target is None``) have no measurable distance/angle
    target and are skipped by the trace/watchdog logic.
    """
    if seg.kind == "linear":
        target, eps = seg.distance_m, _PROGRESS_EPS_M
    elif seg.kind == "turn":
        target, eps = seg.angle_rad, _PROGRESS_EPS_RAD
    elif seg.kind == "arc":
        target, eps = seg.arc_angle_rad, _PROGRESS_EPS_RAD
    else:
        return None
    if target is None:
        return None
    return target, eps


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


def _non_last_reached(
    robot: "GenericRobot",
    seg: Segment,
    seg_origin: float,
    motion,
) -> bool:
    """Completion decision for a NON-terminal geometric segment.

    Two independent completion paths, whichever fires first:

    1. Manual position check (:func:`_check_segment_reached`, 5 mm band) — an
       INFLATED segment cruises through its endpoint and never self-completes,
       so the executor triggers the transition at the real target.
    2. The motion's OWN completion (:func:`_has_reached_target`) — a NON-inflated
       segment (the next leg cold-starts) decelerates to its target exactly like
       the last segment and self-completes at the C++ distance tolerance (1 cm),
       then hard-stops. Its profile has ramped to zero; we MUST accept that
       completion here, otherwise the robot sits parked one tolerance-gap short
       of the tighter manual band, making no progress until the stall watchdog
       trips (the strafe-reversal stall in the 2026-07-01 hardware log).

    For an inflated segment path (2) is inert — the inflated +1 m target is never
    reached and the motion never finishes — so warm-start cruise behaviour is
    unchanged and only path (1) drives the transition.
    """
    return _check_segment_reached(robot, seg, seg_origin) or _has_reached_target(motion, seg)


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

    EPHEMERAL parallel() branches are joined LAZILY, not at every transition:
    only right before a blocking, NON-motion inline side action (a servo / wait
    that might need a resource the branch holds), and at path end (see
    ``PathExecutor.run``'s ``finally``). Two transitions must NOT join, or they
    break motion:

    * A bare segment→segment warm continuation (merged same-type legs carry
      velocity with no hard stop). Blocking the join there freezes the control
      loop while the motor keeps its last velocity command, so the robot coasts
      open-loop for the whole branch duration and overshoots the leg (a 0.3 s
      servo branch ran tens of cm past a merged drive). Letting the loop keep
      ticking keeps the motion closed-loop.
    * An inline side action that is itself a COLLAPSED MOTION spine — under
      ``to_absolute()`` / ``splinify()`` a ``parallel(spine, branch)`` lowers to
      ``[branch(eph), GotoWaypoints/SplineFollow(inline)]``; the branch must run
      CONCURRENTLY with that move (that is the whole point of parallel()), so we
      detect it via the "drive" resource and skip the pre-join.

    In every case the branch still runs concurrently and is joined before the
    next blocking non-motion step or at path end, preserving await-all.
    """

    async def _join_ephemeral() -> None:
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
            # A non-motion inline step waits in place, so honour parallel()'s
            # await-all by joining first. A collapsed-motion inline (drive
            # resource) instead runs concurrently with the branch — don't join.
            if "drive" not in node.step.collected_resources():
                await _join_ephemeral()
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
        # Next Segment: warm only if same type AND not a direction reversal.
        # A reversal cold-starts (see _should_warm), so this leg must NOT
        # inflate/cruise through its endpoint — it has to decelerate to its
        # real target so the reversal point is clean.
        return is_same_type(seg, node) and not _reverses_direction(seg, node)
    return False  # no further segment


# ---------------------------------------------------------------------------
# Time-optimal velocity profile consumption
#
# When optimize(...).time_optimal() ran, every motion Segment carries a
# VelocityProfilePass-stamped entry_speed_mps / exit_speed_mps (else None). The
# executor then drives warm-start / inflate decisions from that GLOBAL profile
# instead of the local same-type heuristic — carrying speed across the seams the
# profile proved safe (including linear↔arc tangents) and stopping where it
# proved a barrier. Strictly gated on the fields being set: an unprofiled path
# (the common case) takes the original branches untouched.
# ---------------------------------------------------------------------------

_VEL_EPS = 1e-3


def _is_profiled(seg) -> bool:
    return isinstance(seg, Segment) and seg.entry_speed_mps is not None


def _carries_out(seg) -> bool:
    """Profiled segment exits carrying speed (next leg warm-starts / inflate)."""
    return (seg.exit_speed_mps or 0.0) > _VEL_EPS


def _carries_in(seg) -> bool:
    """Profiled segment is entered warm (carries speed in)."""
    return (seg.entry_speed_mps or 0.0) > _VEL_EPS


def _warm_lookahead(nodes, idx, seg) -> bool:
    """Will the NEXT leg warm-start from ``seg`` / should ``seg`` inflate?
    Profile says so when stamped, else fall back to the local heuristic."""
    if _is_profiled(seg):
        return _carries_out(seg)
    return _next_segment_warmstarts(nodes, idx, seg)


def _warm_velocity(seg, fallback_mps: float) -> float:
    """Seed velocity for ``seg.start_warm`` — the profile's prescribed entry
    speed (converted to angular for arcs), else the measured carry velocity."""
    if _is_profiled(seg) and _carries_in(seg):
        v = seg.entry_speed_mps or 0.0
        if seg.kind == "arc" and seg.radius_m:
            return v / abs(seg.radius_m)
        return v
    return fallback_mps


def _reverses_direction(prev_seg, seg) -> bool:
    """Do two consecutive same-axis linear legs travel in opposite directions?

    A ``Lateral -5cm`` immediately followed by ``Lateral +6cm`` is same-type
    (so ``is_same_type`` says "warm"), but carrying the previous leg's velocity
    into the reversal sends the robot the WRONG way first — it must decelerate,
    reverse, and only then chase the new target. Observed on hardware: the +6cm
    strafe flew to -5.5cm before turning around, wasting ~1.2s and never
    settling. Opposite signs on the same axis ⇒ cold-start (hard stop between)
    so the new profile begins cleanly at zero velocity.
    """
    if prev_seg.kind != "linear" or seg.kind != "linear":
        return False
    pd, cd = prev_seg.distance_m, seg.distance_m
    if pd is None or cd is None:
        return False
    return pd * cd < 0.0


def _should_warm(prev_seg, seg) -> bool:
    """Warm-start into ``seg``? Profile decides when stamped, else same-type.

    Even for same-type legs, never warm-start across a direction reversal on
    the same axis — carrying velocity into the opposite direction overshoots
    the wrong way first (see :func:`_reverses_direction`).
    """
    if _is_profiled(seg):
        return _carries_in(seg)
    if _reverses_direction(prev_seg, seg):
        return False
    return is_same_type(prev_seg, seg)


# ---------------------------------------------------------------------------
# Physically-realised stop before a cold (opposite-direction) start
# ---------------------------------------------------------------------------

# Conservative body deceleration used to size the settle window. It only needs
# to be a lower bound on the real decel: too small just settles a touch longer,
# never too short. Bounds keep a same-direction cold seam snappy while capping
# the worst case for a full-speed reversal.
_SETTLE_DECEL_MPS2 = 1.2
_MIN_SETTLE_S = 0.05
_MAX_SETTLE_S = 0.5


def _settle_time_s(entry_vel: float) -> float:
    """How long to actively brake before a cold start, given the speed we carry.

    Zero when we're already essentially at rest (barrier legs ramp their own
    profile to zero, so the cold seam has nothing to bleed off). Otherwise the
    time to decelerate ``|entry_vel|`` to zero at :data:`_SETTLE_DECEL_MPS2`,
    clamped into ``[_MIN_SETTLE_S, _MAX_SETTLE_S]``.
    """
    speed = abs(entry_vel)
    if speed < 1e-3:
        return 0.0
    return min(_MAX_SETTLE_S, max(_MIN_SETTLE_S, speed / _SETTLE_DECEL_MPS2))


async def _settle_to_rest(robot: "GenericRobot", hz: float, entry_vel: float) -> None:
    """Bleed off residual velocity to a *physical* standstill before a cold start.

    ``drive.hard_stop()`` only COMMANDS zero body velocity and returns while the
    robot is still coasting on its momentum. Starting the next
    (opposite-direction) segment immediately re-arms the motors mid-coast, so the
    hard stop the reversal logic depends on never physically happens — the robot
    flies past, exactly the overshoot the reversal guard was meant to prevent.

    Hold the zero command and pump the drive controllers for long enough to
    actually come to rest (window scaled by the entry speed we're shedding), so
    the new profile genuinely warm-starts from zero — the Istzustand the cold
    start assumes. Re-asserting ``hard_stop()`` every tick also beats the
    mode-vs-velocity re-arm race that otherwise lets a stale command win.
    """
    settle_s = _settle_time_s(entry_vel)
    if settle_s <= 0.0:
        return
    loop = asyncio.get_event_loop()
    update_rate = 1.0 / hz
    deadline = loop.time() + settle_s
    last_time = loop.time() - update_rate  # seed first dt
    while loop.time() < deadline:
        now = loop.time()
        dt = max(now - last_time, 0.0)
        last_time = now
        if dt < 1e-4:
            await asyncio.sleep(update_rate)
            continue
        robot.drive.hard_stop()  # re-assert zero every tick (beats the re-arm race)
        robot.drive.update(dt)  # keep the host-side brake controllers alive
        await asyncio.sleep(update_rate)


# ---------------------------------------------------------------------------
# Segment logging
# ---------------------------------------------------------------------------


def _describe_segment(seg: Segment) -> str:
    """One-line, human-readable description of a motion segment for logging.

    The optimizer/compiler lowers the user's drive/turn/strafe steps into bare
    ``Segment`` IR that the executor runs directly via ``create_motion`` —
    bypassing ``Step.run_step``, which is where ordinary steps print their
    signature. So none of the motion legs in an ``optimize()`` / ``smooth_path``
    block logged anything. This rebuilds a step-like signature from the IR so the
    executor can announce each leg as it starts.

    Opaque segments (follow_line / spline / diagonal / heading-turn) still carry
    their original step, so we reuse that step's own signature verbatim.
    """
    if seg.opaque_step is not None:
        try:
            return seg.opaque_step._generate_signature()
        except Exception:  # logging must never break the run
            return f"{seg.kind}()"

    suffix = " until <condition>" if seg.condition is not None else ""

    if seg.kind == "linear":
        axis = seg.axis.name if seg.axis is not None else "?"
        if seg.distance_m is not None:
            body = f"{axis} {seg.distance_m * 100.0:+.1f}cm"
        else:
            body = f"{axis} (sensor)"
        return f"Drive({body}){suffix}"
    if seg.kind == "turn":
        if seg.angle_rad is not None:
            return f"Turn({math.degrees(seg.angle_rad):+.1f}°){suffix}"
        if seg.target_heading_rad is not None:
            return f"Turn(to {math.degrees(seg.target_heading_rad):.1f}°){suffix}"
        return f"Turn(sensor){suffix}"
    if seg.kind == "arc":
        r = (seg.radius_m or 0.0) * 100.0
        a = math.degrees(seg.arc_angle_rad or 0.0)
        return f"Arc(R={r:.1f}cm, {a:+.1f}°){suffix}"
    if seg.kind == "crab_arc":
        r = (seg.radius_m or 0.0) * 100.0
        return f"CrabArc(R={r:.1f}cm){suffix}"
    return f"{seg.kind}(){suffix}"


# ---------------------------------------------------------------------------
# Executor
# ---------------------------------------------------------------------------


class PathExecutor(ClassNameLogger):
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

        # Set once the path runs to its natural end (vs. unwinding on an
        # exception). Gates the ephemeral join in ``finally`` so a teardown on
        # error still cancels instead of awaiting a possibly-stuck branch.
        completed = False

        # Running count of motion legs started, for the per-segment log line.
        seg_count = 0

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
                completed = True
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

            seg_count += 1
            self.info(f"#{seg_count} {_describe_segment(seg)}")

            motion = create_motion(
                robot,
                seg,
                is_last,
                current_world_heading_rad=await _world_heading_for_seg(robot, seg),
                inflate=_warm_lookahead(nodes, node_idx, seg),
            )
            motion.start()
            seg_origin = _get_position_offset(robot, seg)

            if seg.condition is not None:
                seg.condition.start(robot)

            # 4. Unified control loop.
            update_rate = 1.0 / self._hz
            loop = asyncio.get_event_loop()
            last_time = loop.time() - update_rate  # seed first dt

            # Per-segment stall watchdog state. Re-armed (by object identity)
            # whenever the active segment changes inside the loop.
            watch_seg: Segment | None = None
            watch_best_remaining = math.inf
            watch_last_progress_t = last_time

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
                is_opaque = seg.kind in ("follow_line", "spline", "diagonal", "crab_arc") or (
                    seg.kind == "turn" and seg.opaque_step is not None
                )

                # Per-tick TRACE telemetry + stall watchdog for geometric
                # segments. Opaque adapters (follow_line/spline/…) log their own
                # per-tick state and self-terminate via is_finished(), so they're
                # excluded here. A geometric leg that logs nothing per tick is the
                # blind spot that hid the M050 30 s strafe hang — make it visible
                # and bail if it stops progressing toward its target.
                if not transition and seg.has_known_endpoint and not is_opaque:
                    te = _segment_target_and_eps(seg)
                    if te is not None:
                        target, eps = te
                        traveled = _get_position_offset(robot, seg) - seg_origin
                        remaining = abs(target - traveled)
                        self.trace(
                            f"#{seg_count} {seg.kind} "
                            f"axis={getattr(seg, 'axis', None)} "
                            f"traveled={traveled:.4f} target={target:.4f} "
                            f"remaining={remaining:.4f} dt={dt:.4f}"
                        )
                        if seg is not watch_seg:
                            watch_seg = seg
                            watch_best_remaining = remaining
                            watch_last_progress_t = current_time
                        elif remaining < watch_best_remaining - eps:
                            watch_best_remaining = remaining
                            watch_last_progress_t = current_time
                        elif current_time - watch_last_progress_t > _SEGMENT_STALL_TIMEOUT_S:
                            robot.drive.hard_stop()
                            msg = (
                                f"path segment #{seg_count} "
                                f"({_describe_segment(seg)}) made no progress "
                                f"toward its target for "
                                f"{_SEGMENT_STALL_TIMEOUT_S:.1f}s "
                                f"(traveled={traveled:.4f}, target={target:.4f}, "
                                f"remaining={remaining:.4f}) — skipping stuck "
                                f"segment and continuing the path. Likely a "
                                f"mis-compiled distance/sign or a blocked robot."
                            )
                            self.error(msg)
                            # Don't crash the mission: treat the stall as if the
                            # segment completed so the control loop advances to the
                            # next node instead of raising out of the run.
                            transition = True

                # Distance/angle completion (geometric segments only).
                if not transition and seg.has_known_endpoint and not is_opaque:
                    if is_last:
                        # Profile decelerates naturally to the real target.
                        transition = _has_reached_target(motion, seg)
                    else:
                        # Manual band OR the motion's own completion — see
                        # _non_last_reached (fixes the ramp-to-zero-then-stuck
                        # strafe-reversal stall).
                        transition = _non_last_reached(robot, seg, seg_origin, motion)

                # Opaque steps: adapter signals completion via is_finished().
                if not transition and is_opaque:
                    transition = motion.is_finished()

                if transition:
                    current_vel = motion.get_filtered_velocity()
                    prev_seg = seg

                    seg_idx = node_idx
                    node_idx += 1

                    # Stop NOW if the next segment won't warm-start from this
                    # one — otherwise the robot keeps the just-finished
                    # segment's velocity command and COASTS while the blocking
                    # inline side actions at this transition run (await'd),
                    # drifting past the endpoint. (A sensor-bounded strafe that
                    # fires on a line then runs arm moves coasted ~10 cm past the
                    # line; a short distance leg's decel tail wasn't clamped.)
                    # Warm continuations intentionally keep moving.
                    if not _warm_lookahead(nodes, seg_idx, seg):
                        robot.drive.hard_stop()

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

                    seg_count += 1
                    self.info(f"#{seg_count} {_describe_segment(seg)}")

                    next_world_heading = await _world_heading_for_seg(robot, seg)
                    inflate = _warm_lookahead(nodes, node_idx, seg)
                    if _should_warm(prev_seg, seg):
                        # Warm start — carry velocity seamlessly. The offset is
                        # read in the NEW segment's frame for a profiled
                        # cross-type carry (e.g. linear↔arc), else the prev
                        # segment's (same-type, identical reading).
                        warm_ref = seg if _is_profiled(seg) else prev_seg
                        offset = _get_position_offset(robot, warm_ref)
                        motion = create_motion(
                            robot,
                            seg,
                            is_last,
                            current_world_heading_rad=next_world_heading,
                            inflate=inflate,
                        )
                        motion.start_warm(offset, _warm_velocity(seg, current_vel))
                    else:
                        # Cross-type / direction reversal: cold start. hard_stop()
                        # only COMMANDS zero — actively bleed off the residual
                        # velocity to a real standstill first, so the new profile
                        # warm-starts from genuine rest (Istzustand) instead of
                        # re-arming the motors mid-coast and flying past. This is
                        # the physical realisation of the reversal hard stop.
                        await _settle_to_rest(robot, self._hz, current_vel)
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

            # Loop exits only via the end-of-path breaks above — natural end.
            completed = True
        finally:
            robot.drive.hard_stop()
            # On NORMAL completion, join any still-pending EPHEMERAL
            # parallel-branch tasks before tearing down: parallel()'s await-all
            # semantics must hold even when the path ends without a following
            # segment transition to join at — e.g. to_absolute()/splinify()
            # collapsed the motion into inline side actions (no real Segment
            # left to transition through), or the parallel sat at the path tail.
            # Without this the branch is launched and then cancelled below
            # before it can finish — the bug where parallel side actions on an
            # optimized path silently didn't run. On exception teardown
            # (completed is False) we skip the join and cancel everything.
            if completed and ephemeral_tasks:
                await asyncio.gather(*ephemeral_tasks, return_exceptions=True)
                ephemeral_tasks.clear()
            # Plain (non-ephemeral) background() tasks stay fire-and-forget:
            # cancel whatever is still running. Ephemeral tasks joined just
            # above are already done here, so cancelling is a no-op for them.
            for task in bg_tasks:
                if not task.done():
                    task.cancel()
            if bg_tasks:
                await asyncio.gather(*bg_tasks, return_exceptions=True)
