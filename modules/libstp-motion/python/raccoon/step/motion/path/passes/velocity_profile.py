"""Time-optimal velocity profiling pass.

Generalises the executor's local warm-start (carry velocity only across the
single adjacent same-type seam) into a GLOBAL forward/backward sweep over the
whole compiled path. It computes, for every motion segment, the fastest
feasible ENTRY and EXIT speed (m/s) such that the robot:

* starts and ends at rest,
* never exceeds a segment's cruise cap (``max_speed * speed_scale``) or, for
  arcs, the curvature cap ``v <= sqrt(lat_accel * R)``,
* can always accelerate up to / decelerate down to the next constraint within a
  segment's length (the two-sweep feasibility), and
* drops to a full stop at every REAL barrier — a turn (rotation in place), a
  direction reversal, a blocking inline side action, a deferred node, a
  sensor-bounded ``.until()`` leg's end, and the path's last segment.

The speeds are stamped onto ``Segment.entry_speed_mps`` / ``exit_speed_mps``.
The pass is PATH-PRESERVING — it changes only speed, never geometry — and is
opt-in via ``optimize(steps).time_optimal()``. With the pass off the fields stay
``None`` and the executor's behaviour is byte-identical to before.
"""

from __future__ import annotations

import dataclasses
import math

from ..ir import PathNode, Segment, SideAction
from .lowering import is_same_type

# Conservative drivetrain defaults (m/s, m/s²). max_speed is deliberately high
# so it never caps BELOW a segment's real cruise — the profile then only ADDS
# carry across seams that previously cold-stopped, it never slows a leg the
# unprofiled executor already ran at full speed.
DEFAULT_MAX_SPEED_MPS = 1.0
DEFAULT_ACCEL_MPS2 = 0.6
DEFAULT_LATERAL_ACCEL_MPS2 = 0.5

_EPS = 1e-6
_TRANSLATIONAL = {"linear", "follow_line", "arc"}


def _dir_sign(seg: Segment) -> float:
    """Travel direction sign of a translational segment (+1 fwd / -1 back)."""
    if seg.distance_m is not None and abs(seg.distance_m) > _EPS:
        return math.copysign(1.0, seg.distance_m)
    return math.copysign(1.0, seg.sign or 1.0)


def _arc_tangent(a: Segment, b: Segment) -> bool:
    """An arc abuts its neighbouring straight/arc tangentially by construction
    (cut_corners emits linear+arc+linear tangent), so speed may carry across."""
    return ("arc" in (a.kind, b.kind)
            and a.kind in _TRANSLATIONAL and b.kind in _TRANSLATIONAL)


def _can_carry(a: Segment, b: Segment) -> bool:
    """May the robot carry speed from segment ``a`` into ``b`` without changing
    the path? Only across a continuous, same-direction translational seam."""
    if a.kind not in _TRANSLATIONAL or b.kind not in _TRANSLATIONAL:
        return False  # a turn / spline / diagonal forces a stop
    if a.condition is not None:
        return False  # a stops when its sensor/time condition fires
    if a.kind == "linear" and b.kind == "linear" and _dir_sign(a) != _dir_sign(b):
        return False  # direction reversal
    return is_same_type(a, b) or _arc_tangent(a, b)


def _length_m(seg: Segment) -> float | None:
    """Translational length of a segment, or None when unknown (condition-only —
    treated as 'long enough' for the accel reach, since it stops at a barrier)."""
    if seg.kind == "arc" and seg.radius_m is not None and seg.arc_angle_rad is not None:
        return abs(seg.radius_m * seg.arc_angle_rad)
    if seg.distance_m is not None:
        return abs(seg.distance_m)
    return None  # sentinel / condition-bounded


def _v_max(seg: Segment, max_speed: float, lat_accel: float) -> float:
    if seg.kind == "turn":
        return 0.0  # rotation in place — no translational speed
    base = max_speed * (seg.speed_scale or 1.0)
    if seg.kind == "arc" and seg.radius_m:
        return min(base, math.sqrt(max(0.0, lat_accel * abs(seg.radius_m))))
    return base


def run_velocity_profile(
    nodes: list[PathNode | None],
    *,
    max_speed_mps: float = DEFAULT_MAX_SPEED_MPS,
    accel_mps2: float = DEFAULT_ACCEL_MPS2,
    lateral_accel_mps2: float = DEFAULT_LATERAL_ACCEL_MPS2,
) -> list[PathNode | None]:
    """Stamp feasible entry/exit speeds onto every motion Segment via a global
    two-sweep. Returns a NEW node list (segments replaced); non-motion nodes are
    preserved in place."""
    # Indices of motion segments, and whether a BARRIER sits before each (a
    # blocking inline side action, or a deferred None) — a barrier forces a stop.
    seg_idx: list[int] = []
    barrier_before: list[bool] = []
    pending_barrier = False
    for i, node in enumerate(nodes):
        if isinstance(node, Segment):
            seg_idx.append(i)
            barrier_before.append(pending_barrier)
            pending_barrier = False
        elif node is None:
            pending_barrier = True  # unresolved Defer — be conservative
        elif isinstance(node, SideAction):
            if not node.is_background:
                pending_barrier = True  # blocking inline side action → stop

    n = len(seg_idx)
    if n == 0:
        return list(nodes)

    segs = [nodes[i] for i in seg_idx]
    vmax = [_v_max(s, max_speed_mps, lateral_accel_mps2) for s in segs]
    length = [_length_m(s) for s in segs]

    # vlink[i] = speed cap on the seam between seg i and i+1 (0 = full stop).
    vlink = [0.0] * max(0, n - 1)
    for i in range(n - 1):
        if barrier_before[i + 1]:
            vlink[i] = 0.0
        elif _can_carry(segs[i], segs[i + 1]):
            vlink[i] = min(vmax[i], vmax[i + 1])
        else:
            vlink[i] = 0.0

    entry = [0.0] * n
    exit_ = [0.0] * n
    for i in range(n):
        exit_[i] = vmax[i]
        entry[i] = vmax[i]
    entry[0] = 0.0
    exit_[n - 1] = 0.0
    for i in range(n - 1):
        exit_[i] = min(exit_[i], vlink[i])
        entry[i + 1] = min(entry[i + 1], vlink[i])

    def reach(v0: float, dist: float | None) -> float:
        if dist is None:
            return math.inf  # condition-bounded — no accel-limited cap
        return math.sqrt(max(0.0, v0 * v0 + 2.0 * accel_mps2 * dist))

    # Forward sweep: exit bounded by accel from entry.
    for i in range(n):
        exit_[i] = min(exit_[i], reach(entry[i], length[i]))
        if i + 1 < n:
            entry[i + 1] = min(entry[i + 1], exit_[i])
    # Backward sweep: entry bounded by decel to exit.
    for i in range(n - 1, -1, -1):
        entry[i] = min(entry[i], reach(exit_[i], length[i]))
        if i - 1 >= 0:
            exit_[i - 1] = min(exit_[i - 1], entry[i])

    out = list(nodes)
    for k, i in enumerate(seg_idx):
        out[i] = dataclasses.replace(
            segs[k], entry_speed_mps=entry[k], exit_speed_mps=exit_[k]
        )
    return out


class VelocityProfilePass:
    """Compiler pass: stamp a global time-optimal velocity profile (entry/exit
    speeds) onto the path. Path-preserving; opt-in via ``.time_optimal()``."""

    name = "time_optimal"

    def __init__(
        self,
        max_speed_mps: float = DEFAULT_MAX_SPEED_MPS,
        accel_mps2: float = DEFAULT_ACCEL_MPS2,
        lateral_accel_mps2: float = DEFAULT_LATERAL_ACCEL_MPS2,
    ) -> None:
        self.max_speed_mps = max_speed_mps
        self.accel_mps2 = accel_mps2
        self.lateral_accel_mps2 = lateral_accel_mps2

    def run(self, nodes: list[PathNode | None]) -> list[PathNode | None]:
        return run_velocity_profile(
            nodes,
            max_speed_mps=self.max_speed_mps,
            accel_mps2=self.accel_mps2,
            lateral_accel_mps2=self.lateral_accel_mps2,
        )
