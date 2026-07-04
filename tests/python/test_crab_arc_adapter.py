"""Runtime behaviour of the ``CrabArcAdapter`` (constant-heading corner blend).

Drives the adapter against a minimal holonomic fake robot (body-frame velocity
integrated into a world pose) and checks that it:

* sweeps the body velocity from the entry leg's direction into the exit leg's,
* holds the heading roughly fixed (no rotation), and
* completes after ~one quarter-circle of travel (``R · π/2``).
"""

from __future__ import annotations

import importlib.util
import math

import pytest


def _has_raccoon() -> bool:
    return importlib.util.find_spec("raccoon") is not None


pytestmark = pytest.mark.skipif(not _has_raccoon(), reason="raccoon not importable")


class _Pose:
    def __init__(self, x, y, heading):
        self.position = [x, y]
        self.heading = heading


class _Odometry:
    """Integrates the drive's last body-frame command into a world pose."""

    def __init__(self, drive):
        self._drive = drive
        self.x = 0.0
        self.y = 0.0
        self.heading = 0.0

    def update(self, dt):
        vx, vy, wz = self._drive.cmd
        # Body → world rotation by current heading (vy is +right).
        c, s = math.cos(self.heading), math.sin(self.heading)
        self.x += (vx * c - vy * s) * dt
        self.y += (vx * s + vy * c) * dt
        self.heading += wz * dt

    def get_pose(self):
        return _Pose(self.x, self.y, self.heading)

    def get_heading(self):
        return self.heading


class _Axis:
    def __init__(self, v):
        self.max_velocity = v


class _PidConfig:
    def __init__(self):
        self.linear = _Axis(0.2)
        self.lateral = _Axis(0.2)
        self.angular = _Axis(3.0)


class _Drive:
    def __init__(self):
        self.cmd = (0.0, 0.0, 0.0)
        self.history = []

    def set_velocity(self, v):
        self.cmd = (v.vx, v.vy, v.wz)
        self.history.append(self.cmd)

    def update(self, dt):
        pass

    def hard_stop(self):
        self.cmd = (0.0, 0.0, 0.0)


class _Robot:
    def __init__(self):
        self.drive = _Drive()
        self.odometry = _Odometry(self.drive)
        self.motion_pid_config = _PidConfig()


def _crab_seg(crab_from, crab_to, radius_m=0.1):
    from raccoon.step.motion.path.ir import Segment

    return Segment(
        kind="crab_arc",
        radius_m=radius_m,
        arc_angle_rad=math.pi / 2,
        speed_scale=1.0,
        crab_from=crab_from,
        crab_to=crab_to,
    )


def _run(adapter, robot, dt=0.01, max_cycles=2000):
    cycles = 0
    while not adapter.is_finished() and cycles < max_cycles:
        adapter.update(dt)
        cycles += 1
    return cycles


def test_crab_arc_completes_after_quarter_circle():
    from raccoon.step.motion.path.motion_factory import CrabArcAdapter

    robot = _Robot()
    seg = _crab_seg((1.0, 0.0), (0.0, -1.0), radius_m=0.1)  # forward → strafe left
    adapter = CrabArcAdapter(seg, robot)
    adapter.start()
    cycles = _run(adapter, robot)

    assert adapter.is_finished()
    assert cycles < 2000
    # Travelled length ≈ R·π/2 (one quarter circle), within a cycle's slack.
    arc_len = 0.1 * math.pi / 2
    assert adapter._traveled == pytest.approx(arc_len, abs=0.01)


def test_crab_arc_blends_velocity_from_entry_to_exit():
    from raccoon.step.motion.path.motion_factory import CrabArcAdapter

    robot = _Robot()
    seg = _crab_seg((1.0, 0.0), (0.0, -1.0))  # forward (+vx) → strafe left (-vy)
    adapter = CrabArcAdapter(seg, robot)
    adapter.start()
    _run(adapter, robot)

    # Drop the trailing zero-velocity stop command(s).
    moving = [(vx, vy, wz) for (vx, vy, wz) in robot.drive.history if (vx, vy) != (0.0, 0.0)]
    first, last = moving[0], moving[-1]
    # Starts almost pure forward (+vx, ~0 vy).
    assert first[0] > 0.18 and abs(first[1]) < 0.05
    # Ends almost pure strafe-left (~0 vx, -vy).
    assert abs(last[0]) < 0.05 and last[1] < -0.18


def test_crab_arc_holds_heading():
    from raccoon.step.motion.path.motion_factory import CrabArcAdapter

    robot = _Robot()
    robot.odometry.heading = 0.5  # start at an arbitrary held heading
    seg = _crab_seg((0.0, 1.0), (1.0, 0.0))  # strafe right → forward
    adapter = CrabArcAdapter(seg, robot)
    adapter.start()
    _run(adapter, robot)

    # No commanded rotation disturbed the held heading by more than a hair.
    assert robot.odometry.heading == pytest.approx(0.5, abs=0.02)


def test_crab_arc_speed_capped_by_slower_axis():
    from raccoon.step.motion.path.motion_factory import CrabArcAdapter

    robot = _Robot()
    robot.motion_pid_config.lateral = _Axis(0.1)  # lateral slower than forward
    seg = _crab_seg((1.0, 0.0), (0.0, 1.0))
    adapter = CrabArcAdapter(seg, robot)
    adapter.start()
    assert adapter._speed == pytest.approx(0.1, abs=1e-9)


def test_crab_arc_warm_start_uses_carried_velocity():
    from raccoon.step.motion.path.motion_factory import CrabArcAdapter

    robot = _Robot()
    seg = _crab_seg((1.0, 0.0), (0.0, 1.0))
    adapter = CrabArcAdapter(seg, robot)
    adapter.start_warm(offset=0.0, velocity=0.05)
    assert adapter._speed == pytest.approx(0.05, abs=1e-9)
