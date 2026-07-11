"""Standalone motion steps ramp from the robot's measured Ist-velocity.

A standalone ``drive``/``turn``/``arc`` step used to cold-start its profile at
zero velocity. When the robot is already moving at ``on_start`` the trapezoid
must ramp from the *measured* velocity instead (a leg entering at -0.22 m/s runs
its ramp from -0.22, not 0). Below a small deadband the robot is treated as at
rest and the motion cold-starts, so the common sequential-steps case (previous
leg ramped to zero) is byte-for-byte unchanged.

Pure-Python unit tests over :mod:`raccoon.step.motion._warm_start` — no C++/sim.
"""

from __future__ import annotations

import importlib.util
from types import SimpleNamespace

import pytest


def _libstp_available() -> bool:
    return importlib.util.find_spec("raccoon") is not None


requires_libstp = pytest.mark.skipif(
    not _libstp_available(),
    reason="raccoon native module not installed",
)


@pytest.fixture
def _mod():
    from raccoon.motion import LinearAxis
    from raccoon.step.motion import _warm_start as ws

    return ws, LinearAxis


class _FakeDrive:
    def __init__(self, vx=0.0, vy=0.0, wz=0.0):
        self._state = SimpleNamespace(vx=vx, vy=vy, wz=wz)

    def estimate_state(self):
        return self._state


class _FakeRobot:
    def __init__(self, vx=0.0, vy=0.0, wz=0.0):
        self.drive = _FakeDrive(vx, vy, wz)


class _FakeMotion:
    """Records which start path (cold vs warm) the helper took."""

    def __init__(self):
        self.cold = 0
        self.warm_args = None

    def start(self):
        self.cold += 1

    def start_warm(self, offset, velocity):
        self.warm_args = (offset, velocity)


# ---------------------------------------------------------------------------
# Measured-velocity component selection
# ---------------------------------------------------------------------------


@requires_libstp
def test_measured_linear_picks_axis_component(_mod):
    ws, LinearAxis = _mod
    robot = _FakeRobot(vx=0.3, vy=-0.7)
    assert ws.measured_linear_velocity(robot, LinearAxis.Forward) == 0.3
    assert ws.measured_linear_velocity(robot, LinearAxis.Lateral) == -0.7


@requires_libstp
def test_measured_angular_returns_wz(_mod):
    ws, _LinearAxis = _mod
    robot = _FakeRobot(wz=1.4)
    assert ws.measured_angular_velocity(robot) == 1.4


# ---------------------------------------------------------------------------
# Warm vs cold decision (linear)
# ---------------------------------------------------------------------------


@requires_libstp
def test_linear_warm_starts_when_moving(_mod):
    ws, LinearAxis = _mod
    robot = _FakeRobot(vx=-0.22)  # the -0.22 -> +... reversal case
    motion = _FakeMotion()
    ws.warm_start_linear(motion, robot, LinearAxis.Forward)
    assert motion.cold == 0
    assert motion.warm_args == (0.0, -0.22)  # ramps from the measured Ist velocity


@requires_libstp
def test_linear_cold_starts_at_rest(_mod):
    ws, LinearAxis = _mod
    # Below the deadband → identical to the pre-warm-start cold path.
    robot = _FakeRobot(vx=ws.WARM_START_MIN_MPS / 2.0)
    motion = _FakeMotion()
    ws.warm_start_linear(motion, robot, LinearAxis.Forward)
    assert motion.cold == 1
    assert motion.warm_args is None


@requires_libstp
def test_linear_deadband_is_axis_specific(_mod):
    ws, LinearAxis = _mod
    # Moving laterally but the step is a Forward drive → Forward sees ~0 → cold.
    robot = _FakeRobot(vx=0.0, vy=0.9)
    motion = _FakeMotion()
    ws.warm_start_linear(motion, robot, LinearAxis.Forward)
    assert motion.cold == 1


# ---------------------------------------------------------------------------
# Warm vs cold decision (angular)
# ---------------------------------------------------------------------------


@requires_libstp
def test_angular_warm_starts_when_turning(_mod):
    ws, _LinearAxis = _mod
    robot = _FakeRobot(wz=-0.8)
    motion = _FakeMotion()
    ws.warm_start_angular(motion, robot)
    assert motion.cold == 0
    assert motion.warm_args == (0.0, -0.8)


@requires_libstp
def test_angular_cold_starts_at_rest(_mod):
    ws, _LinearAxis = _mod
    robot = _FakeRobot(wz=ws.WARM_START_MIN_RADPS / 2.0)
    motion = _FakeMotion()
    ws.warm_start_angular(motion, robot)
    assert motion.cold == 1
    assert motion.warm_args is None
