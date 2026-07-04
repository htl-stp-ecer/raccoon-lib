"""Unit tests for the continuous line-sensor → localization fusion service.

Uses fakes (no C++): a fake localization that records observe() calls, fake IR
sensors with settable detected state, and a fake robot exposing all_sensors().
Proves the service reads every line sensor each tick and pushes ONE Observation
carrying a SurfaceMeasurement per sensor with the live detected boolean, anchored
weakly on the filter's own current estimate.
"""

from __future__ import annotations

from dataclasses import dataclass

import pytest

# The service builds real Observation / SurfaceMeasurement objects, so it needs
# the compiled localization bindings. Skip cleanly when the wheel is absent.
pytest.importorskip("raccoon.localization")
from raccoon.robot.localization_fusion import ContinuousLocalizationFusion  # noqa: E402


@dataclass
class _Pose:
    position: tuple
    heading: float


class _FakeLoc:
    def __init__(self, x=1.0, y=2.0, h=0.3):
        self._pose = _Pose((x, y, 0.0), h)
        self.observed = []

    def get_pose(self):
        return self._pose

    def observe(self, obs):
        self.observed.append(obs)


class _FakeIR:
    """The service duck-types line sensors on ``probabilityOfBlack`` — no need to
    subclass the compiled IRSensor."""

    def __init__(self, p):
        self._p = p

    def probabilityOfBlack(self):
        return self._p


_make_ir = _FakeIR


@dataclass
class _Pos:
    forward_cm: float
    strafe_cm: float


class _FakeRobot:
    def __init__(self, sensors):
        self.localization = _FakeLoc()
        self._sensors = sensors

    def all_sensors(self):
        return self._sensors


def test_discovers_line_sensors():
    s1, s2 = _make_ir(0.9), _make_ir(0.1)
    robot = _FakeRobot({s1: _Pos(6.0, -4.0), s2: _Pos(6.0, 4.0)})
    fusion = ContinuousLocalizationFusion(robot)
    assert fusion.sensor_count == 2


def test_tick_pushes_one_observation_with_a_measurement_per_sensor():
    s_on, s_off = _make_ir(0.95), _make_ir(0.05)
    robot = _FakeRobot({s_on: _Pos(6.0, -4.0), s_off: _Pos(6.0, 4.0)})
    fusion = ContinuousLocalizationFusion(robot, threshold=0.7)

    assert fusion.tick() is True
    assert len(robot.localization.observed) == 1
    obs = robot.localization.observed[0]
    assert len(obs.surface_measurements) == 2
    detected = sorted(m.detected for m in obs.surface_measurements)
    assert detected == [False, True]  # one sensor over a line, one not


def test_anchor_pose_tracks_current_estimate_not_ground_truth():
    s = _make_ir(0.9)
    robot = _FakeRobot({s: _Pos(6.0, 0.0)})
    robot.localization = _FakeLoc(x=1.23, y=4.56, h=0.7)
    fusion = ContinuousLocalizationFusion(robot, anchor_sigma_cm=8.0)
    fusion.tick()
    obs = robot.localization.observed[0]
    # the pose anchor is the filter's OWN current estimate (no truth leak)
    assert obs.pose.position[0] == pytest.approx(1.23)
    assert obs.pose.position[1] == pytest.approx(4.56)
    assert obs.pose.heading == pytest.approx(0.7)
    # weak, finite anchor sigma (8 cm) on x/y
    assert obs.sigma[0] == pytest.approx(0.08)
    assert obs.sigma[1] == pytest.approx(0.08)


def test_no_sensors_is_noop():
    robot = _FakeRobot({})
    fusion = ContinuousLocalizationFusion(robot)
    assert fusion.sensor_count == 0
    assert fusion.tick() is False
    assert robot.localization.observed == []


def test_bad_sensor_read_is_skipped_not_fatal():
    class _Boom(_make_ir):
        def probabilityOfBlack(self):
            msg = "uncalibrated"
            raise RuntimeError(msg)

    good, bad = _make_ir(0.9), _Boom(0.0)
    robot = _FakeRobot({good: _Pos(6.0, 0.0), bad: _Pos(6.0, 4.0)})
    fusion = ContinuousLocalizationFusion(robot)
    assert fusion.tick() is True
    obs = robot.localization.observed[0]
    # only the good sensor produced a measurement
    assert len(obs.surface_measurements) == 1
    assert obs.surface_measurements[0].detected is True
