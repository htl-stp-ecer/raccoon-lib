from __future__ import annotations

import asyncio
import math
from types import SimpleNamespace

import pytest

from raccoon.foundation import Pose
from raccoon.localization import SurfaceKind
from raccoon.map import SensorOffset
from raccoon.step.motion.resync import (
    AlignToWallResync,
    FindLineResync,
    ResyncAtStartPose,
)
from raccoon.step.motion.wall_align import WallDirection


class _FakeLocalization:
    def __init__(self) -> None:
        self.calls = []

    def get_pose(self):
        return Pose()

    def observe(self, observation) -> None:
        self.calls.append(observation)


class _FakeDrive:
    def __init__(self) -> None:
        self.commands = []
        self.updated = []

    def set_velocity(self, velocity) -> None:
        self.commands.append(velocity)

    def update(self, dt: float) -> None:
        self.updated.append(dt)

    def hard_stop(self) -> None:
        self.commands.append(("stop",))


class _FakeSensor:
    def __init__(self, readings: list[float]) -> None:
        self._readings = iter(readings)

    def probabilityOfBlack(self) -> float:
        return next(self._readings, 1.0)


class _FakeWallAlignStep:
    def required_resources(self) -> frozenset[str]:
        return frozenset({"drive"})

    async def run_step(self, robot) -> None:
        robot.wall_align_ran = True


def test_resync_at_start_pose_observes_immediately() -> None:
    robot = SimpleNamespace(localization=_FakeLocalization())
    step = ResyncAtStartPose(expected_x_cm=120, expected_y_cm=35, expected_theta_deg=90)

    asyncio.run(step._execute_step(robot))

    assert len(robot.localization.calls) == 1
    obs = robot.localization.calls[0]
    assert obs.pose.position[0] == pytest.approx(1.2)
    assert obs.pose.position[1] == pytest.approx(0.35)
    assert obs.pose.heading == pytest.approx(math.pi / 2)


def test_find_line_resync_observes_with_line_measurement() -> None:
    robot = SimpleNamespace(localization=_FakeLocalization(), drive=_FakeDrive())
    sensor = _FakeSensor([0.2, 0.4, 0.85])
    step = FindLineResync(
        sensor=sensor,
        sensor_position=SensorOffset(forward_cm=6.0, strafe_cm=1.5),
        expected_y_cm=50.0,
        threshold=0.7,
        timeout_s=2.0,
        line_sigma_cm=0.5,
    )

    step.on_start(robot)
    assert step.on_update(robot, 0.1) is False
    assert step.on_update(robot, 0.1) is False
    assert step.on_update(robot, 0.1) is True

    assert len(robot.localization.calls) == 1
    obs = robot.localization.calls[0]
    assert math.isinf(obs.sigma[0])
    assert obs.sigma[1] == pytest.approx(0.01)
    assert math.isinf(obs.sigma[2])
    assert len(obs.surface_measurements) == 1
    measurement = obs.surface_measurements[0]
    assert tuple(measurement.sensor) == pytest.approx((6.0, 1.5))
    assert measurement.detected is True
    assert measurement.sigma_cm == pytest.approx(0.5)


def test_align_to_wall_resync_runs_inner_step_and_observes() -> None:
    robot = SimpleNamespace(localization=_FakeLocalization(), wall_align_ran=False)
    step = AlignToWallResync(
        direction=WallDirection.STRAFE_LEFT,
        expected_x_cm=80.0,
        expected_theta_deg=180.0,
        snap_axes=(True, False, True),
        sensor_position=SensorOffset(forward_cm=7.5, strafe_cm=0.0),
        wall_sigma_cm=0.4,
    )
    step._inner = _FakeWallAlignStep()

    asyncio.run(step._execute_step(robot))

    assert robot.wall_align_ran is True
    assert len(robot.localization.calls) == 1
    obs = robot.localization.calls[0]
    assert obs.pose.position[0] == pytest.approx(0.8)
    assert math.isinf(obs.sigma[1])
    assert obs.pose.heading == pytest.approx(math.pi)
    assert len(obs.surface_measurements) == 1
    assert obs.surface_measurements[0].kind == SurfaceKind.WALL
