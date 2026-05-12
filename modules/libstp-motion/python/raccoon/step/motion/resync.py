"""Minimal resync steps that inject observations into the localization filter."""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

from raccoon.foundation import ChassisVelocity, Pose
from raccoon.localization import Observation, SurfaceKind, SurfaceMeasurement

from .. import Step, dsl
from ..annotation import dsl_step
from .motion_step import MotionStep
from .wall_align import (
    WallAlignBackward,
    WallAlignForward,
    WallAlignStrafeLeft,
    WallAlignStrafeRight,
    WallDirection,
)

if TYPE_CHECKING:
    from raccoon.map import SensorOffset
    from raccoon.robot.api import GenericRobot
    from raccoon.sensor_ir import IRSensor


def _current_pose(robot: "GenericRobot") -> Pose:
    localization = getattr(robot, "localization", None)
    if localization is not None:
        return localization.get_pose()
    return Pose()


def _axis_sigma(enabled: bool, finite_sigma: float) -> float:
    return finite_sigma if enabled else math.inf


def _build_observation(
    robot: "GenericRobot",
    *,
    expected_x_cm: float | None,
    expected_y_cm: float | None,
    expected_theta_deg: float | None,
    snap_axes: tuple[bool, bool, bool],
    sigma_xy_cm: float,
    sigma_theta_deg: float,
    surface: SurfaceMeasurement | None = None,
) -> Observation:
    current = _current_pose(robot)
    pose = Pose()
    pose.position = current.position
    pose.heading = current.heading
    coords = [float(pose.position[0]), float(pose.position[1]), float(pose.position[2])]
    if expected_x_cm is not None:
        coords[0] = expected_x_cm / 100.0
    if expected_y_cm is not None:
        coords[1] = expected_y_cm / 100.0
    if expected_theta_deg is not None:
        pose.heading = math.radians(expected_theta_deg)
    pose.position = coords

    obs = Observation(
        pose=pose,
        sigma=(
            _axis_sigma(snap_axes[0], sigma_xy_cm / 100.0),
            _axis_sigma(snap_axes[1], sigma_xy_cm / 100.0),
            _axis_sigma(snap_axes[2], math.radians(sigma_theta_deg)),
        ),
    )
    if surface is not None:
        obs.surface_measurements = [surface]
    return obs


def _observe_if_available(robot: "GenericRobot", observation: Observation) -> bool:
    localization = getattr(robot, "localization", None)
    if localization is None:
        return False
    localization.observe(observation)
    return True


@dsl_step(tags=["localization", "resync"])
class ResyncAtStartPose(Step):
    """Inject a direct absolute pose observation without additional motion.

    Reads the robot's current odometry pose (or uses supplied override values)
    and feeds it as an absolute-pose observation into the localization filter.
    Use this at the start of a run to anchor the EKF to the known start pose.

    Args:
        expected_x_cm: Override for the expected X position in cm. If None,
            the robot's current odometric X is used.
        expected_y_cm: Override for the expected Y position in cm. If None,
            the robot's current odometric Y is used.
        expected_theta_deg: Override for the expected heading in degrees. If
            None, the robot's current odometric heading is used.
        snap_axes: Which axes (x, y, theta) to include in the observation.
            Defaults to (True, True, True) — all three.
        sigma_xy_cm: Position uncertainty (standard deviation) in cm.
        sigma_theta_deg: Heading uncertainty (standard deviation) in degrees.

    Example::

        from raccoon.step.motion import resync_at_start_pose

        resync_at_start_pose(expected_x_cm=0.0, expected_y_cm=0.0, expected_theta_deg=0.0)
    """

    def __init__(
        self,
        expected_x_cm: float | None = None,
        expected_y_cm: float | None = None,
        expected_theta_deg: float | None = None,
        snap_axes: tuple[bool, bool, bool] = (True, True, True),
        sigma_xy_cm: float = 1.0,
        sigma_theta_deg: float = 5.0,
    ) -> None:
        super().__init__()
        self.expected_x_cm = expected_x_cm
        self.expected_y_cm = expected_y_cm
        self.expected_theta_deg = expected_theta_deg
        self.snap_axes = snap_axes
        self.sigma_xy_cm = sigma_xy_cm
        self.sigma_theta_deg = sigma_theta_deg

    def _generate_signature(self) -> str:
        return "ResyncAtStartPose"

    async def _execute_step(self, robot: "GenericRobot") -> None:
        observation = _build_observation(
            robot,
            expected_x_cm=self.expected_x_cm,
            expected_y_cm=self.expected_y_cm,
            expected_theta_deg=self.expected_theta_deg,
            snap_axes=self.snap_axes,
            sigma_xy_cm=self.sigma_xy_cm,
            sigma_theta_deg=self.sigma_theta_deg,
        )
        _observe_if_available(robot, observation)


@dsl(hidden=True)
class FindLineResync(MotionStep):
    """Drive until an IR sensor sees a line, then inject a localization observation."""

    def __init__(
        self,
        sensor: "IRSensor",
        sensor_position: "SensorOffset",
        *,
        expected_x_cm: float | None = None,
        expected_y_cm: float | None = None,
        expected_theta_deg: float | None = None,
        snap_axes: tuple[bool, bool, bool] = (False, True, False),
        sigma_xy_cm: float = 1.0,
        sigma_theta_deg: float = 5.0,
        line_sigma_cm: float = 0.75,
        threshold: float = 0.7,
        forward_speed_mps: float = 0.15,
        strafe_speed_mps: float = 0.0,
        timeout_s: float = 3.0,
    ) -> None:
        super().__init__()
        self.sensor = sensor
        self.sensor_position = sensor_position
        self.expected_x_cm = expected_x_cm
        self.expected_y_cm = expected_y_cm
        self.expected_theta_deg = expected_theta_deg
        self.snap_axes = snap_axes
        self.sigma_xy_cm = sigma_xy_cm
        self.sigma_theta_deg = sigma_theta_deg
        self.line_sigma_cm = line_sigma_cm
        self.threshold = threshold
        self.forward_speed_mps = forward_speed_mps
        self.strafe_speed_mps = strafe_speed_mps
        self.timeout_s = timeout_s
        self._elapsed_s = 0.0
        self._observed = False

    def _generate_signature(self) -> str:
        return "FindLineResync"

    def on_start(self, robot: "GenericRobot") -> None:
        self._elapsed_s = 0.0
        self._observed = False
        robot.drive.set_velocity(
            ChassisVelocity(self.forward_speed_mps, self.strafe_speed_mps, 0.0)
        )

    def on_update(self, robot: "GenericRobot", dt: float) -> bool:
        robot.drive.update(dt)
        self._elapsed_s += dt

        if self.sensor.probabilityOfBlack() >= self.threshold:
            observation = _build_observation(
                robot,
                expected_x_cm=self.expected_x_cm,
                expected_y_cm=self.expected_y_cm,
                expected_theta_deg=self.expected_theta_deg,
                snap_axes=self.snap_axes,
                sigma_xy_cm=self.sigma_xy_cm,
                sigma_theta_deg=self.sigma_theta_deg,
                surface=SurfaceMeasurement(
                    SurfaceKind.LINE,
                    self.sensor_position,
                    detected=True,
                    sigma_cm=self.line_sigma_cm,
                ),
            )
            self._observed = _observe_if_available(robot, observation)
            return True

        return self._elapsed_s >= self.timeout_s


class AlignToWallResync(Step):
    """Run wall-align, then inject a wall-based observation into localization."""

    def __init__(
        self,
        direction: WallDirection = WallDirection.FORWARD,
        *,
        expected_x_cm: float | None = None,
        expected_y_cm: float | None = None,
        expected_theta_deg: float | None = None,
        snap_axes: tuple[bool, bool, bool] = (True, True, True),
        sigma_xy_cm: float = 1.0,
        sigma_theta_deg: float = 5.0,
        wall_sigma_cm: float = 0.75,
        sensor_position: "SensorOffset | None" = None,
        speed: float = 1.0,
        accel_threshold: float = 0.5,
        settle_duration: float = 0.2,
        max_duration: float = 5.0,
        grace_period: float = 0.3,
    ) -> None:
        super().__init__()
        self.direction = direction
        self.expected_x_cm = expected_x_cm
        self.expected_y_cm = expected_y_cm
        self.expected_theta_deg = expected_theta_deg
        self.snap_axes = snap_axes
        self.sigma_xy_cm = sigma_xy_cm
        self.sigma_theta_deg = sigma_theta_deg
        self.wall_sigma_cm = wall_sigma_cm
        self.sensor_position = sensor_position
        self._inner = {
            WallDirection.FORWARD: WallAlignForward,
            WallDirection.BACKWARD: WallAlignBackward,
            WallDirection.STRAFE_LEFT: WallAlignStrafeLeft,
            WallDirection.STRAFE_RIGHT: WallAlignStrafeRight,
        }[direction](
            speed=speed,
            accel_threshold=accel_threshold,
            settle_duration=settle_duration,
            max_duration=max_duration,
            grace_period=grace_period,
        )

    def _generate_signature(self) -> str:
        return f"AlignToWallResync(direction={self.direction.value})"

    def required_resources(self) -> frozenset[str]:
        return self._inner.required_resources()

    async def _execute_step(self, robot: "GenericRobot") -> None:
        await self._inner.run_step(robot)
        surface = None
        if self.sensor_position is not None:
            surface = SurfaceMeasurement(
                SurfaceKind.WALL,
                self.sensor_position,
                detected=True,
                sigma_cm=self.wall_sigma_cm,
            )
        observation = _build_observation(
            robot,
            expected_x_cm=self.expected_x_cm,
            expected_y_cm=self.expected_y_cm,
            expected_theta_deg=self.expected_theta_deg,
            snap_axes=self.snap_axes,
            sigma_xy_cm=self.sigma_xy_cm,
            sigma_theta_deg=self.sigma_theta_deg,
            surface=surface,
        )
        _observe_if_available(robot, observation)


@dsl(tags=["localization", "resync"])
def resync_at_start_pose(
    expected_x_cm: float | None = None,
    expected_y_cm: float | None = None,
    expected_theta_deg: float | None = None,
    *,
    snap_axes: tuple[bool, bool, bool] = (True, True, True),
    sigma_xy_cm: float = 1.0,
    sigma_theta_deg: float = 5.0,
) -> ResyncAtStartPose:
    return ResyncAtStartPose(
        expected_x_cm=expected_x_cm,
        expected_y_cm=expected_y_cm,
        expected_theta_deg=expected_theta_deg,
        snap_axes=snap_axes,
        sigma_xy_cm=sigma_xy_cm,
        sigma_theta_deg=sigma_theta_deg,
    )


@dsl(tags=["localization", "resync", "sensor"])
def find_line_resync(
    sensor: "IRSensor",
    sensor_position: "SensorOffset",
    *,
    expected_x_cm: float | None = None,
    expected_y_cm: float | None = None,
    expected_theta_deg: float | None = None,
    snap_axes: tuple[bool, bool, bool] = (False, True, False),
    sigma_xy_cm: float = 1.0,
    sigma_theta_deg: float = 5.0,
    line_sigma_cm: float = 0.75,
    threshold: float = 0.7,
    forward_speed_mps: float = 0.15,
    strafe_speed_mps: float = 0.0,
    timeout_s: float = 3.0,
) -> FindLineResync:
    return FindLineResync(
        sensor=sensor,
        sensor_position=sensor_position,
        expected_x_cm=expected_x_cm,
        expected_y_cm=expected_y_cm,
        expected_theta_deg=expected_theta_deg,
        snap_axes=snap_axes,
        sigma_xy_cm=sigma_xy_cm,
        sigma_theta_deg=sigma_theta_deg,
        line_sigma_cm=line_sigma_cm,
        threshold=threshold,
        forward_speed_mps=forward_speed_mps,
        strafe_speed_mps=strafe_speed_mps,
        timeout_s=timeout_s,
    )


@dsl(tags=["localization", "resync", "wall"])
def align_to_wall_resync(
    *,
    direction: WallDirection = WallDirection.FORWARD,
    expected_x_cm: float | None = None,
    expected_y_cm: float | None = None,
    expected_theta_deg: float | None = None,
    snap_axes: tuple[bool, bool, bool] = (True, True, True),
    sigma_xy_cm: float = 1.0,
    sigma_theta_deg: float = 5.0,
    wall_sigma_cm: float = 0.75,
    sensor_position: "SensorOffset | None" = None,
    speed: float = 1.0,
    accel_threshold: float = 0.5,
    settle_duration: float = 0.2,
    max_duration: float = 5.0,
    grace_period: float = 0.3,
) -> AlignToWallResync:
    return AlignToWallResync(
        direction=direction,
        expected_x_cm=expected_x_cm,
        expected_y_cm=expected_y_cm,
        expected_theta_deg=expected_theta_deg,
        snap_axes=snap_axes,
        sigma_xy_cm=sigma_xy_cm,
        sigma_theta_deg=sigma_theta_deg,
        wall_sigma_cm=wall_sigma_cm,
        sensor_position=sensor_position,
        speed=speed,
        accel_threshold=accel_threshold,
        settle_duration=settle_duration,
        max_duration=max_duration,
        grace_period=grace_period,
    )


__all__ = [
    "AlignToWallResync",
    "FindLineResync",
    "ResyncAtStartPose",
    "align_to_wall_resync",
    "find_line_resync",
    "resync_at_start_pose",
]
