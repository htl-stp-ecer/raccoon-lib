"""
Forward/backward lineup on lines using two IR sensors.

Drives until both sensors hit a line, measures the distance between hits,
then computes and executes a corrective turn angle.
"""
import math
import time
from libstp.foundation import ChassisVelocity
from libstp.sensor_ir import IRSensor
from libstp.step import Sequential, seq
from typing import TYPE_CHECKING

from ..turn import Turn, TurnConfig
from ..move_until import SurfaceColor, drive_until_black, drive_until_white
from ... import dsl
from ..motion_step import MotionStep

if TYPE_CHECKING:
    from libstp.robot.api import GenericRobot


@dsl(hidden=True)
class TimingBasedLineUp(MotionStep):
    def __init__(self, left_sensor: IRSensor, right_sensor: IRSensor,
                 target: SurfaceColor = SurfaceColor.BLACK,
                 forward_speed: float = 1.0,
                 detection_threshold: float = 0.9):
        super().__init__()
        self.left_sensor = left_sensor
        self.right_sensor = right_sensor
        self.target = target
        self.forward_speed = forward_speed
        self.threshold = detection_threshold
        self.distance_between_hits_m: float = 0.0
        self.results: tuple = (None, 0.0)
        self._left_triggered = False
        self._right_triggered = False
        self._t_first: float | None = None
        self._first_sensor: str | None = None
        self._first_hit_distance: float = 0.0

    def _get_confidences(self):
        if self.target == SurfaceColor.BLACK:
            return self.left_sensor.probabilityOfBlack(), self.right_sensor.probabilityOfBlack()
        else:
            return self.left_sensor.probabilityOfWhite(), self.right_sensor.probabilityOfWhite()

    def on_start(self, robot: "GenericRobot") -> None:
        self._left_triggered = False
        self._right_triggered = False
        self._t_first = None
        self._first_sensor = None
        self._first_hit_distance = 0.0
        robot.odometry.reset()
        robot.drive.set_velocity(ChassisVelocity(self.forward_speed, 0.0, 0.0))

    def on_update(self, robot: "GenericRobot", dt: float) -> bool:
        robot.odometry.update(dt)
        robot.drive.update(dt)

        left_conf, right_conf = self._get_confidences()
        now = time.monotonic()

        distance_info = robot.odometry.get_distance_from_origin()
        current_distance = distance_info.forward
        self.info(f"Left conf: {left_conf:.3f}, Right conf: {right_conf:.3f}, Distance: {robot.odometry.get_distance_from_origin()}")

        if not self._left_triggered and left_conf >= self.threshold:
            self._left_triggered = True
            if self._t_first is None:
                self._t_first = now
                self._first_sensor = "left"
                self._first_hit_distance = current_distance
                self.info("Left sensor hit line at t = 0.000s")
            else:
                elapsed = now - self._t_first
                self.distance_between_hits_m = abs(current_distance - self._first_hit_distance)
                self.info(f"Left sensor hit line at t = {elapsed:.3f}s, distance = {self.distance_between_hits_m * 100:.2f}cm")
                return True

        if not self._right_triggered and right_conf >= self.threshold:
            self._right_triggered = True
            if self._t_first is None:
                self._t_first = now
                self._first_sensor = "right"
                self._first_hit_distance = current_distance
                self.info("Right sensor hit line at t = 0.000s")
            else:
                elapsed = now - self._t_first
                self.distance_between_hits_m = abs(current_distance - self._first_hit_distance)
                self.info(f"Right sensor hit line at t = {elapsed:.3f}s, distance = {self.distance_between_hits_m * 100:.2f}cm")
                return True

        return self._left_triggered and self._right_triggered

    def on_stop(self, robot: "GenericRobot") -> None:
        robot.drive.hard_stop()
        self.info(f"Distance between sensor hits: {self.distance_between_hits_m * 100:.2f}cm")
        self.results = (self._first_sensor, self.distance_between_hits_m)


@dsl(hidden=True)
class ComputeTimingBasedAngle(Turn):

    def __init__(self, step: TimingBasedLineUp):
        config = TurnConfig()
        config.max_angular_rate = 1.0
        super().__init__(config)
        self.step = step

    async def _execute_step(self, robot: "GenericRobot") -> None:
        distance_between_sensors_m = robot.distance_between_sensors(
            self.step.left_sensor,
            self.step.right_sensor
        ) / 100
        self.info(f"Distance between sensors: {distance_between_sensors_m}m")
        distance_driven = self.step.results[1]
        self.config.target_angle_rad = math.atan(distance_driven / distance_between_sensors_m)
        if self.step.results[0] == "right":
            self.config.target_angle_rad = -self.config.target_angle_rad
        self.info(f"Computed turn angle: {math.degrees(self.config.target_angle_rad):.2f}\u00b0 sensor that hit first: {self.step.results[0]}")
        await super()._execute_step(robot)


@dsl(hidden=True)
def lineup(
        left_sensor: IRSensor,
        right_sensor: IRSensor,
        target: SurfaceColor = SurfaceColor.BLACK,
        detection_threshold: float = 0.7
) -> Sequential:
    step = TimingBasedLineUp(left_sensor, right_sensor, target, detection_threshold=detection_threshold)

    return seq([
        step,
        ComputeTimingBasedAngle(step),
    ])


@dsl(tags=["motion", "lineup"])
def forward_lineup_on_black(
        left_sensor: IRSensor,
        right_sensor: IRSensor,
        detection_threshold: float = 0.7
) -> Sequential:
    return seq([
        lineup(
            left_sensor=left_sensor,
            right_sensor=right_sensor,
            target=SurfaceColor.BLACK,
            detection_threshold=detection_threshold
        ),
        drive_until_white(
            [left_sensor, right_sensor],
            forward_speed=0.5
        )
    ])


@dsl(tags=["motion", "lineup"])
def forward_lineup_on_white(
        left_sensor: IRSensor,
        right_sensor: IRSensor,
        detection_threshold: float = 0.7
) -> Sequential:
    """
    Drive forward past black, then line up on white.

    Args:
        left_sensor: Left IR sensor instance
        right_sensor: Right IR sensor instance
        detection_threshold: Stop when confidence exceeds this

    Returns:
        Sequential step: drive_until_black -> lineup_on_white
    """
    return seq([
        lineup(
            left_sensor=left_sensor,
            right_sensor=right_sensor,
            target=SurfaceColor.WHITE,
            detection_threshold=detection_threshold
        ),
        drive_until_black(
            [left_sensor, right_sensor],
            forward_speed=0.5
        )
    ])


@dsl(tags=["motion", "lineup"])
def backward_lineup_on_black(
        left_sensor: IRSensor,
        right_sensor: IRSensor,
        detection_threshold: float = 0.7
) -> Sequential:
    """
    Drive backward past white, then line up on black.

    Args:
        left_sensor: Left IR sensor instance
        right_sensor: Right IR sensor instance
        detection_threshold: Stop when confidence exceeds this

    Returns:
        Sequential step: drive_until_white (backward) -> lineup_on_black (backward)
    """
    return seq([
        lineup(
            left_sensor=left_sensor,
            right_sensor=right_sensor,
            target=SurfaceColor.BLACK,
            detection_threshold=detection_threshold
        ),
        drive_until_white(
            [left_sensor, right_sensor],
            forward_speed=-0.5
        )
    ])


@dsl(tags=["motion", "lineup"])
def backward_lineup_on_white(
        left_sensor: IRSensor,
        right_sensor: IRSensor,
        detection_threshold: float = 0.7
) -> Sequential:
    """
    Drive backward past black, then line up on white.

    Args:
        left_sensor: Left IR sensor instance
        right_sensor: Right IR sensor instance
        detection_threshold: Stop when confidence exceeds this

    Returns:
        Sequential step: drive_until_black (backward) -> lineup_on_white (backward)
    """
    return seq([
        lineup(
            left_sensor=left_sensor,
            right_sensor=right_sensor,
            target=SurfaceColor.WHITE,
            detection_threshold=detection_threshold
        ),
        drive_until_black(
            [left_sensor, right_sensor],
            forward_speed=0.5
        )
    ])
