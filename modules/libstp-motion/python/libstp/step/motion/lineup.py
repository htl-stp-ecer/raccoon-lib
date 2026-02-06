"""
Line up on black or white lines using two IR sensors.

This module provides steps for aligning the robot on lines using differential
wheel control based on sensor feedback.
"""
import asyncio
import time
from dataclasses import dataclass
from libstp.foundation import ChassisVelocity, PIDController
from libstp.log import info
from libstp.sensor_ir import IRSensor
from libstp.step import Sequential, seq
from typing import TYPE_CHECKING
from .turn import Turn, TurnConfig

from .drive_until import SurfaceColor, drive_until_black, drive_until_white
from .. import Step, SimulationStep, SimulationStepDelta, dsl
import math

if TYPE_CHECKING:
    from libstp.robot.api import GenericRobot


@dsl(hidden=True)
class TimingBasedLineUp(Step):
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
        self.distance_between_hits_m: float = 0.0  # Distance traveled between sensor hits
        self.result = (None, 0.0)  # (first_sensor: str, distance_between_hits_m: float)

    def _get_confidences(self):
        if self.target == SurfaceColor.BLACK:
            return self.left_sensor.probabilityOfBlack(), self.right_sensor.probabilityOfBlack()
        else:
            return self.left_sensor.probabilityOfWhite(), self.right_sensor.probabilityOfWhite()

    async def _execute_step(self, robot: "GenericRobot") -> None:
        left_triggered = False
        right_triggered = False
        t_first = None
        first_sensor = None
        first_hit_distance: float = 0.0

        # Reset odometry to track distance from start
        robot.odometry.reset()

        robot.drive.set_velocity(ChassisVelocity(self.forward_speed, 0.0, 0.0))

        update_rate = 1 / 100  # 100 Hz
        last_time = asyncio.get_event_loop().time() - update_rate

        while not (left_triggered and right_triggered):
            current_time = asyncio.get_event_loop().time()
            delta_time = max(current_time - last_time, 0.0)
            last_time = current_time

            robot.odometry.update(delta_time)
            robot.drive.update(delta_time)

            left_conf, right_conf = self._get_confidences()
            now = time.monotonic()

            # Use get_distance_from_origin() - avoids Pose.position crash
            distance_info = robot.odometry.get_distance_from_origin()
            current_distance = distance_info.forward
            self.info(f"Left conf: {left_conf:.3f}, Right conf: {right_conf:.3f}, Distance: {robot.odometry.get_distance_from_origin()}")

            if not left_triggered and left_conf >= self.threshold:
                left_triggered = True
                if t_first is None:
                    t_first = now
                    first_sensor = "left"
                    first_hit_distance = current_distance
                    self.info("Left sensor hit line at t = 0.000s")
                else:
                    dt = now - t_first
                    self.distance_between_hits_m = abs(current_distance - first_hit_distance)
                    self.info(f"Left sensor hit line at t = {dt:.3f}s, distance = {self.distance_between_hits_m * 100:.2f}cm")
                    robot.drive.hard_stop()
                    break

            if not right_triggered and right_conf >= self.threshold:
                right_triggered = True
                if t_first is None:
                    t_first = now
                    first_sensor = "right"
                    first_hit_distance = current_distance
                    self.info("Right sensor hit line at t = 0.000s")
                else:
                    dt = now - t_first
                    self.distance_between_hits_m = abs(current_distance - first_hit_distance)
                    self.info(f"Right sensor hit line at t = {dt:.3f}s, distance = {self.distance_between_hits_m * 100:.2f}cm")
                    robot.drive.hard_stop()
                    break

            await asyncio.sleep(update_rate)

        robot.drive.hard_stop()
        self.info(f"Distance between sensor hits: {self.distance_between_hits_m * 100:.2f}cm")
        self.results = (first_sensor, self.distance_between_hits_m)

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
        distance_driven = self.step.results[1] * 1.025 # Todo: Configurable?
        self.config.target_angle_rad = math.atan(distance_driven / distance_between_sensors_m)
        if self.step.results[0] == "right":
            self.config.target_angle_rad = -self.config.target_angle_rad
        self.info(f"Computed turn angle: {math.degrees(self.config.target_angle_rad):.2f}° sensor that hit first: {self.step.results[0]}")
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

    First drives until one sensor sees black (on the line), then
    performs the lineup on white.

    Args:
        left_sensor: Left IR sensor instance
        right_sensor: Right IR sensor instance
        base_speed: Forward speed in m/s (must be positive)
        confidence_threshold: Stop when confidence exceeds this

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
        base_speed: Backward speed in m/s (will be negated)
        confidence_threshold: Stop when confidence exceeds this

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
        base_speed: Backward speed in m/s (will be negated)
        confidence_threshold: Stop when confidence exceeds this

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
