"""
Strafe lineup on lines using two IR sensors (for omni bots).

Strafes until both sensors hit a line, measures the distance between hits,
then computes and executes a corrective turn angle.
"""
import math
import time
from libstp.foundation import ChassisVelocity
from libstp.sensor_ir import IRSensor
from libstp.step import Sequential, seq
from typing import TYPE_CHECKING

from ..turn import Turn, TurnConfig
from ..move_until import SurfaceColor, strafe_until_black, strafe_until_white
from ... import dsl
from ..motion_step import MotionStep

if TYPE_CHECKING:
    from libstp.robot.api import GenericRobot


@dsl(hidden=True)
class TimingBasedStrafeLineUp(MotionStep):
    def __init__(self, front_sensor: IRSensor, back_sensor: IRSensor,
                 target: SurfaceColor = SurfaceColor.BLACK,
                 strafe_speed: float = 1.0,
                 detection_threshold: float = 0.9):
        super().__init__()
        self.front_sensor = front_sensor
        self.back_sensor = back_sensor
        self.target = target
        self.strafe_speed = strafe_speed
        self.threshold = detection_threshold
        self.distance_between_hits_m: float = 0.0
        self.results: tuple = (None, 0.0)
        self._front_triggered = False
        self._back_triggered = False
        self._t_first: float | None = None
        self._first_sensor: str | None = None
        self._first_hit_distance: float = 0.0

    def _get_confidences(self):
        if self.target == SurfaceColor.BLACK:
            return self.front_sensor.probabilityOfBlack(), self.back_sensor.probabilityOfBlack()
        else:
            return self.front_sensor.probabilityOfWhite(), self.back_sensor.probabilityOfWhite()

    def on_start(self, robot: "GenericRobot") -> None:
        self._front_triggered = False
        self._back_triggered = False
        self._t_first = None
        self._first_sensor = None
        self._first_hit_distance = 0.0
        robot.odometry.reset()
        robot.drive.set_velocity(ChassisVelocity(0.0, self.strafe_speed, 0.0))

    def on_update(self, robot: "GenericRobot", dt: float) -> bool:
        robot.odometry.update(dt)
        robot.drive.update(dt)

        front_conf, back_conf = self._get_confidences()
        now = time.monotonic()

        distance_info = robot.odometry.get_distance_from_origin()
        current_distance = distance_info.lateral
        self.info(f"Front conf: {front_conf:.3f}, Back conf: {back_conf:.3f}, Distance: {robot.odometry.get_distance_from_origin()}")

        if not self._front_triggered and front_conf >= self.threshold:
            self._front_triggered = True
            if self._t_first is None:
                self._t_first = now
                self._first_sensor = "front"
                self._first_hit_distance = current_distance
                self.info("Front sensor hit line at t = 0.000s")
            else:
                elapsed = now - self._t_first
                self.distance_between_hits_m = abs(current_distance - self._first_hit_distance)
                self.info(f"Front sensor hit line at t = {elapsed:.3f}s, distance = {self.distance_between_hits_m * 100:.2f}cm")
                return True

        if not self._back_triggered and back_conf >= self.threshold:
            self._back_triggered = True
            if self._t_first is None:
                self._t_first = now
                self._first_sensor = "back"
                self._first_hit_distance = current_distance
                self.info("Back sensor hit line at t = 0.000s")
            else:
                elapsed = now - self._t_first
                self.distance_between_hits_m = abs(current_distance - self._first_hit_distance)
                self.info(f"Back sensor hit line at t = {elapsed:.3f}s, distance = {self.distance_between_hits_m * 100:.2f}cm")
                return True

        return self._front_triggered and self._back_triggered

    def on_stop(self, robot: "GenericRobot") -> None:
        robot.drive.hard_stop()
        self.info(f"Distance between sensor hits: {self.distance_between_hits_m * 100:.2f}cm")
        self.results = (self._first_sensor, self.distance_between_hits_m)


@dsl(hidden=True)
class ComputeTimingBasedStrafeAngle(Turn):

    def __init__(self, step: TimingBasedStrafeLineUp):
        config = TurnConfig()
        config.max_angular_rate = 1.0
        super().__init__(config)
        self.step = step

    async def _execute_step(self, robot: "GenericRobot") -> None:
        distance_between_sensors_m = robot.distance_between_sensors(
            self.step.front_sensor,
            self.step.back_sensor
        ) / 100
        self.info(f"Distance between sensors: {distance_between_sensors_m}m")
        distance_strafed = self.step.results[1]
        self.config.target_angle_rad = math.atan(distance_strafed / distance_between_sensors_m)
        # When strafing right (+), front first -> CCW (+); back first -> CW (-)
        # When strafing left (-), the correction direction flips
        if self.step.results[0] == "back":
            self.config.target_angle_rad = -self.config.target_angle_rad
        if self.step.strafe_speed < 0:
            self.config.target_angle_rad = -self.config.target_angle_rad
        self.info(f"Computed turn angle: {math.degrees(self.config.target_angle_rad):.2f} sensor that hit first: {self.step.results[0]}")
        await super()._execute_step(robot)


@dsl(hidden=True)
def strafe_lineup(
        front_sensor: IRSensor,
        back_sensor: IRSensor,
        target: SurfaceColor = SurfaceColor.BLACK,
        strafe_speed: float = 1.0,
        detection_threshold: float = 0.7
) -> Sequential:
    step = TimingBasedStrafeLineUp(front_sensor, back_sensor, target,
                                   strafe_speed=strafe_speed,
                                   detection_threshold=detection_threshold)

    return seq([
        step,
        ComputeTimingBasedStrafeAngle(step),
    ])


@dsl(tags=["motion", "lineup"])
def strafe_right_lineup_on_black(
        front_sensor: IRSensor,
        back_sensor: IRSensor,
        detection_threshold: float = 0.7
) -> Sequential:
    return seq([
        strafe_lineup(
            front_sensor=front_sensor,
            back_sensor=back_sensor,
            target=SurfaceColor.BLACK,
            strafe_speed=1.0,
            detection_threshold=detection_threshold
        ),
        strafe_until_white(
            [front_sensor, back_sensor],
            strafe_speed=0.3
        )
    ])


@dsl(tags=["motion", "lineup"])
def strafe_right_lineup_on_white(
        front_sensor: IRSensor,
        back_sensor: IRSensor,
        detection_threshold: float = 0.7
) -> Sequential:
    return seq([
        strafe_lineup(
            front_sensor=front_sensor,
            back_sensor=back_sensor,
            target=SurfaceColor.WHITE,
            strafe_speed=1.0,
            detection_threshold=detection_threshold
        ),
        strafe_until_black(
            [front_sensor, back_sensor],
            strafe_speed=0.3
        )
    ])


@dsl(tags=["motion", "lineup"])
def strafe_left_lineup_on_black(
        front_sensor: IRSensor,
        back_sensor: IRSensor,
        detection_threshold: float = 0.7
) -> Sequential:
    return seq([
        strafe_lineup(
            front_sensor=front_sensor,
            back_sensor=back_sensor,
            target=SurfaceColor.BLACK,
            strafe_speed=-1.0,
            detection_threshold=detection_threshold
        ),
        strafe_until_white(
            [front_sensor, back_sensor],
            strafe_speed=-0.3
        )
    ])


@dsl(tags=["motion", "lineup"])
def strafe_left_lineup_on_white(
        front_sensor: IRSensor,
        back_sensor: IRSensor,
        detection_threshold: float = 0.7
) -> Sequential:
    return seq([
        strafe_lineup(
            front_sensor=front_sensor,
            back_sensor=back_sensor,
            target=SurfaceColor.WHITE,
            strafe_speed=-1.0,
            detection_threshold=detection_threshold
        ),
        strafe_until_black(
            [front_sensor, back_sensor],
            strafe_speed=-0.3
        )
    ])
