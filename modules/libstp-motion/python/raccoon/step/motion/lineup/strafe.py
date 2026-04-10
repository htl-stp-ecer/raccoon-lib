"""
Strafe lineup on lines using two IR sensors (for omni bots).

Strafes until both sensors hit a line, measures the distance between hits,
then computes and executes a corrective turn angle.
"""
import math
import time
from raccoon.foundation import ChassisVelocity, info
from raccoon.sensor_ir import IRSensor
from raccoon.step import Sequential, seq, defer, Run
from raccoon.step.condition import on_black, on_white
from typing import TYPE_CHECKING

from ..turn_dsl import turn_left, turn_right
from ..drive_dsl import strafe_left, strafe_right
from .forward import SurfaceColor
from ... import dsl
from ..motion_step import MotionStep

if TYPE_CHECKING:
    from raccoon.robot.api import GenericRobot


@dsl(hidden=True)
class TimingBasedStrafeLineUp(MotionStep):
    """Strafe sideways until both front and back IR sensors detect a line.

    The robot strafes at ``strafe_speed`` (positive = right, negative = left)
    while polling both sensors each update cycle.  When the first sensor
    crosses the target-colored line its lateral odometry position is recorded.
    Strafing continues until the second sensor also crosses.  The lateral
    distance traveled between the two hits is stored in ``results`` as
    ``(first_sensor_name, distance_m)`` so a downstream step can compute the
    corrective turn angle.

    This is an internal building-block step -- use the public
    ``strafe_right_lineup_on_black`` family of functions instead.

    Prerequisites:
        A mecanum or omni-wheel drivetrain capable of lateral strafing.

    Args:
        front_sensor: IR sensor mounted on the front of the chassis.
        back_sensor: IR sensor mounted on the rear of the chassis.
        target: The surface color to detect (BLACK or WHITE).
        strafe_speed: Lateral speed in m/s.  Positive = strafe right,
            negative = strafe left.
        detection_threshold: Confidence value (0--1) a sensor must reach to
            count as having detected the target color.
    """

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
        self.debug(f"Front conf: {front_conf:.3f}, Back conf: {back_conf:.3f}, Distance: {robot.odometry.get_distance_from_origin()}")

        if not self._front_triggered and front_conf >= self.threshold:
            self._front_triggered = True
            if self._t_first is None:
                self._t_first = now
                self._first_sensor = "front"
                self._first_hit_distance = current_distance
                self.debug("Front sensor hit line at t = 0.000s")
            else:
                elapsed = now - self._t_first
                self.distance_between_hits_m = abs(current_distance - self._first_hit_distance)
                self.debug(f"Front sensor hit line at t = {elapsed:.3f}s, distance = {self.distance_between_hits_m * 100:.2f}cm")
                return True

        if not self._back_triggered and back_conf >= self.threshold:
            self._back_triggered = True
            if self._t_first is None:
                self._t_first = now
                self._first_sensor = "back"
                self._first_hit_distance = current_distance
                self.debug("Back sensor hit line at t = 0.000s")
            else:
                elapsed = now - self._t_first
                self.distance_between_hits_m = abs(current_distance - self._first_hit_distance)
                self.debug(f"Back sensor hit line at t = {elapsed:.3f}s, distance = {self.distance_between_hits_m * 100:.2f}cm")
                return True

        return self._front_triggered and self._back_triggered

    def on_stop(self, robot: "GenericRobot") -> None:
        robot.drive.hard_stop()
        self.debug(f"Distance between sensor hits: {self.distance_between_hits_m * 100:.2f}cm")
        self.results = (self._first_sensor, self.distance_between_hits_m)


def _compute_strafe_lineup_turn(measure: TimingBasedStrafeLineUp, robot: "GenericRobot"):
    sensor_gap_m = robot.distance_between_sensors(
        measure.front_sensor, measure.back_sensor
    ) / 100
    first_sensor, distance_strafed = measure.results
    angle_rad = math.atan(distance_strafed / sensor_gap_m)
    # When strafing right (+), front first -> CCW (+); back first -> CW (-)
    # When strafing left (-), the correction direction flips
    if first_sensor == "back":
        angle_rad = -angle_rad
    if measure.strafe_speed < 0:
        angle_rad = -angle_rad
    degrees = math.degrees(abs(angle_rad))

    if abs(degrees) < 0.1:
        info(f"Already at target heading (error={degrees:.3f}°) — skipping turn")
        return Run(lambda _robot: None)

    return turn_left(degrees) if angle_rad >= 0 else turn_right(degrees)


@dsl(hidden=True)
def strafe_lineup(
        front_sensor: IRSensor,
        back_sensor: IRSensor,
        target: SurfaceColor = SurfaceColor.BLACK,
        strafe_speed: float = 1.0,
        detection_threshold: float = 0.7
) -> Sequential:
    """Measure angular skew from a line via lateral strafing, then correct with a turn.

    Composes a ``TimingBasedStrafeLineUp`` measurement phase with a deferred
    corrective turn.  During the measurement the robot strafes until both
    sensors cross the target line.  The lateral distance between the two hit
    positions and the known physical gap between the sensors are used to compute
    a corrective turn angle via ``atan(distance_strafed / sensor_gap)``.  The
    turn is then executed to align the robot perpendicular to the line.

    This is an internal helper -- prefer the public
    ``strafe_right_lineup_on_black`` family of functions.

    Prerequisites:
        A mecanum or omni-wheel drivetrain capable of lateral strafing.

    Args:
        front_sensor: IR sensor on the front of the chassis.
        back_sensor: IR sensor on the rear of the chassis.
        target: Surface color to detect (BLACK or WHITE).
        strafe_speed: Lateral speed in m/s (positive = right, negative = left).
        detection_threshold: Confidence (0--1) required to register a hit.

    Returns:
        Sequential: A two-step sequence (measure + corrective turn).
    """
    measure = TimingBasedStrafeLineUp(front_sensor, back_sensor, target,
                                       strafe_speed=strafe_speed,
                                       detection_threshold=detection_threshold)

    return seq([
        measure,
        defer(lambda robot: _compute_strafe_lineup_turn(measure, robot)),
    ])


@dsl(tags=["motion", "lineup"])
def strafe_right_lineup_on_black(
        front_sensor: IRSensor,
        back_sensor: IRSensor,
        detection_threshold: float = 0.7
) -> Sequential:
    """Strafe right onto a black line, align perpendicular, then clear to white.

    The robot strafes right until both front and back IR sensors detect a black
    line.  The lateral stagger distance between the two sensor hits is used to
    compute a corrective turn that aligns the chassis perpendicular to the
    line.  After the turn the robot strafes right at reduced speed until both
    sensors see white, leaving it just past the line and squared up.

    Prerequisites:
        Two IR line sensors mounted on the front and rear of the chassis, with
        the robot's ``distance_between_sensors`` configured.  Requires a
        mecanum or omni-wheel drivetrain capable of lateral strafing.

    Args:
        front_sensor: IR sensor on the front of the chassis.
        back_sensor: IR sensor on the rear of the chassis.
        detection_threshold: Confidence value (0--1) each sensor must reach to
            count as detecting a line.  Lower values trigger earlier but are
            more susceptible to noise.

    Returns:
        Sequential: measure + corrective turn + strafe-until-white.

    Example::

        from raccoon.step.motion.lineup.strafe import strafe_right_lineup_on_black

        step = strafe_right_lineup_on_black(
            front_sensor=robot.front_line_sensor,
            back_sensor=robot.back_line_sensor,
            detection_threshold=0.8,
        )
        step.run(robot)
    """
    return seq([
        strafe_lineup(
            front_sensor=front_sensor,
            back_sensor=back_sensor,
            target=SurfaceColor.BLACK,
            strafe_speed=1.0,
            detection_threshold=detection_threshold
        ),
        strafe_right(speed=0.3).until(
            on_white(front_sensor) | on_white(back_sensor)
        ),
    ])


@dsl(tags=["motion", "lineup"])
def strafe_right_lineup_on_white(
        front_sensor: IRSensor,
        back_sensor: IRSensor,
        detection_threshold: float = 0.7
) -> Sequential:
    """Strafe right onto a white line, align perpendicular, then clear to black.

    The robot strafes right until both front and back IR sensors detect a white
    surface.  The lateral stagger distance between the two sensor hits is used
    to compute a corrective turn that aligns the chassis perpendicular to the
    line edge.  After the turn the robot strafes right at reduced speed until
    both sensors see black, leaving it just past the white region and squared
    up.

    Prerequisites:
        Two IR line sensors mounted on the front and rear of the chassis, with
        the robot's ``distance_between_sensors`` configured.  Requires a
        mecanum or omni-wheel drivetrain capable of lateral strafing.

    Args:
        front_sensor: IR sensor on the front of the chassis.
        back_sensor: IR sensor on the rear of the chassis.
        detection_threshold: Confidence value (0--1) each sensor must reach to
            count as detecting a white surface.

    Returns:
        Sequential: measure + corrective turn + strafe-until-black.

    Example::

        from raccoon.step.motion.lineup.strafe import strafe_right_lineup_on_white

        step = strafe_right_lineup_on_white(
            front_sensor=robot.front_line_sensor,
            back_sensor=robot.back_line_sensor,
        )
        step.run(robot)
    """
    return seq([
        strafe_lineup(
            front_sensor=front_sensor,
            back_sensor=back_sensor,
            target=SurfaceColor.WHITE,
            strafe_speed=1.0,
            detection_threshold=detection_threshold
        ),
        strafe_right(speed=0.3).until(
            on_black(front_sensor) | on_black(back_sensor)
        ),
    ])


@dsl(tags=["motion", "lineup"])
def strafe_left_lineup_on_black(
        front_sensor: IRSensor,
        back_sensor: IRSensor,
        detection_threshold: float = 0.7
) -> Sequential:
    """Strafe left onto a black line, align perpendicular, then clear to white.

    Identical to ``strafe_right_lineup_on_black`` but the robot strafes left
    instead of right.  The corrective turn direction is automatically mirrored
    to account for the reversed lateral geometry.  After alignment the robot
    continues strafing left at reduced speed until both sensors see white.

    Prerequisites:
        Two IR line sensors mounted on the front and rear of the chassis, with
        the robot's ``distance_between_sensors`` configured.  Requires a
        mecanum or omni-wheel drivetrain capable of lateral strafing.

    Args:
        front_sensor: IR sensor on the front of the chassis.
        back_sensor: IR sensor on the rear of the chassis.
        detection_threshold: Confidence value (0--1) each sensor must reach to
            count as detecting a line.

    Returns:
        Sequential: measure + corrective turn + strafe-until-white (leftward).

    Example::

        from raccoon.step.motion.lineup.strafe import strafe_left_lineup_on_black

        step = strafe_left_lineup_on_black(
            front_sensor=robot.front_line_sensor,
            back_sensor=robot.back_line_sensor,
        )
        step.run(robot)
    """
    return seq([
        strafe_lineup(
            front_sensor=front_sensor,
            back_sensor=back_sensor,
            target=SurfaceColor.BLACK,
            strafe_speed=-1.0,
            detection_threshold=detection_threshold
        ),
        strafe_left(speed=0.3).until(
            on_white(front_sensor) | on_white(back_sensor)
        ),
    ])


@dsl(tags=["motion", "lineup"])
def strafe_left_lineup_on_white(
        front_sensor: IRSensor,
        back_sensor: IRSensor,
        detection_threshold: float = 0.7
) -> Sequential:
    """Strafe left onto a white line, align perpendicular, then clear to black.

    Identical to ``strafe_right_lineup_on_white`` but the robot strafes left
    instead of right.  The corrective turn direction is automatically mirrored
    to account for the reversed lateral geometry.  After alignment the robot
    continues strafing left at reduced speed until both sensors see black.

    Prerequisites:
        Two IR line sensors mounted on the front and rear of the chassis, with
        the robot's ``distance_between_sensors`` configured.  Requires a
        mecanum or omni-wheel drivetrain capable of lateral strafing.

    Args:
        front_sensor: IR sensor on the front of the chassis.
        back_sensor: IR sensor on the rear of the chassis.
        detection_threshold: Confidence value (0--1) each sensor must reach to
            count as detecting a white surface.

    Returns:
        Sequential: measure + corrective turn + strafe-until-black (leftward).

    Example::

        from raccoon.step.motion.lineup.strafe import strafe_left_lineup_on_white

        step = strafe_left_lineup_on_white(
            front_sensor=robot.front_line_sensor,
            back_sensor=robot.back_line_sensor,
            detection_threshold=0.65,
        )
        step.run(robot)
    """
    return seq([
        strafe_lineup(
            front_sensor=front_sensor,
            back_sensor=back_sensor,
            target=SurfaceColor.WHITE,
            strafe_speed=-1.0,
            detection_threshold=detection_threshold
        ),
        strafe_left(speed=0.3).until(
            on_black(front_sensor) | on_black(back_sensor)
        ),
    ])
