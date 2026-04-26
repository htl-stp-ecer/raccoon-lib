"""
Forward/backward lineup on lines using two IR sensors.

Drives until both sensors hit a line, measures the distance between hits,
then computes and executes a corrective turn angle.
"""

from __future__ import annotations

import math
import time
from enum import Enum
from typing import TYPE_CHECKING

from raccoon.foundation import ChassisVelocity, info
from raccoon.sensor_ir import IRSensor
from raccoon.step import Run, Sequential, defer, seq
from raccoon.step.condition import on_black, on_white

from ... import dsl
from ..drive_dsl import drive_backward, drive_forward
from ..motion_step import MotionStep
from ..turn_dsl import turn_left, turn_right

if TYPE_CHECKING:
    from raccoon.robot.api import GenericRobot


class SurfaceColor(Enum):
    """Target surface color for sensor-based motion."""

    BLACK = "black"
    WHITE = "white"


@dsl(hidden=True)
class TimingBasedLineUp(MotionStep):
    """Drive forward or backward until both left and right IR sensors detect a line.

    The robot drives at ``forward_speed`` while polling both sensors each update
    cycle.  When the first sensor crosses the target-colored line its odometry
    position is recorded.  Driving continues until the second sensor also
    crosses.  The distance traveled between the two hits is stored in
    ``results`` as ``(first_sensor_name, distance_m)`` so a downstream step can
    compute the corrective turn angle.

    This is an internal building-block step -- use the public
    ``forward_lineup_on_black`` / ``backward_lineup_on_white`` (etc.) factory
    functions instead.

    Args:
        left_sensor: IR sensor mounted on the left side of the chassis.
        right_sensor: IR sensor mounted on the right side of the chassis.
        target: The surface color to detect (BLACK or WHITE).
        forward_speed: Driving speed in m/s.  Positive = forward, negative =
            backward.
        detection_threshold: Confidence value (0--1) a sensor must reach to
            count as having detected the target color.
    """

    def __init__(
        self,
        left_sensor: IRSensor,
        right_sensor: IRSensor,
        target: SurfaceColor = SurfaceColor.BLACK,
        forward_speed: float = 1.0,
        detection_threshold: float = 0.9,
    ):
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
        self.trace(
            f"Left conf: {left_conf:.3f}, Right conf: {right_conf:.3f}, Distance: {robot.odometry.get_distance_from_origin()}"
        )

        if not self._left_triggered and left_conf >= self.threshold:
            self._left_triggered = True
            if self._t_first is None:
                self._t_first = now
                self._first_sensor = "left"
                self._first_hit_distance = current_distance
                self.debug("Left sensor hit line at t = 0.000s")
            else:
                elapsed = now - self._t_first
                self.distance_between_hits_m = abs(current_distance - self._first_hit_distance)
                self.debug(
                    f"Left sensor hit line at t = {elapsed:.3f}s, distance = {self.distance_between_hits_m * 100:.2f}cm"
                )
                return True

        if not self._right_triggered and right_conf >= self.threshold:
            self._right_triggered = True
            if self._t_first is None:
                self._t_first = now
                self._first_sensor = "right"
                self._first_hit_distance = current_distance
                self.debug("Right sensor hit line at t = 0.000s")
            else:
                elapsed = now - self._t_first
                self.distance_between_hits_m = abs(current_distance - self._first_hit_distance)
                self.debug(
                    f"Right sensor hit line at t = {elapsed:.3f}s, distance = {self.distance_between_hits_m * 100:.2f}cm"
                )
                return True

        return self._left_triggered and self._right_triggered

    def on_stop(self, robot: "GenericRobot") -> None:
        robot.drive.hard_stop()
        self.debug(f"Distance between sensor hits: {self.distance_between_hits_m * 100:.2f}cm")
        self.results = (self._first_sensor, self.distance_between_hits_m)


def _compute_lineup_turn(measure: TimingBasedLineUp, robot: "GenericRobot"):
    sensor_gap_m = robot.distance_between_sensors(measure.left_sensor, measure.right_sensor) / 100
    first_sensor, distance_driven = measure.results
    angle_rad = math.atan(distance_driven / sensor_gap_m)
    if first_sensor == "right":
        angle_rad = -angle_rad
    # When driving backward the sensor-to-line geometry is mirrored,
    # so the corrective turn must go in the opposite direction.
    if measure.forward_speed < 0:
        angle_rad = -angle_rad
    degrees = math.degrees(abs(angle_rad))

    if abs(degrees) < 0.1:
        info(f"Already at target heading (error={degrees:.3f}°) — skipping turn")
        return Run(lambda _robot: None)

    return turn_left(degrees) if angle_rad >= 0 else turn_right(degrees)


@dsl(hidden=True)
def lineup(
    left_sensor: IRSensor,
    right_sensor: IRSensor,
    target: SurfaceColor = SurfaceColor.BLACK,
    forward_speed: float = 1.0,
    detection_threshold: float = 0.7,
) -> Sequential:
    """Measure angular skew from a line using two IR sensors, then correct with a turn.

    Composes a ``TimingBasedLineUp`` measurement phase with a deferred
    corrective turn.  During the measurement the robot drives until both
    sensors cross the target line.  The distance between the two hit positions
    and the known physical gap between the sensors are used to compute a
    corrective turn angle via ``atan(distance_driven / sensor_gap)``.  The turn
    is then executed to align the robot perpendicular to the line.

    This is an internal helper -- prefer the public ``forward_lineup_on_black``
    family of functions.

    Args:
        left_sensor: IR sensor on the left side of the chassis.
        right_sensor: IR sensor on the right side of the chassis.
        target: Surface color to detect (BLACK or WHITE).
        forward_speed: Driving speed in m/s (negative for backward).
        detection_threshold: Confidence (0--1) required to register a hit.

    Returns:
        Sequential: A two-step sequence (measure + corrective turn).
    """
    measure = TimingBasedLineUp(
        left_sensor,
        right_sensor,
        target,
        forward_speed=forward_speed,
        detection_threshold=detection_threshold,
    )

    return seq(
        [
            measure,
            defer(lambda robot: _compute_lineup_turn(measure, robot)),
        ]
    )


@dsl(tags=["motion", "lineup"])
def forward_lineup_on_black(
    left_sensor: IRSensor, right_sensor: IRSensor, detection_threshold: float = 0.7
) -> Sequential:
    """Drive forward onto a black line, align perpendicular, then clear to white.

    The robot drives forward until both IR sensors detect a black line.  The
    stagger distance between the two sensor hits is used to compute a
    corrective turn that aligns the chassis perpendicular to the line.  After
    the turn the robot creeps forward at half speed until both sensors see
    white, leaving it just past the line and squared up.

    Prerequisites:
        Two IR line sensors mounted on the left and right sides of the chassis,
        with the robot's ``distance_between_sensors`` configured.

    Args:
        left_sensor: IR sensor on the left side of the chassis.
        right_sensor: IR sensor on the right side of the chassis.
        detection_threshold: Confidence value (0--1) each sensor must reach to
            count as detecting a line.  Lower values trigger earlier but are
            more susceptible to noise.

    Returns:
        Sequential: measure + corrective turn + drive-until-white.

    Example::

        from raccoon.step.motion.lineup.forward import forward_lineup_on_black

        step = forward_lineup_on_black(
            left_sensor=robot.left_line_sensor,
            right_sensor=robot.right_line_sensor,
            detection_threshold=0.8,
        )
        step.run(robot)
    """
    return seq(
        [
            lineup(
                left_sensor=left_sensor,
                right_sensor=right_sensor,
                target=SurfaceColor.BLACK,
                detection_threshold=detection_threshold,
            ),
            drive_forward(speed=0.5).until(on_white(left_sensor) | on_white(right_sensor)),
        ]
    )


@dsl(tags=["motion", "lineup"])
def forward_lineup_on_white(
    left_sensor: IRSensor, right_sensor: IRSensor, detection_threshold: float = 0.7
) -> Sequential:
    """Drive forward onto a white line, align perpendicular, then clear to black.

    The robot drives forward until both IR sensors detect a white surface.  The
    stagger distance between the two sensor hits is used to compute a
    corrective turn that aligns the chassis perpendicular to the line edge.
    After the turn the robot creeps forward at half speed until both sensors see
    black, leaving it just past the white region and squared up.

    Prerequisites:
        Two IR line sensors mounted on the left and right sides of the chassis,
        with the robot's ``distance_between_sensors`` configured.

    Args:
        left_sensor: IR sensor on the left side of the chassis.
        right_sensor: IR sensor on the right side of the chassis.
        detection_threshold: Confidence value (0--1) each sensor must reach to
            count as detecting a white surface.  Lower values trigger earlier
            but are more susceptible to noise.

    Returns:
        Sequential: measure + corrective turn + drive-until-black.

    Example::

        from raccoon.step.motion.lineup.forward import forward_lineup_on_white

        step = forward_lineup_on_white(
            left_sensor=robot.left_line_sensor,
            right_sensor=robot.right_line_sensor,
        )
        step.run(robot)
    """
    return seq(
        [
            lineup(
                left_sensor=left_sensor,
                right_sensor=right_sensor,
                target=SurfaceColor.WHITE,
                detection_threshold=detection_threshold,
            ),
            drive_forward(speed=0.5).until(on_black(left_sensor) | on_black(right_sensor)),
        ]
    )


@dsl(tags=["motion", "lineup"])
def backward_lineup_on_black(
    left_sensor: IRSensor, right_sensor: IRSensor, detection_threshold: float = 0.7
) -> Sequential:
    """Drive backward onto a black line, align perpendicular, then clear to white.

    Identical to ``forward_lineup_on_black`` but the robot reverses into the
    line instead of driving forward.  The corrective turn direction is
    automatically mirrored to account for the reversed geometry.  After
    alignment the robot continues backward at half speed until both sensors see
    white.

    Prerequisites:
        Two IR line sensors mounted on the left and right sides of the chassis,
        with the robot's ``distance_between_sensors`` configured.

    Args:
        left_sensor: IR sensor on the left side of the chassis.
        right_sensor: IR sensor on the right side of the chassis.
        detection_threshold: Confidence value (0--1) each sensor must reach to
            count as detecting a line.

    Returns:
        Sequential: measure (backward) + corrective turn + drive-until-white
        (backward).

    Example::

        from raccoon.step.motion.lineup.forward import backward_lineup_on_black

        step = backward_lineup_on_black(
            left_sensor=robot.left_line_sensor,
            right_sensor=robot.right_line_sensor,
            detection_threshold=0.7,
        )
        step.run(robot)
    """
    return seq(
        [
            lineup(
                left_sensor=left_sensor,
                right_sensor=right_sensor,
                target=SurfaceColor.BLACK,
                forward_speed=-1.0,
                detection_threshold=detection_threshold,
            ),
            drive_backward(speed=0.5).until(on_white(left_sensor) | on_white(right_sensor)),
        ]
    )


@dsl(tags=["motion", "lineup"])
def backward_lineup_on_white(
    left_sensor: IRSensor, right_sensor: IRSensor, detection_threshold: float = 0.7
) -> Sequential:
    """Drive backward onto a white line, align perpendicular, then clear to black.

    Identical to ``forward_lineup_on_white`` but the robot reverses into the
    line instead of driving forward.  The corrective turn direction is
    automatically mirrored to account for the reversed geometry.  After
    alignment the robot continues backward at half speed until both sensors see
    black.

    Prerequisites:
        Two IR line sensors mounted on the left and right sides of the chassis,
        with the robot's ``distance_between_sensors`` configured.

    Args:
        left_sensor: IR sensor on the left side of the chassis.
        right_sensor: IR sensor on the right side of the chassis.
        detection_threshold: Confidence value (0--1) each sensor must reach to
            count as detecting a white surface.

    Returns:
        Sequential: measure (backward) + corrective turn + drive-until-black
        (backward).

    Example::

        from raccoon.step.motion.lineup.forward import backward_lineup_on_white

        step = backward_lineup_on_white(
            left_sensor=robot.left_line_sensor,
            right_sensor=robot.right_line_sensor,
        )
        step.run(robot)
    """
    return seq(
        [
            lineup(
                left_sensor=left_sensor,
                right_sensor=right_sensor,
                target=SurfaceColor.WHITE,
                forward_speed=-1.0,
                detection_threshold=detection_threshold,
            ),
            drive_backward(speed=0.5).until(on_black(left_sensor) | on_black(right_sensor)),
        ]
    )
