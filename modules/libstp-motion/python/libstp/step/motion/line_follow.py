"""
Line following using IR sensors.

This module provides steps for following lines using one or two IR sensors
with PID-based steering control.
"""
from dataclasses import dataclass
from enum import Enum
from typing import TYPE_CHECKING

from libstp.foundation import ChassisVelocity, PidConfig, PidController
from libstp.sensor_ir import IRSensor

from .. import SimulationStep, SimulationStepDelta, dsl
from .motion_step import MotionStep
from .move_until import SurfaceColor

if TYPE_CHECKING:
    from libstp.robot.api import GenericRobot


@dataclass
class LineFollowConfig:
    """Configuration for LineFollow step with two sensors."""
    left_sensor: IRSensor
    right_sensor: IRSensor
    forward_speed: float  # m/s
    distance_cm: float | None = None  # None = run until both black
    strafe_gain: float = 0.05  # how much to strafe based on error
    forward_reduction: float = 0.0  # reduce forward speed proportional to error
    kp: float = 0.75
    ki: float = 0.0
    kd: float = 0.5
    both_black_threshold: float = 0.7  # threshold for "both black" stop condition


class LineSide(Enum):
    """Which edge of the line to track with a single sensor."""
    LEFT = "left"
    RIGHT = "right"


@dataclass
class SingleLineFollowConfig:
    """Configuration for single-sensor line following.

    The sensor tracks the edge of a line using PID control.
    ``side`` selects which edge: LEFT means the sensor approaches
    from the left (steers right when it sees black), RIGHT is the
    opposite.
    """
    sensor: IRSensor
    forward_speed: float  # m/s
    distance_cm: float  # distance to follow
    side: LineSide = LineSide.LEFT
    kp: float = 1.0
    ki: float = 0.0
    kd: float = 0.3
    forward_reduction: float = 0.5  # slow down proportional to |steering|


@dsl(hidden=True)
class LineFollow(MotionStep):
    """
    Follow a line using two IR sensors with PID steering.

    The robot follows the edge of a line by comparing left and right sensor
    readings. The difference drives steering corrections via PID control.
    Forward speed can optionally be reduced when the error is large.
    """

    def __init__(self, config: LineFollowConfig):
        super().__init__()
        self.config = config
        self._pid: PidController | None = None
        self._target_distance_m: float | None = None

    def _generate_signature(self) -> str:
        mode = f"{self.config.distance_cm:.1f}cm" if self.config.distance_cm else "until_both_black"
        return (
            f"LineFollow(mode={mode}, speed={self.config.forward_speed:.2f})"
        )

    def to_simulation_step(self) -> SimulationStep:
        base = super().to_simulation_step()
        # Use configured distance or estimate
        distance_m = (self.config.distance_cm / 100.0) if self.config.distance_cm else 0.3
        base.delta = SimulationStepDelta(
            forward=distance_m,
            strafe=0.0,
            angular=0.0,
        )
        return base

    def on_start(self, robot: "GenericRobot") -> None:
        self._pid = PidController(PidConfig(kp=self.config.kp, ki=self.config.ki, kd=self.config.kd))
        self._target_distance_m = (self.config.distance_cm / 100.0) if self.config.distance_cm else None
        robot.odometry.reset()

    def on_update(self, robot: "GenericRobot", dt: float) -> bool:
        # Check stop condition
        if self._target_distance_m is not None:
            current_pose = robot.odometry.get_pose()
            traveled = abs(current_pose.position[0])  # x is forward
            if traveled >= self._target_distance_m:
                return True
        else:
            left_black = self.config.left_sensor.probabilityOfBlack()
            right_black = self.config.right_sensor.probabilityOfBlack()
            if (left_black >= self.config.both_black_threshold and
                right_black >= self.config.both_black_threshold):
                return True

        # Calculate error: difference between sensors
        left_conf = self.config.left_sensor.probabilityOfBlack()
        right_conf = self.config.right_sensor.probabilityOfBlack()
        error = left_conf - right_conf

        # PID output for steering
        pid_output = self._pid.update(error, dt)

        # Dynamic forward speed reduction based on error magnitude
        reduction = min(abs(pid_output) * self.config.forward_reduction, 1.0)
        forward = self.config.forward_speed * (1.0 - reduction)

        # Strafe and rotation from PID
        strafe = -pid_output * self.config.strafe_gain
        rotation = pid_output

        self.info(f"LineFollow: left={left_conf:.2f}, right={right_conf:.2f}, error={error:.2f}")

        velocity = ChassisVelocity(forward, strafe, rotation)
        robot.drive.set_velocity(velocity)
        robot.drive.update(dt)
        return False


@dsl(hidden=True)
class SingleSensorLineFollow(MotionStep):
    """
    Follow a line using a single IR sensor with PID edge-tracking.

    The sensor targets the edge of the line (probabilityOfBlack ≈ 0.5).
    PID drives angular velocity to keep the sensor on that edge.
    ``side`` determines which edge is tracked (flips the error sign).

    Terminates after traveling ``distance_cm``.
    """

    def __init__(self, config: SingleLineFollowConfig):
        super().__init__()
        self.config = config
        self._pid: PidController | None = None
        self._target_distance_m: float = 0.0

    def _generate_signature(self) -> str:
        return (
            f"SingleSensorLineFollow(distance={self.config.distance_cm:.1f}cm, "
            f"side={self.config.side.value}, speed={self.config.forward_speed:.2f})"
        )

    def to_simulation_step(self) -> SimulationStep:
        base = super().to_simulation_step()
        base.delta = SimulationStepDelta(
            forward=self.config.distance_cm / 100.0,
            strafe=0.0,
            angular=0.0,
        )
        return base

    def on_start(self, robot: "GenericRobot") -> None:
        cfg = self.config
        self._pid = PidController(PidConfig(kp=cfg.kp, ki=cfg.ki, kd=cfg.kd))
        self._target_distance_m = cfg.distance_cm / 100.0
        robot.odometry.reset()

    def on_update(self, robot: "GenericRobot", dt: float) -> bool:
        cfg = self.config

        # --- stop condition: distance traveled ---
        current_pose = robot.odometry.get_pose()
        traveled = abs(current_pose.position[0])
        if traveled >= self._target_distance_m:
            return True

        # --- edge-tracking error ---
        # 0.5 = edge of line; positive error = too far onto the black
        error = cfg.sensor.probabilityOfBlack() - 0.5
        if cfg.side == LineSide.RIGHT:
            error = -error

        # --- PID steering ---
        wz = self._pid.update(error, dt)

        # reduce forward speed when steering hard
        speed_factor = 1.0 - min(abs(wz) * cfg.forward_reduction, 0.8)
        vx = cfg.forward_speed * speed_factor

        self.debug(
            f"LineFollow1: black={cfg.sensor.probabilityOfBlack():.2f} "
            f"err={error:.2f} wz={wz:.2f} vx={vx:.2f}"
        )

        robot.drive.set_velocity(ChassisVelocity(vx, 0.0, wz))
        robot.drive.update(dt)
        return False


@dsl(tags=["motion", "line-follow"])
def follow_line(
    left_sensor: IRSensor,
    right_sensor: IRSensor,
    distance_cm: float,
    forward_speed: float = 0.5,
    strafe_gain: float = 0.05,
    forward_reduction: float = 0.0,
    kp: float = 0.75,
    ki: float = 0.0,
    kd: float = 0.5,
) -> LineFollow:
    """
    Follow a line for a specified distance using two sensors.

    Args:
        left_sensor: Left IR sensor instance
        right_sensor: Right IR sensor instance
        distance_cm: Distance to follow in centimeters
        forward_speed: Forward speed in m/s
        strafe_gain: How much to strafe based on PID output
        forward_reduction: Reduce forward speed by this fraction of error
        kp, ki, kd: PID gains for steering

    Returns:
        LineFollow step configured for distance-based following
    """
    config = LineFollowConfig(
        left_sensor=left_sensor,
        right_sensor=right_sensor,
        forward_speed=forward_speed,
        distance_cm=distance_cm,
        strafe_gain=strafe_gain,
        forward_reduction=forward_reduction,
        kp=kp, ki=ki, kd=kd,
    )
    return LineFollow(config)


@dsl(tags=["motion", "line-follow"])
def follow_line_until_both_black(
    left_sensor: IRSensor,
    right_sensor: IRSensor,
    forward_speed: float,
    strafe_gain: float = 0.05,
    forward_reduction: float = 0.0,
    kp: float = 0.75,
    ki: float = 0.0,
    kd: float = 0.5,
    both_black_threshold: float = 0.7,
) -> LineFollow:
    """
    Follow a line until both sensors detect black (intersection).

    Args:
        left_sensor: Left IR sensor instance
        right_sensor: Right IR sensor instance
        forward_speed: Forward speed in m/s
        strafe_gain: How much to strafe based on PID output
        forward_reduction: Reduce forward speed by this fraction of error
        kp, ki, kd: PID gains for steering
        both_black_threshold: Both sensors must exceed this to stop

    Returns:
        LineFollow step that stops at intersections
    """
    config = LineFollowConfig(
        left_sensor=left_sensor,
        right_sensor=right_sensor,
        forward_speed=forward_speed,
        distance_cm=None,  # Run until both black
        strafe_gain=strafe_gain,
        forward_reduction=forward_reduction,
        kp=kp, ki=ki, kd=kd,
        both_black_threshold=both_black_threshold,
    )
    return LineFollow(config)


@dsl(tags=["motion", "line-follow"])
def follow_line_single(
    sensor: IRSensor,
    distance_cm: float,
    forward_speed: float = 0.5,
    side: LineSide = LineSide.LEFT,
    kp: float = 1.0,
    ki: float = 0.0,
    kd: float = 0.3,
    forward_reduction: float = 0.5,
) -> SingleSensorLineFollow:
    """
    Follow a line using a single sensor for a specified distance.

    The sensor tracks the edge of the line (where probabilityOfBlack ≈ 0.5).
    ``side`` selects which edge to follow.

    Args:
        sensor: The IR sensor instance
        distance_cm: Distance to follow in centimeters
        forward_speed: Forward speed in m/s
        side: Which edge to track (LEFT or RIGHT)
        kp, ki, kd: PID gains for steering
        forward_reduction: Slow down by this fraction of |steering| on curves

    Returns:
        SingleSensorLineFollow step
    """
    config = SingleLineFollowConfig(
        sensor=sensor,
        forward_speed=forward_speed,
        side=side,
        distance_cm=distance_cm,
        kp=kp, ki=ki, kd=kd,
        forward_reduction=forward_reduction,
    )
    return SingleSensorLineFollow(config)
