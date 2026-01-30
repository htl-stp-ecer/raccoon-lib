"""
Line following using IR sensors.

This module provides steps for following lines using one or two IR sensors
with PID-based steering control.
"""
import asyncio
from dataclasses import dataclass
from typing import TYPE_CHECKING

from libstp.foundation import ChassisVelocity, PIDController, info
from libstp.sensor_ir import IRSensor

from .. import Step, SimulationStep, SimulationStepDelta
from .drive_until import SurfaceColor

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


@dataclass
class SingleLineFollowConfig:
    """Configuration for single-sensor line following."""
    sensor: IRSensor
    forward_speed: float  # m/s
    distance_cm: float
    threshold: float = 0.5  # black confidence above this = on line
    rotation_gain: float = 0.25  # how much to rotate based on error
    strafe_gain: float = 0.05  # how much to strafe based on error


class LineFollow(Step):
    """
    Follow a line using two IR sensors with PID steering.

    The robot follows the edge of a line by comparing left and right sensor
    readings. The difference drives steering corrections via PID control.
    Forward speed can optionally be reduced when the error is large.
    """

    def __init__(self, config: LineFollowConfig):
        super().__init__()
        self.config = config

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

    async def _execute_step(self, robot: "GenericRobot") -> None:
        """Execute the line follow step."""
        # Initialize PID
        pid = PIDController(self.config.kp, self.config.ki, self.config.kd)

        # Calculate target distance in meters
        target_distance_m = (self.config.distance_cm / 100.0) if self.config.distance_cm else None

        # Reset odometry to track distance
        robot.odometry.reset()

        update_rate = 1 / 20  # 20 Hz
        last_time = asyncio.get_event_loop().time() - update_rate

        while True:
            current_time = asyncio.get_event_loop().time()
            delta_time = max(current_time - last_time, 0.0)
            last_time = current_time

            # Check stop condition
            if target_distance_m is not None:
                # Distance-based stop
                current_pose = robot.odometry.get_pose()
                traveled = abs(current_pose.position[0])  # x is forward
                if traveled >= target_distance_m:
                    break
            else:
                # Stop when both sensors see black
                left_black = self.config.left_sensor.probabilityOfBlack()
                right_black = self.config.right_sensor.probabilityOfBlack()
                if (left_black >= self.config.both_black_threshold and
                    right_black >= self.config.both_black_threshold):
                    break

            if delta_time < 1e-4:
                await asyncio.sleep(update_rate)
                continue

            # Calculate error: difference between sensors
            left_conf = self.config.left_sensor.probabilityOfBlack()
            right_conf = self.config.right_sensor.probabilityOfBlack()
            error = left_conf - right_conf

            # PID output for steering
            pid_output = pid.calculate(error)

            # Dynamic forward speed reduction based on error magnitude
            reduction = min(abs(pid_output) * self.config.forward_reduction, 1.0)
            forward = self.config.forward_speed * (1.0 - reduction)

            # Strafe and rotation from PID
            strafe = -pid_output * self.config.strafe_gain
            rotation = pid_output

            info(f"LineFollow: left={left_conf:.2f}, right={right_conf:.2f}, error={error:.2f}, ")

            velocity = ChassisVelocity(forward, strafe, rotation)
            robot.drive.set_velocity(velocity)
            robot.drive.update(delta_time)

            await asyncio.sleep(update_rate)

        robot.drive.hard_stop()


class SingleSensorLineFollow(Step):
    """
    Follow a line using a single IR sensor.

    Uses the sensor's black confidence relative to a threshold to determine
    steering direction. Simpler than two-sensor following but less accurate.
    """

    def __init__(self, config: SingleLineFollowConfig):
        super().__init__()
        self.config = config

    def _generate_signature(self) -> str:
        return (
            f"SingleSensorLineFollow(distance={self.config.distance_cm:.1f}cm, "
            f"speed={self.config.forward_speed:.2f})"
        )

    def to_simulation_step(self) -> SimulationStep:
        base = super().to_simulation_step()
        distance_m = self.config.distance_cm / 100.0
        base.delta = SimulationStepDelta(
            forward=distance_m,
            strafe=0.0,
            angular=0.0,
        )
        return base

    async def _execute_step(self, robot: "GenericRobot") -> None:
        """Execute the single-sensor line follow step."""
        target_distance_m = self.config.distance_cm / 100.0

        # Reset odometry to track distance
        robot.odometry.reset()

        update_rate = 1 / 20  # 20 Hz
        last_time = asyncio.get_event_loop().time() - update_rate

        while True:
            current_time = asyncio.get_event_loop().time()
            delta_time = max(current_time - last_time, 0.0)
            last_time = current_time

            # Check distance
            current_pose = robot.odometry.get_pose()
            traveled = abs(current_pose.position[0])  # x is forward
            if traveled >= target_distance_m:
                break

            if delta_time < 1e-4:
                await asyncio.sleep(update_rate)
                continue

            # Calculate error relative to threshold
            confidence = self.config.sensor.probabilityOfBlack()
            error = confidence - self.config.threshold

            # Apply gains
            rotation = self.config.rotation_gain * error
            strafe = -self.config.strafe_gain * error

            velocity = ChassisVelocity(self.config.forward_speed, strafe, rotation)
            robot.drive.set_velocity(velocity)
            robot.drive.update(delta_time)

            await asyncio.sleep(update_rate)

        robot.drive.hard_stop()


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


def follow_line_single(
    sensor: IRSensor,
    distance_cm: float,
    forward_speed: float,
    threshold: float = 0.5,
    rotation_gain: float = 0.25,
    strafe_gain: float = 0.05,
) -> SingleSensorLineFollow:
    """
    Follow a line using a single sensor for a specified distance.

    Args:
        sensor: The IR sensor instance
        distance_cm: Distance to follow in centimeters
        forward_speed: Forward speed in m/s
        threshold: Black confidence above this = on line
        rotation_gain: How much to rotate based on error
        strafe_gain: How much to strafe based on error

    Returns:
        SingleSensorLineFollow step
    """
    config = SingleLineFollowConfig(
        sensor=sensor,
        forward_speed=forward_speed,
        distance_cm=distance_cm,
        threshold=threshold,
        rotation_gain=rotation_gain,
        strafe_gain=strafe_gain,
    )
    return SingleSensorLineFollow(config)
