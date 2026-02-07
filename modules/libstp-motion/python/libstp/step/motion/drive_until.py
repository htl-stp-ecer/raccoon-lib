"""
Move until a sensor condition is met.

This module provides steps for moving (drive, turn, strafe) until IR sensors
detect black or white lines.
"""
import asyncio
from dataclasses import dataclass, field
from enum import Enum
from libstp.foundation import ChassisVelocity
from libstp.sensor_ir import IRSensor
from typing import TYPE_CHECKING, Union

from .. import Step, SimulationStep, SimulationStepDelta, dsl

if TYPE_CHECKING:
    from libstp.robot.api import GenericRobot


class SurfaceColor(Enum):
    """Target surface color for sensor-based motion."""
    BLACK = "black"
    WHITE = "white"


@dataclass
class MoveUntilConfig:
    """Configuration for MoveUntil step."""
    sensor: Union[IRSensor, list[IRSensor]]  # Single sensor or list of sensors
    target: SurfaceColor
    forward_speed: float = 0.0  # m/s, positive = forward
    angular_speed: float = 0.0  # rad/s, positive = CCW
    strafe_speed: float = 0.0  # m/s, positive = left
    confidence_threshold: float = 0.7
    scale_speed_on_approach: bool = True  # slow down as we approach target


@dsl(hidden=True)
class MoveUntil(Step):
    """
    Move until a sensor detects a specified color (black or white).

    Supports any combination of forward, angular, and strafe velocities.
    Optionally scales speed down as the sensor approaches the target confidence.
    Can accept a single sensor or a list of sensors - triggers when ANY sensor detects.
    """

    def __init__(self, config: MoveUntilConfig):
        super().__init__()
        self.config = config
        # Normalize to list for internal use
        self._sensors: list[IRSensor] = (
            config.sensor if isinstance(config.sensor, list) else [config.sensor]
        )
        # Track which sensor triggered detection (set after execution)
        self.triggered_sensor: IRSensor | None = None

    def _generate_signature(self) -> str:
        parts = []
        if self.config.forward_speed != 0:
            parts.append(f"fwd={self.config.forward_speed:.2f}")
        if self.config.angular_speed != 0:
            parts.append(f"ang={self.config.angular_speed:.2f}")
        if self.config.strafe_speed != 0:
            parts.append(f"str={self.config.strafe_speed:.2f}")
        return f"MoveUntil({', '.join(parts)}, target={self.config.target.value})"

    def to_simulation_step(self) -> SimulationStep:
        base = super().to_simulation_step()
        # Estimate movement based on primary direction
        base.delta = SimulationStepDelta(
            forward=0.1 if self.config.forward_speed > 0 else (-0.1 if self.config.forward_speed < 0 else 0.0),
            strafe=0.1 if self.config.strafe_speed > 0 else (-0.1 if self.config.strafe_speed < 0 else 0.0),
            angular=0.2 if self.config.angular_speed > 0 else (-0.2 if self.config.angular_speed < 0 else 0.0),
        )
        return base

    def _is_condition_met(self) -> bool:
        """Check if any sensor has detected the target color."""
        for sensor in self._sensors:
            if self.config.target == SurfaceColor.BLACK:
                confidence = sensor.probabilityOfBlack()
            else:
                confidence = sensor.probabilityOfWhite()
            if confidence >= self.config.confidence_threshold:
                self.triggered_sensor = sensor
                return True
        return False

    def _get_velocity(self) -> ChassisVelocity:
        """Get velocity, optionally scaled by distance to target."""
        return ChassisVelocity(
            self.config.forward_speed,
            self.config.strafe_speed,
            self.config.angular_speed
        )

    async def _execute_step(self, robot: "GenericRobot") -> None:
        """Execute the move until step."""
        update_rate = 1 / 100
        last_time = asyncio.get_event_loop().time() - update_rate

        velocity = self._get_velocity()
        robot.drive.set_velocity(velocity)

        while not self._is_condition_met():
            current_time = asyncio.get_event_loop().time()
            delta_time = max(current_time - last_time, 0.0)
            last_time = current_time

            if delta_time < 1e-4:
                await asyncio.sleep(update_rate)
                continue

            robot.drive.update(delta_time)

            await asyncio.sleep(update_rate)

        robot.drive.hard_stop()


# =============================================================================
# Drive (forward/backward) until sensor
# =============================================================================

@dsl(hidden=True)
def drive_until_black(
        sensor: Union[IRSensor, list[IRSensor]],
        forward_speed: float,
        confidence_threshold: float = 0.7,
) -> MoveUntil:
    """
    Drive forward/backward until any sensor detects black.

    Args:
        sensor: Single IR sensor or list of sensors (triggers when ANY detects)
        forward_speed: Speed in m/s (positive = forward, negative = backward)
        confidence_threshold: Probability threshold for detection (0-1)
    """
    return MoveUntil(MoveUntilConfig(
        sensor=sensor,
        target=SurfaceColor.BLACK,
        forward_speed=forward_speed,
        confidence_threshold=confidence_threshold,
    ))


@dsl(hidden=True)
def drive_until_white(
        sensor: Union[IRSensor, list[IRSensor]],
        forward_speed: float,
        confidence_threshold: float = 0.7,
) -> MoveUntil:
    """
    Drive forward/backward until any sensor detects white.

    Args:
        sensor: Single IR sensor or list of sensors (triggers when ANY detects)
        forward_speed: Speed in m/s (positive = forward, negative = backward)
        confidence_threshold: Probability threshold for detection (0-1)
    """
    return MoveUntil(MoveUntilConfig(
        sensor=sensor,
        target=SurfaceColor.WHITE,
        forward_speed=forward_speed,
        confidence_threshold=confidence_threshold,
    ))


@dsl(tags=["motion", "sensor"])
def drive_forward_until_black(
        sensor: Union[IRSensor, list[IRSensor]],
        speed: float = 1.0,
        confidence_threshold: float = 0.7,
) -> MoveUntil:
    """
    Drive forward until any sensor detects black.

    Args:
        sensor: Single IR sensor or list of sensors (triggers when ANY detects)
        speed: Forward speed in m/s (positive value)
        confidence_threshold: Probability threshold for detection (0-1)
    """
    return MoveUntil(MoveUntilConfig(
        sensor=sensor,
        target=SurfaceColor.BLACK,
        forward_speed=abs(speed),
        confidence_threshold=confidence_threshold,
    ))


@dsl(tags=["motion", "sensor"])
def drive_forward_until_white(
        sensor: Union[IRSensor, list[IRSensor]],
        speed: float = 1.0,
        confidence_threshold: float = 0.7,
) -> MoveUntil:
    """
    Drive forward until any sensor detects white.

    Args:
        sensor: Single IR sensor or list of sensors (triggers when ANY detects)
        speed: Forward speed in m/s (positive value)
        confidence_threshold: Probability threshold for detection (0-1)
    """
    return MoveUntil(MoveUntilConfig(
        sensor=sensor,
        target=SurfaceColor.WHITE,
        forward_speed=abs(speed),
        confidence_threshold=confidence_threshold,
    ))


@dsl(tags=["motion", "sensor"])
def drive_backward_until_black(
        sensor: Union[IRSensor, list[IRSensor]],
        speed: float = 1.0,
        confidence_threshold: float = 0.7,
) -> MoveUntil:
    """
    Drive backward until any sensor detects black.

    Args:
        sensor: Single IR sensor or list of sensors (triggers when ANY detects)
        speed: Backward speed in m/s (positive value)
        confidence_threshold: Probability threshold for detection (0-1)
    """
    return MoveUntil(MoveUntilConfig(
        sensor=sensor,
        target=SurfaceColor.BLACK,
        forward_speed=-abs(speed),
        confidence_threshold=confidence_threshold,
    ))


@dsl(tags=["motion", "sensor"])
def drive_backward_until_white(
        sensor: Union[IRSensor, list[IRSensor]],
        speed: float = 1.0,
        confidence_threshold: float = 0.7,
) -> MoveUntil:
    """
    Drive backward until any sensor detects white.

    Args:
        sensor: Single IR sensor or list of sensors (triggers when ANY detects)
        speed: Backward speed in m/s (positive value)
        confidence_threshold: Probability threshold for detection (0-1)
    """
    return MoveUntil(MoveUntilConfig(
        sensor=sensor,
        target=SurfaceColor.WHITE,
        forward_speed=-abs(speed),
        confidence_threshold=confidence_threshold,
    ))


# =============================================================================
# Turn until sensor
# =============================================================================

@dsl(hidden=True)
def turn_until_black(
        sensor: Union[IRSensor, list[IRSensor]],
        angular_speed: float,
        confidence_threshold: float = 0.7,
) -> MoveUntil:
    """
    Turn until any sensor detects black.

    Args:
        sensor: Single IR sensor or list of sensors (triggers when ANY detects)
        angular_speed: Speed in rad/s (positive = CCW, negative = CW)
        confidence_threshold: Probability threshold for detection (0-1)
    """
    return MoveUntil(MoveUntilConfig(
        sensor=sensor,
        target=SurfaceColor.BLACK,
        angular_speed=angular_speed,
        confidence_threshold=confidence_threshold,
    ))


@dsl(hidden=True)
def turn_until_white(
        sensor: Union[IRSensor, list[IRSensor]],
        angular_speed: float,
        confidence_threshold: float = 0.7,
) -> MoveUntil:
    """
    Turn until any sensor detects white.

    Args:
        sensor: Single IR sensor or list of sensors (triggers when ANY detects)
        angular_speed: Speed in rad/s (positive = CCW, negative = CW)
        confidence_threshold: Probability threshold for detection (0-1)
    """
    return MoveUntil(MoveUntilConfig(
        sensor=sensor,
        target=SurfaceColor.WHITE,
        angular_speed=angular_speed,
        confidence_threshold=confidence_threshold,
    ))


@dsl(tags=["motion", "sensor"])
def turn_left_until_black(
        sensor: Union[IRSensor, list[IRSensor]],
        speed: float = 1.0,
        confidence_threshold: float = 0.7,
) -> MoveUntil:
    """
    Turn left (CCW) until any sensor detects black.

    Args:
        sensor: Single IR sensor or list of sensors (triggers when ANY detects)
        speed: Angular speed in rad/s (positive value)
        confidence_threshold: Probability threshold for detection (0-1)
    """
    return MoveUntil(MoveUntilConfig(
        sensor=sensor,
        target=SurfaceColor.BLACK,
        angular_speed=abs(speed),
        confidence_threshold=confidence_threshold,
    ))


@dsl(tags=["motion", "sensor"])
def turn_left_until_white(
        sensor: Union[IRSensor, list[IRSensor]],
        speed: float = 1.0,
        confidence_threshold: float = 0.7,
) -> MoveUntil:
    """
    Turn left (CCW) until any sensor detects white.

    Args:
        sensor: Single IR sensor or list of sensors (triggers when ANY detects)
        speed: Angular speed in rad/s (positive value)
        confidence_threshold: Probability threshold for detection (0-1)
    """
    return MoveUntil(MoveUntilConfig(
        sensor=sensor,
        target=SurfaceColor.WHITE,
        angular_speed=abs(speed),
        confidence_threshold=confidence_threshold,
    ))


@dsl(tags=["motion", "sensor"])
def turn_right_until_black(
        sensor: Union[IRSensor, list[IRSensor]],
        speed: float = 1.0,
        confidence_threshold: float = 0.7,
) -> MoveUntil:
    """
    Turn right (CW) until any sensor detects black.

    Args:
        sensor: Single IR sensor or list of sensors (triggers when ANY detects)
        speed: Angular speed in rad/s (positive value)
        confidence_threshold: Probability threshold for detection (0-1)
    """
    return MoveUntil(MoveUntilConfig(
        sensor=sensor,
        target=SurfaceColor.BLACK,
        angular_speed=-abs(speed),
        confidence_threshold=confidence_threshold,
    ))


@dsl(tags=["motion", "sensor"])
def turn_right_until_white(
        sensor: Union[IRSensor, list[IRSensor]],
        speed: float = 1.0,
        confidence_threshold: float = 0.7,
) -> MoveUntil:
    """
    Turn right (CW) until any sensor detects white.

    Args:
        sensor: Single IR sensor or list of sensors (triggers when ANY detects)
        speed: Angular speed in rad/s (positive value)
        confidence_threshold: Probability threshold for detection (0-1)
    """
    return MoveUntil(MoveUntilConfig(
        sensor=sensor,
        target=SurfaceColor.WHITE,
        angular_speed=-abs(speed),
        confidence_threshold=confidence_threshold,
    ))


# =============================================================================
# Strafe until sensor
# =============================================================================

@dsl(hidden=True)
def strafe_until_black(
        sensor: Union[IRSensor, list[IRSensor]],
        strafe_speed: float,
        confidence_threshold: float = 0.7,
) -> MoveUntil:
    """
    Strafe until any sensor detects black.

    Args:
        sensor: Single IR sensor or list of sensors (triggers when ANY detects)
        strafe_speed: Speed in m/s (positive = left, negative = right)
        confidence_threshold: Probability threshold for detection (0-1)
    """
    return MoveUntil(MoveUntilConfig(
        sensor=sensor,
        target=SurfaceColor.BLACK,
        strafe_speed=strafe_speed,
        confidence_threshold=confidence_threshold,
    ))


@dsl(hidden=True)
def strafe_until_white(
        sensor: Union[IRSensor, list[IRSensor]],
        strafe_speed: float,
        confidence_threshold: float = 0.7,
) -> MoveUntil:
    """
    Strafe until any sensor detects white.

    Args:
        sensor: Single IR sensor or list of sensors (triggers when ANY detects)
        strafe_speed: Speed in m/s (positive = left, negative = right)
        confidence_threshold: Probability threshold for detection (0-1)
    """
    return MoveUntil(MoveUntilConfig(
        sensor=sensor,
        target=SurfaceColor.WHITE,
        strafe_speed=strafe_speed,
        confidence_threshold=confidence_threshold,
    ))


@dsl(tags=["motion", "sensor"])
def strafe_left_until_black(
        sensor: Union[IRSensor, list[IRSensor]],
        speed: float = 0.3,
        confidence_threshold: float = 0.7,
) -> MoveUntil:
    """
    Strafe left until any sensor detects black.

    Args:
        sensor: Single IR sensor or list of sensors (triggers when ANY detects)
        speed: Strafe speed in m/s (positive value)
        confidence_threshold: Probability threshold for detection (0-1)
    """
    return MoveUntil(MoveUntilConfig(
        sensor=sensor,
        target=SurfaceColor.BLACK,
        strafe_speed=-abs(speed),
        confidence_threshold=confidence_threshold,
    ))


@dsl(tags=["motion", "sensor"])
def strafe_left_until_white(
        sensor: Union[IRSensor, list[IRSensor]],
        speed: float = 0.3,
        confidence_threshold: float = 0.7,
) -> MoveUntil:
    """
    Strafe left until any sensor detects white.

    Args:
        sensor: Single IR sensor or list of sensors (triggers when ANY detects)
        speed: Strafe speed in m/s (positive value)
        confidence_threshold: Probability threshold for detection (0-1)
    """
    return MoveUntil(MoveUntilConfig(
        sensor=sensor,
        target=SurfaceColor.WHITE,
        strafe_speed=-abs(speed),
        confidence_threshold=confidence_threshold,
    ))


@dsl(tags=["motion", "sensor"])
def strafe_right_until_black(
        sensor: Union[IRSensor, list[IRSensor]],
        speed: float = 0.3,
        confidence_threshold: float = 0.7,
) -> MoveUntil:
    """
    Strafe right until any sensor detects black.

    Args:
        sensor: Single IR sensor or list of sensors (triggers when ANY detects)
        speed: Strafe speed in m/s (positive value)
        confidence_threshold: Probability threshold for detection (0-1)
    """
    return MoveUntil(MoveUntilConfig(
        sensor=sensor,
        target=SurfaceColor.BLACK,
        strafe_speed=abs(speed),
        confidence_threshold=confidence_threshold,
    ))


@dsl(tags=["motion", "sensor"])
def strafe_right_until_white(
        sensor: Union[IRSensor, list[IRSensor]],
        speed: float = 0.3,
        confidence_threshold: float = 0.7,
) -> MoveUntil:
    """
    Strafe right until any sensor detects white.

    Args:
        sensor: Single IR sensor or list of sensors (triggers when ANY detects)
        speed: Strafe speed in m/s (positive value)
        confidence_threshold: Probability threshold for detection (0-1)
    """
    return MoveUntil(MoveUntilConfig(
        sensor=sensor,
        target=SurfaceColor.WHITE,
        strafe_speed=abs(speed),
        confidence_threshold=confidence_threshold,
    ))


# =============================================================================
# Generic move_until for custom combinations
# =============================================================================

@dsl(hidden=True)
def move_until(
        sensor: Union[IRSensor, list[IRSensor]],
        target: SurfaceColor,
        forward_speed: float = 0.0,
        angular_speed: float = 0.0,
        strafe_speed: float = 0.0,
        confidence_threshold: float = 0.7,
        scale_speed_on_approach: bool = True,
) -> MoveUntil:
    """
    Move with any combination of velocities until any sensor detects target color.

    Args:
        sensor: Single IR sensor or list of sensors (triggers when ANY detects)
        target: Target color (SurfaceColor.BLACK or SurfaceColor.WHITE)
        forward_speed: Forward speed in m/s (positive = forward)
        angular_speed: Angular speed in rad/s (positive = CCW)
        strafe_speed: Strafe speed in m/s (positive = left)
        confidence_threshold: Probability threshold for detection (0-1)
        scale_speed_on_approach: Slow down as sensor approaches target
    """
    return MoveUntil(MoveUntilConfig(
        sensor=sensor,
        target=target,
        forward_speed=forward_speed,
        angular_speed=angular_speed,
        strafe_speed=strafe_speed,
        confidence_threshold=confidence_threshold,
        scale_speed_on_approach=scale_speed_on_approach,
    ))
