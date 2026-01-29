"""
Move until a sensor condition is met.

This module provides steps for moving (drive, turn, strafe) until IR sensors
detect black or white lines.
"""
import asyncio
from dataclasses import dataclass
from enum import Enum
from typing import TYPE_CHECKING

from libstp.foundation import ChassisVelocity
from libstp.sensor_ir import IRSensor

from .. import Step, SimulationStep, SimulationStepDelta

if TYPE_CHECKING:
    from libstp.robot.api import GenericRobot


class SurfaceColor(Enum):
    """Target surface color for sensor-based motion."""
    BLACK = "black"
    WHITE = "white"


@dataclass
class MoveUntilConfig:
    """Configuration for MoveUntil step."""
    sensor: IRSensor
    target: SurfaceColor
    forward_speed: float = 0.0  # m/s, positive = forward
    angular_speed: float = 0.0  # rad/s, positive = CCW
    strafe_speed: float = 0.0   # m/s, positive = left
    confidence_threshold: float = 0.7
    scale_speed_on_approach: bool = True  # slow down as we approach target


class MoveUntil(Step):
    """
    Move until a sensor detects a specified color (black or white).

    Supports any combination of forward, angular, and strafe velocities.
    Optionally scales speed down as the sensor approaches the target confidence.
    """

    def __init__(self, config: MoveUntilConfig):
        super().__init__()
        self.config = config

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
        """Check if the sensor has detected the target color."""
        if self.config.target == SurfaceColor.BLACK:
            confidence = self.config.sensor.probabilityOfBlack()
        else:
            confidence = self.config.sensor.probabilityOfWhite()
        return confidence >= self.config.confidence_threshold

    def _get_velocity(self) -> ChassisVelocity:
        """Get velocity, optionally scaled by distance to target."""
        scale = 1.0
        if self.config.scale_speed_on_approach:
            # Scale by confidence of OPPOSITE color (high when far from target)
            if self.config.target == SurfaceColor.BLACK:
                scale = self.config.sensor.probabilityOfWhite()
            else:
                scale = self.config.sensor.probabilityOfBlack()
            scale = max(scale, 0.2)  # Minimum 20% speed

        return ChassisVelocity(
            self.config.forward_speed * scale,
            self.config.strafe_speed * scale,
            self.config.angular_speed * scale
        )

    async def _execute_step(self, robot: "GenericRobot") -> None:
        """Execute the move until step."""
        update_rate = 1 / 20  # 20 Hz
        last_time = asyncio.get_event_loop().time() - update_rate

        while not self._is_condition_met():
            current_time = asyncio.get_event_loop().time()
            delta_time = max(current_time - last_time, 0.0)
            last_time = current_time

            if delta_time < 1e-4:
                await asyncio.sleep(update_rate)
                continue

            velocity = self._get_velocity()
            robot.drive.set_velocity(velocity)
            robot.drive.update(delta_time)

            await asyncio.sleep(update_rate)

        robot.drive.hard_stop()


# Keep DriveUntil as alias for backwards compatibility
DriveUntil = MoveUntil
DriveUntilConfig = MoveUntilConfig


# =============================================================================
# Drive (forward/backward) until sensor
# =============================================================================

def drive_until_black(
    sensor: IRSensor,
    forward_speed: float,
    confidence_threshold: float = 0.7,
) -> MoveUntil:
    """
    Drive forward/backward until the sensor detects black.

    Args:
        sensor: The IR sensor instance
        forward_speed: Speed in m/s (positive = forward, negative = backward)
        confidence_threshold: Probability threshold for detection (0-1)
    """
    return MoveUntil(MoveUntilConfig(
        sensor=sensor,
        target=SurfaceColor.BLACK,
        forward_speed=forward_speed,
        confidence_threshold=confidence_threshold,
    ))


def drive_until_white(
    sensor: IRSensor,
    forward_speed: float,
    confidence_threshold: float = 0.7,
) -> MoveUntil:
    """
    Drive forward/backward until the sensor detects white.

    Args:
        sensor: The IR sensor instance
        forward_speed: Speed in m/s (positive = forward, negative = backward)
        confidence_threshold: Probability threshold for detection (0-1)
    """
    return MoveUntil(MoveUntilConfig(
        sensor=sensor,
        target=SurfaceColor.WHITE,
        forward_speed=forward_speed,
        confidence_threshold=confidence_threshold,
    ))


# =============================================================================
# Turn until sensor
# =============================================================================

def turn_until_black(
    sensor: IRSensor,
    angular_speed: float,
    confidence_threshold: float = 0.7,
) -> MoveUntil:
    """
    Turn until the sensor detects black.

    Args:
        sensor: The IR sensor instance
        angular_speed: Speed in rad/s (positive = CCW, negative = CW)
        confidence_threshold: Probability threshold for detection (0-1)
    """
    return MoveUntil(MoveUntilConfig(
        sensor=sensor,
        target=SurfaceColor.BLACK,
        angular_speed=angular_speed,
        confidence_threshold=confidence_threshold,
    ))


def turn_until_white(
    sensor: IRSensor,
    angular_speed: float,
    confidence_threshold: float = 0.7,
) -> MoveUntil:
    """
    Turn until the sensor detects white.

    Args:
        sensor: The IR sensor instance
        angular_speed: Speed in rad/s (positive = CCW, negative = CW)
        confidence_threshold: Probability threshold for detection (0-1)
    """
    return MoveUntil(MoveUntilConfig(
        sensor=sensor,
        target=SurfaceColor.WHITE,
        angular_speed=angular_speed,
        confidence_threshold=confidence_threshold,
    ))


# =============================================================================
# Strafe until sensor
# =============================================================================

def strafe_until_black(
    sensor: IRSensor,
    strafe_speed: float,
    confidence_threshold: float = 0.7,
) -> MoveUntil:
    """
    Strafe until the sensor detects black.

    Args:
        sensor: The IR sensor instance
        strafe_speed: Speed in m/s (positive = left, negative = right)
        confidence_threshold: Probability threshold for detection (0-1)
    """
    return MoveUntil(MoveUntilConfig(
        sensor=sensor,
        target=SurfaceColor.BLACK,
        strafe_speed=strafe_speed,
        confidence_threshold=confidence_threshold,
    ))


def strafe_until_white(
    sensor: IRSensor,
    strafe_speed: float,
    confidence_threshold: float = 0.7,
) -> MoveUntil:
    """
    Strafe until the sensor detects white.

    Args:
        sensor: The IR sensor instance
        strafe_speed: Speed in m/s (positive = left, negative = right)
        confidence_threshold: Probability threshold for detection (0-1)
    """
    return MoveUntil(MoveUntilConfig(
        sensor=sensor,
        target=SurfaceColor.WHITE,
        strafe_speed=strafe_speed,
        confidence_threshold=confidence_threshold,
    ))


# =============================================================================
# Generic move_until for custom combinations
# =============================================================================

def move_until(
    sensor: IRSensor,
    target: SurfaceColor,
    forward_speed: float = 0.0,
    angular_speed: float = 0.0,
    strafe_speed: float = 0.0,
    confidence_threshold: float = 0.7,
    scale_speed_on_approach: bool = True,
) -> MoveUntil:
    """
    Move with any combination of velocities until sensor detects target color.

    Args:
        sensor: The IR sensor instance
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
