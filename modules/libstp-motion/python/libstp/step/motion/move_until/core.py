"""
Core MoveUntil step and configuration.

Moves (drive, turn, strafe) until an IR sensor condition is met.
"""
from dataclasses import dataclass
from enum import Enum
from libstp.foundation import ChassisVelocity
from libstp.sensor_ir import IRSensor
from typing import TYPE_CHECKING, Union

from ... import SimulationStep, SimulationStepDelta, dsl
from ..motion_step import MotionStep

if TYPE_CHECKING:
    from libstp.robot.api import GenericRobot


class SurfaceColor(Enum):
    """Target surface color for sensor-based motion."""
    BLACK = "black"
    WHITE = "white"


@dataclass
class MoveUntilConfig:
    """Configuration for MoveUntil step."""
    sensor: Union[IRSensor, list[IRSensor]]
    target: SurfaceColor
    forward_speed: float = 0.0  # m/s, positive = forward
    angular_speed: float = 0.0  # rad/s, positive = CCW
    strafe_speed: float = 0.0  # m/s, positive = left
    confidence_threshold: float = 0.7
    scale_speed_on_approach: bool = True


@dsl(hidden=True)
class MoveUntil(MotionStep):
    """
    Move until a sensor detects a specified color (black or white).

    Supports any combination of forward, angular, and strafe velocities.
    Can accept a single sensor or a list of sensors - triggers when ANY sensor detects.
    """

    def __init__(self, config: MoveUntilConfig):
        super().__init__()
        self.config = config
        self._sensors: list[IRSensor] = (
            config.sensor if isinstance(config.sensor, list) else [config.sensor]
        )
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
        return ChassisVelocity(
            self.config.forward_speed,
            self.config.strafe_speed,
            self.config.angular_speed
        )

    def on_start(self, robot: "GenericRobot") -> None:
        robot.drive.set_velocity(self._get_velocity())

    def on_update(self, robot: "GenericRobot", dt: float) -> bool:
        robot.drive.update(dt)
        return self._is_condition_met()


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
