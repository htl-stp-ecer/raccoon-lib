"""Robot module for libstp.

Provides:
- GenericRobot: Abstract base class for robot implementations
- RobotGeometry: Mixin for geometry calculations
- SensorPosition, WheelPosition: Position dataclasses
"""

from .api import GenericRobot, RobotDefinitionsProtocol
from .geometry import RobotGeometry, SensorPosition, WheelPosition

__all__ = [
    "GenericRobot",
    "RobotDefinitionsProtocol",
    "RobotGeometry",
    "SensorPosition",
    "WheelPosition",
]
