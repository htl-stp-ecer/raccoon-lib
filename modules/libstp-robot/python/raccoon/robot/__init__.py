"""Robot abstraction layer that ties missions, services, and geometry together."""

from __future__ import annotations

from .api import GenericRobot, RobotDefinitionsProtocol
from .geometry import RobotGeometry, SensorPosition, WheelPosition
from .service import RobotService
from raccoon.map import WorldMap as TableMap, MapSegment

__all__ = [
    "GenericRobot",
    "RobotService",
    "RobotDefinitionsProtocol",
    "RobotGeometry",
    "SensorPosition",
    "WheelPosition",
    "TableMap",
    "MapSegment",
]
