from __future__ import annotations

from typing import TYPE_CHECKING

from raccoon.class_name_logger import ClassNameLogger

if TYPE_CHECKING:
    from .api import GenericRobot


class RobotService(ClassNameLogger):
    """Base class for robot services.

    Services encapsulate reusable business logic (e.g. drum navigation,
    arm control) and are lazily instantiated and cached per robot via
    ``robot.get_service(ServiceClass)``.

    Subclasses receive the robot instance in ``__init__`` and can access
    hardware through ``self.robot.defs``.
    """

    def __init__(self, robot: GenericRobot) -> None:
        self.robot = robot
