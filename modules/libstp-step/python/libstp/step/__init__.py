from abc import abstractmethod
from typing import Any, runtime_checkable, Protocol, Optional, TypeVar, Union, cast

from libstp.class_name_logger import ClassNameLogger
from libstp.robot.api import GenericRobot


@runtime_checkable
class StepProtocol(Protocol):
    async def run_step(self, robot: GenericRobot) -> None: ...

T = TypeVar('T')

class Step(ClassNameLogger):
    def __init__(self) -> None:
        pass

    @abstractmethod
    async def run_step(self, robot: GenericRobot) -> None:
        """
        Execute the step logic. Can only be run once.
        
        Args:
            robot: The robot instance to interact with hardware

        Raises:
            RuntimeError: If attempting to run a step that has already been executed
        """
        self.debug(f"Executing {self.__class__.__name__} step")