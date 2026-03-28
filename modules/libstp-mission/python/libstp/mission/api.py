from abc import abstractmethod
from typing import List, runtime_checkable, Protocol, TYPE_CHECKING

from libstp.class_name_logger import ClassNameLogger
from libstp.foundation import initialize_timer

if TYPE_CHECKING:
    from libstp.step.base import Step

@runtime_checkable
class MissionProtocol(Protocol):
    """Protocol for mission objects that can be run on a robot."""

    async def run(self, robot: "GenericRobot") -> None:
        """Execute the mission on the given robot."""
        ...


class Mission(ClassNameLogger, MissionProtocol):
    """Base mission that delegates execution to a root step sequence."""

    def __str__(self):
        return self.__class__.__name__

    def __repr__(self):
        return self.__class__.__name__

    async def run(self, robot):
        """Build the mission sequence and execute it on the provided robot."""
        self.info(f"Starting mission: {self.__class__.__name__}")
        await self.sequence().run_step(robot)
        self.info(f"Completed mission: {self.__class__.__name__}")

    @abstractmethod
    def sequence(self) -> "Step":
        """Return the root step tree for this mission."""
        raise NotImplementedError("Method sequence() not implemented")