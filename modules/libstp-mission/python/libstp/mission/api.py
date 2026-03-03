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

    async def run(self, robot):
        """Build the mission sequence and execute it on the provided robot."""
        self.debug(f"Executing {self.__class__.__name__}")
        await self.sequence().run_step(robot)
        self.debug(f"Completed {self.__class__.__name__}")

    @abstractmethod
    def sequence(self) -> "Step":
        """Return the root step tree for this mission."""
        raise NotImplementedError("Method sequence() not implemented")


class MissionController:
    """Placeholder controller API for future mission orchestration work."""

    def __init__(self, robot):
        """Store the robot instance that mission execution would target."""
        self.robot = robot

    async def execute_missions(self, missions: List[Mission]):
        """Reserved entry point for batch mission execution; not implemented yet."""
        initialize_timer()
        raise NotImplementedError("Method execute_missions() not implemented")
        # sequence = seq([mission.sequence() for mission in missions])
        # await sequence.run_step(self.device, self.definitions)
        # sequence.call_on_exit(None)
