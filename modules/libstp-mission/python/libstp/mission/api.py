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
    async def run(self, robot):
        self.debug(f"Executing {self.__class__.__name__}")
        await self.sequence().run_step(robot)
        self.debug(f"Completed {self.__class__.__name__}")

    @abstractmethod
    def sequence(self) -> "Step":
        raise NotImplementedError("Method sequence() not implemented")


# todo: Track mission lead time in csv file for later analysis of how much variance each mission has and how it changed over time
class MissionController:
    def __init__(self, robot):
        self.robot = robot

    async def execute_missions(self, missions: List[Mission]):
        initialize_timer()
        raise NotImplementedError("Method execute_missions() not implemented")
        # sequence = seq([mission.sequence() for mission in missions])
        # await sequence.run_step(self.device, self.definitions)
        # sequence.call_on_exit(None)