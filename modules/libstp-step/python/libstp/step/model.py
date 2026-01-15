from dataclasses import dataclass
from typing import runtime_checkable, Protocol, TYPE_CHECKING

if TYPE_CHECKING:
    from libstp.robot.api import GenericRobot


@runtime_checkable
class StepProtocol(Protocol):
    async def run_step(self, robot: "GenericRobot") -> None: ...

@dataclass
class SimulationStepDelta:
    forward: float # in meters
    strafe: float # in meters
    angular: float # in radians

@dataclass
class SimulationStep:
    # unique identifier for this step
    id: str
    # For display purposes only
    label: str | None
    # how long does it take on avg to execute this step
    average_duration_ms: float
    # std deviation of execution time
    duration_stddev_ms: float
    # how much this step changes the robot's position
    delta: SimulationStepDelta