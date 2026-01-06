import time
from abc import abstractmethod
from typing import Optional, Protocol, runtime_checkable

from libstp.class_name_logger import ClassNameLogger
from libstp.robot.api import GenericRobot

from dataclasses import dataclass


@runtime_checkable
class StepProtocol(Protocol):
    async def run_step(self, robot: GenericRobot) -> None: ...

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


class Step(ClassNameLogger):
    def __init__(self) -> None:
        pass

    async def run_step(self, robot: GenericRobot) -> None:
        """
        Execute the step logic with execution timing instrumentation.
        """
        self.debug(f"Executing {self.__class__.__name__} step")

        tracker = None
        signature: Optional[str] = None
        start_time: Optional[float] = None

        try:
            from libstp.timing import StepTimingTracker

            tracker = StepTimingTracker.get_instance()
            if tracker.config.enabled:
                signature = self._generate_signature()
                start_time = time.perf_counter()
        except Exception:
            tracker = None

        try:
            await self._execute_step(robot)
        finally:
            if tracker and tracker.config.enabled and signature is not None and start_time is not None:
                duration = time.perf_counter() - start_time
                try:
                    await tracker.record_execution(signature, duration)
                except Exception as exc:
                    self.error(f"Failed to record step timing: {exc}")

    def _generate_signature(self) -> str:
        """
        Generate a unique signature for this step and its parameters.
        Override in subclasses to include configuration details.
        """
        return self.__class__.__name__

    @abstractmethod
    async def _execute_step(self, robot: GenericRobot) -> None:
        """Actual step logic implemented by subclasses."""
        raise NotImplementedError

    @abstractmethod
    def to_simulation_step(self) -> SimulationStep:
        """Convert this step to a SimulationStep representation."""
        raise NotImplementedError
