import asyncio
import time
from abc import abstractmethod
from typing import Optional, TYPE_CHECKING

from .model import SimulationStep, SimulationStepDelta
from libstp.class_name_logger import ClassNameLogger
from libstp.timing import StepTimingTracker

if TYPE_CHECKING:
    from libstp.robot.api import GenericRobot


class Step(ClassNameLogger):
    def __init__(self) -> None:
        pass

    async def run_step(self, robot: "GenericRobot") -> None:
        """
        Execute the step logic with execution timing instrumentation.
        """
        self.debug(f"Executing {self.__class__.__name__} step")

        signature: Optional[str] = None
        start_time: Optional[float] = None

        tracker = StepTimingTracker.get_instance()
        if tracker.config.enabled:
            signature = self._generate_signature()
            start_time = time.perf_counter()

        step_start = time.perf_counter()
        try:
            await self._execute_step(robot)
        finally:
            elapsed = time.perf_counter() - step_start
            self.debug(f"Finished {self.__class__.__name__} step in {elapsed:.3f}s")

            if tracker and tracker.config.enabled and signature is not None and start_time is not None:
                duration = time.perf_counter() - start_time
                try:
                    await tracker.record_execution(signature, duration)
                except asyncio.CancelledError:
                    self.debug("Timing recording cancelled - skipping")
                except Exception as exc:
                    self.error(f"Failed to record step timing: {exc}")

    def _generate_signature(self) -> str:
        """
        Generate a unique signature for this step and its parameters.
        Override in subclasses to include configuration details.
        """
        return self.__class__.__name__

    @abstractmethod
    async def _execute_step(self, robot: "GenericRobot") -> None:
        """Actual step logic implemented by subclasses."""
        raise NotImplementedError

    def to_simulation_step(self) -> SimulationStep:
        """
        Convert this step to a SimulationStep representation.

        Default implementation uses timing data from the database if available,
        otherwise returns reasonable defaults. Override this method in subclasses
        to provide accurate delta values for position/heading changes.
        """
        signature = self._generate_signature()

        # Try to get timing statistics from database
        avg_duration_ms = 100.0  # Default: 100ms
        stddev_ms = 10.0  # Default: 10ms

        tracker = StepTimingTracker.get_instance()
        if tracker.config.enabled:

            async def fetch_timing() -> tuple[float, float]:
                durations = await tracker.database.fetch_recent_durations(
                    signature, tracker.config.window_size
                )
                if len(durations) >= 2:
                    import statistics
                    mean_s = statistics.mean(durations)
                    stddev_s = statistics.stdev(durations)
                    return mean_s * 1000.0, stddev_s * 1000.0
                return 100.0, 10.0

            # Run in event loop if one exists, otherwise use default values
            try:
                loop = asyncio.get_running_loop()
                # Can't await in sync context, use defaults
            except RuntimeError:
                # No running loop, can create one
                avg_duration_ms, stddev_ms = asyncio.run(fetch_timing())

        return SimulationStep(
            id=signature,
            label=self.__class__.__name__,
            average_duration_ms=avg_duration_ms,
            duration_stddev_ms=stddev_ms,
            delta=SimulationStepDelta(forward=0.0, strafe=0.0, angular=0.0),
        )
