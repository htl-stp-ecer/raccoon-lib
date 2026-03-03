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
    """Base async action executed by missions and higher-level step combinators."""

    def __init__(self) -> None:
        pass

    async def run_step(self, robot: "GenericRobot") -> None:
        """
        Execute the step with logging and optional timing instrumentation.
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
        Return the timing/simulation identity for this step instance.

        Override in subclasses when constructor parameters materially change
        runtime behavior and should not share the same timing baseline.
        """
        return self.__class__.__name__

    @abstractmethod
    async def _execute_step(self, robot: "GenericRobot") -> None:
        """Actual step logic implemented by subclasses."""
        raise NotImplementedError

    def to_simulation_step(self) -> SimulationStep:
        """
        Convert this step to a simulation-friendly summary.

        The default implementation uses timing history only when it can query
        the tracker synchronously; otherwise it returns conservative defaults.
        Override in subclasses that know their motion delta or exact duration.
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
