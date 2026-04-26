import asyncio
import contextvars
import time
from abc import abstractmethod
from typing import Any, Awaitable, Callable, List, Optional, TYPE_CHECKING

from .model import SimulationStep, SimulationStepDelta
from .resource import get_resource_manager
from raccoon.class_name_logger import ClassNameLogger
from raccoon.timing import StepTimingTracker

if TYPE_CHECKING:
    from raccoon.robot.api import GenericRobot

StepAnomalyCallback = Callable[["Step", "GenericRobot"], Awaitable[Any]]

_step_path: contextvars.ContextVar[List[str]] = contextvars.ContextVar(
    "step_path", default=[]
)

_in_background: contextvars.ContextVar[bool] = contextvars.ContextVar(
    "_in_background", default=False
)


class Step(ClassNameLogger):
    """Base async action executed by missions and higher-level step combinators."""

    _composite: bool = False

    def __init__(self) -> None:
        self._skip_timing: bool = False
        self._anomaly_callback: Optional[StepAnomalyCallback] = None

    def required_resources(self) -> frozenset[str]:
        """Return the hardware resources this step requires exclusive access to.

        For leaf steps (drive, motor, servo), return the resources this step
        directly uses.  Composite steps override ``collected_resources``
        instead to include children — ``required_resources`` stays empty for
        composites because they don't touch hardware themselves.
        """
        return frozenset()

    def collected_resources(self) -> frozenset[str]:
        """Return all resources this step *and its children* require.

        Used by ``validate_no_overlap`` for static conflict detection at
        construction time.  Leaf steps don't need to override this — the
        default delegates to ``required_resources``.  Composite steps
        override to union their children's collected resources.
        """
        return self.required_resources()

    def resolve(self) -> "Step":
        """Return the concrete Step that should actually execute.

        Default returns ``self``. ``StepBuilder`` overrides this to call
        ``_build()`` so that fluent-DSL builders are converted into the
        underlying Step at composite-construction time. Composite steps
        call ``resolve()`` on their children before storing them so that
        ``validate_no_overlap`` and ``collected_resources`` see the real
        resource sets, not empty builder placeholders.
        """
        return self

    @staticmethod
    def _push_path(segment: str) -> contextvars.Token[List[str]]:
        path = _step_path.get()
        return _step_path.set([*path, segment])

    async def run_step(self, robot: "GenericRobot") -> None:
        """
        Execute the step with logging and optional timing instrumentation.

        When a per-step ``_anomaly_callback`` is set and a timing baseline
        exists, a background watchdog fires the callback as soon as the
        elapsed time exceeds the anomaly upper bound — even while the step
        is still running.
        """
        if not self._composite:
            path = _step_path.get()
            prefix = " > ".join(path) + ": " if path else ""
            self.info(f"{prefix}{self._generate_signature()}")
        else:
            self.debug(f"Executing {self.__class__.__name__} step")

        signature: Optional[str] = None
        start_time: Optional[float] = None

        tracker = StepTimingTracker.get_instance()
        track_timing = tracker.config.enabled and not self._skip_timing
        if track_timing:
            signature = self._generate_signature()
            start_time = time.perf_counter()

        # Launch a live watchdog if there is a per-step anomaly callback.
        watchdog_task: Optional[asyncio.Task[None]] = None
        if track_timing and self._anomaly_callback and signature is not None:
            upper = await tracker.get_upper_bound(signature)
            if upper is not None and start_time is not None:
                watchdog_task = asyncio.create_task(
                    self._anomaly_watchdog(upper, start_time, robot)
                )

        # Runtime resource guard (safety net for Defer / Run)
        resources = self.required_resources()

        # Preempt conflicting background steps before acquiring
        if resources and not _in_background.get(False):
            from .background_manager import get_background_manager

            bg_mgr = get_background_manager(robot)
            if bg_mgr.active_count > 0:
                await bg_mgr.preempt_conflicts(
                    resources, self._generate_signature()
                )

        mgr = get_resource_manager(robot) if resources else None
        if mgr is not None:
            mgr.acquire(resources, self._generate_signature())

        step_start = time.perf_counter()
        try:
            await self._execute_step(robot)
        finally:
            if mgr is not None:
                mgr.release(resources)

            # If the enclosing task is being cancelled, skip any further awaits
            # in this finally block. A new await here would observe the pending
            # cancel and, if caught, consume it — silently defeating the outer
            # cancellation (see shutdown-timer regression).
            current_task = asyncio.current_task()
            outer_cancelling = (
                current_task is not None and current_task.cancelling() > 0
            )

            if watchdog_task is not None:
                watchdog_task.cancel()
                if not outer_cancelling:
                    try:
                        await watchdog_task
                    except asyncio.CancelledError:
                        pass

            elapsed = time.perf_counter() - step_start
            self.debug(f"Finished {self.__class__.__name__} step in {elapsed:.3f}s")

            if (
                track_timing
                and signature is not None
                and start_time is not None
                and not outer_cancelling
            ):
                duration = time.perf_counter() - start_time
                try:
                    anomaly = await tracker.record_execution(signature, duration)
                    if anomaly and self._anomaly_callback:
                        try:
                            await self._anomaly_callback(self, robot)
                        except Exception as exc:
                            self.error(f"Step anomaly callback error: {exc}")
                except asyncio.CancelledError:
                    self.debug("Timing recording cancelled - skipping")
                    raise
                except Exception as exc:
                    self.error(f"Failed to record step timing: {exc}")

    async def _anomaly_watchdog(
        self,
        upper_bound: float,
        start_time: float,
        robot: "GenericRobot",
    ) -> None:
        """Sleep until upper_bound elapsed, then fire the anomaly callback.

        Runs as a background task alongside ``_execute_step``.  Cancelled
        automatically when the step finishes in time.
        """
        remaining = upper_bound - (time.perf_counter() - start_time)
        if remaining > 0:
            await asyncio.sleep(remaining)
        # Step is still running — fire the callback
        assert self._anomaly_callback is not None
        try:
            await self._anomaly_callback(self, robot)
        except Exception as exc:
            self.error(f"Live anomaly callback error: {exc}")

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
