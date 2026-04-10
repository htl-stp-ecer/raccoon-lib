import asyncio
import statistics
import time
from typing import Awaitable, Callable, List, Optional

from raccoon.class_name_logger import ClassNameLogger

from .config import TimingConfig
from .database import StepTimingDatabase
from .models import AnomalyDetection, StepStatistics

AnomalyCallback = Callable[[AnomalyDetection], Awaitable[None]]


class StepTimingTracker(ClassNameLogger):
    """Singleton tracker that persists step runtimes and reports anomalies."""

    _instance: Optional["StepTimingTracker"] = None

    def __init__(self, config: Optional[TimingConfig] = None) -> None:
        """Initialize tracker state and open the configured timing database lazily."""
        self.config = config or TimingConfig()
        self.database = StepTimingDatabase(self.config.db_path)
        self.anomaly_callbacks: List[AnomalyCallback] = []
        self._lock = asyncio.Lock()

    @classmethod
    def get_instance(cls) -> "StepTimingTracker":
        """Return the process-wide tracker instance used by ``Step.run_step``."""
        if cls._instance is None:
            cls._instance = cls()
        return cls._instance

    def configure(self, config: TimingConfig) -> None:
        """Replace configuration at runtime."""
        self.config = config
        self.database = StepTimingDatabase(config.db_path)

    def register_anomaly_callback(self, callback: AnomalyCallback) -> None:
        """Register an async callback to run when an anomaly is detected."""
        self.anomaly_callbacks.append(callback)

    async def record_execution(
        self, signature: str, duration: float
    ) -> Optional[AnomalyDetection]:
        """Persist execution and evaluate for anomalies.

        Returns:
            The detected anomaly, or ``None`` if the execution was normal.
        """
        if not self.config.enabled:
            return None

        async with self._lock:
            durations = await self.database.fetch_recent_durations(
                signature, self.config.window_size
            )
            stats = self._compute_statistics(durations)
            anomaly = self._detect_anomaly(signature, duration, stats)

            if anomaly:
                direction = "FASTER" if anomaly.faster_than_expected else "SLOWER"
                self.debug(
                    (
                        f"Timing anomaly for {signature}: {duration:.3f}s "
                        f"(expected {anomaly.expected_mean:.3f}s +/- "
                        f"{anomaly.threshold_multiplier:.1f}*"
                        f"{anomaly.expected_stddev:.3f}s, "
                        f"{anomaly.deviation_sigma:.1f} sigma) {direction}"
                    )
                )
                await self._fire_callbacks(anomaly)
            else:
                self._log_baseline_progress(signature, len(durations))

            await self.database.insert_execution(signature, duration, anomaly)
            return anomaly

    async def get_upper_bound(self, signature: str) -> Optional[float]:
        """Return the anomaly upper bound for a signature, or ``None`` if no baseline.

        This is the duration above which an execution would be flagged as
        SLOWER than expected.  Used by the live watchdog in ``Step.run_step``.
        """
        durations = await self.database.fetch_recent_durations(
            signature, self.config.window_size
        )
        stats = self._compute_statistics(durations)
        if stats is None:
            return None
        return stats.mean + self.config.threshold_multiplier * stats.stddev

    def _log_baseline_progress(self, signature: str, prior_samples: int) -> None:
        """Emit cold-start info while collecting the first few samples."""
        recorded = prior_samples + 1  # include the sample being recorded
        if recorded <= 2:
            self.debug(f"Establishing timing baseline for {signature} ({recorded}/2)")

    def _detect_anomaly(
        self,
        signature: str,
        duration: float,
        stats: Optional[StepStatistics],
    ) -> Optional[AnomalyDetection]:
        """Return anomaly record if duration exceeds mean +/- threshold * stddev."""
        if stats is None:
            return None

        threshold = self.config.threshold_multiplier

        if stats.stddev == 0:
            if duration == stats.mean:
                return None
            deviation_sigma = float("inf")
        else:
            deviation_sigma = abs(duration - stats.mean) / stats.stddev

        lower_bound = stats.mean - (threshold * stats.stddev)
        upper_bound = stats.mean + (threshold * stats.stddev)

        if duration < lower_bound or duration > upper_bound:
            return AnomalyDetection(
                signature=signature,
                duration=duration,
                expected_mean=stats.mean,
                expected_stddev=stats.stddev,
                threshold_multiplier=threshold,
                deviation_sigma=deviation_sigma,
                faster_than_expected=duration < lower_bound,
                timestamp=time.time(),
            )
        return None

    def _compute_statistics(self, durations: List[float]) -> Optional[StepStatistics]:
        """Compute rolling statistics from prior samples."""
        if len(durations) < 2:
            return None

        return StepStatistics(
            mean=statistics.mean(durations),
            stddev=statistics.stdev(durations),
            min=min(durations),
            max=max(durations),
            count=len(durations),
        )

    async def _fire_callbacks(self, anomaly: AnomalyDetection) -> None:
        """Invoke anomaly callbacks without interrupting mission execution."""
        for callback in list(self.anomaly_callbacks):
            try:
                await callback(anomaly)
            except Exception as exc:
                self.error(f"Anomaly callback error (ignored): {exc}")
