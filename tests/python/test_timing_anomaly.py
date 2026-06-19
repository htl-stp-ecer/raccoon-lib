"""Tests for step timing anomaly detection (the engine behind ``.on_anomaly()``).

These cover the reported bug where ``.on_anomaly()`` fires even though the step
ran normally: when a step's runtime is very consistent the rolling ``stddev``
collapses toward zero, so the ``mean +/- threshold * stddev`` band becomes
razor-thin and ordinary run-to-run jitter trips the detector.

Tests stay synchronous and drive the async tracker via ``asyncio.run`` so they
do not depend on pytest-asyncio (which the raccoon test plugin deliberately
avoids).
"""

from __future__ import annotations

import asyncio
import importlib.util
from pathlib import Path
from unittest.mock import MagicMock

import pytest


def _libstp_available() -> bool:
    return importlib.util.find_spec("raccoon") is not None


requires_libstp = pytest.mark.skipif(
    not _libstp_available(),
    reason="raccoon native module not installed",
)


def _tracker(db_path: str | None = None):
    from raccoon.timing.config import TimingConfig
    from raccoon.timing.tracker import StepTimingTracker

    config = TimingConfig(db_path=db_path) if db_path is not None else TimingConfig()
    return StepTimingTracker(config)


# ---------------------------------------------------------------------------
# _detect_anomaly: tight baselines must tolerate normal jitter
# ---------------------------------------------------------------------------


@requires_libstp
class TestDetectAnomalyTightBaseline:
    def test_identical_baseline_does_not_flag_small_jitter(self):
        """Four identical 0.5s runs then a 0.55s run (10% jitter) is NOT an anomaly.

        With ``stddev == 0`` the old code treated any deviation as infinite
        sigma and flagged it -- the exact false positive the user hit.
        """
        tracker = _tracker()
        stats = tracker._compute_statistics([0.50, 0.50, 0.50, 0.50])

        assert tracker._detect_anomaly("MyStep()", 0.55, stats) is None

    def test_low_variance_baseline_does_not_flag_normal_run(self):
        """A baseline with tiny (but non-zero) variance tolerates a normal run."""
        tracker = _tracker()
        stats = tracker._compute_statistics([0.50, 0.51, 0.50, 0.51, 0.50])

        # ~9% slower than the mean -- normal jitter, not an anomaly.
        assert tracker._detect_anomaly("MyStep()", 0.55, stats) is None

    def test_genuine_outlier_is_still_detected(self):
        """A 10x-slower run must still be flagged after the tolerance fix."""
        tracker = _tracker()
        stats = tracker._compute_statistics([0.50, 0.50, 0.50, 0.50])

        anomaly = tracker._detect_anomaly("MyStep()", 5.0, stats)

        assert anomaly is not None
        assert anomaly.faster_than_expected is False

    def test_genuine_fast_outlier_is_still_detected(self):
        """A run far below the baseline is still flagged as FASTER."""
        tracker = _tracker()
        stats = tracker._compute_statistics([1.00, 1.00, 1.00, 1.00])

        anomaly = tracker._detect_anomaly("MyStep()", 0.01, stats)

        assert anomaly is not None
        assert anomaly.faster_than_expected is True


# ---------------------------------------------------------------------------
# Edge cases: NO baseline data present (must never crash or fire)
# ---------------------------------------------------------------------------


@requires_libstp
class TestNoData:
    def test_compute_statistics_empty_is_none(self):
        assert _tracker()._compute_statistics([]) is None

    def test_compute_statistics_single_sample_is_none(self):
        # One sample is not enough to compute a stddev -> no baseline yet.
        assert _tracker()._compute_statistics([0.5]) is None

    def test_detect_anomaly_with_no_stats_is_none(self):
        # No baseline -> can never be an anomaly, regardless of duration.
        tracker = _tracker()
        assert tracker._detect_anomaly("MyStep()", 0.0, None) is None
        assert tracker._detect_anomaly("MyStep()", 999.0, None) is None

    def test_get_upper_bound_with_no_rows_is_none(self, tmp_path: Path):
        tracker = _tracker(str(tmp_path / "timing.db"))
        assert asyncio.run(tracker.get_upper_bound("Never()")) is None

    def test_get_upper_bound_with_single_row_is_none(self, tmp_path: Path):
        tracker = _tracker(str(tmp_path / "timing.db"))

        async def scenario():
            await tracker.database.insert_execution("Sig()", 1.0, None)
            return await tracker.get_upper_bound("Sig()")

        assert asyncio.run(scenario()) is None

    def test_record_execution_first_run_is_not_anomalous(self, tmp_path: Path):
        tracker = _tracker(str(tmp_path / "timing.db"))

        async def scenario():
            # First two runs have no/insufficient baseline -> never anomalous.
            first = await tracker.record_execution("Sig()", 0.5)
            second = await tracker.record_execution("Sig()", 9.9)
            return first, second

        first, second = asyncio.run(scenario())
        assert first is None
        assert second is None


# ---------------------------------------------------------------------------
# get_upper_bound: live-watchdog bound must not collapse onto the mean
# ---------------------------------------------------------------------------


@requires_libstp
class TestGetUpperBoundTightBaseline:
    def test_upper_bound_widened_for_zero_variance(self, tmp_path: Path):
        tracker = _tracker(str(tmp_path / "timing.db"))

        async def scenario():
            # Two identical baseline samples -> mean 1.0, stddev 0.0.
            await tracker.database.insert_execution("Sig()", 1.0, None)
            await tracker.database.insert_execution("Sig()", 1.0, None)
            return await tracker.get_upper_bound("Sig()")

        upper = asyncio.run(scenario())
        assert upper is not None
        # The old code returned exactly the mean (1.0), so the watchdog fired
        # the instant the step exceeded its own average runtime by any margin.
        assert upper > 1.0


# ---------------------------------------------------------------------------
# record_execution end-to-end: faithful reproduction of the report
# ---------------------------------------------------------------------------


@requires_libstp
class TestRecordExecutionEndToEnd:
    def test_normal_run_after_stable_baseline_is_not_anomalous(self, tmp_path: Path):
        tracker = _tracker(str(tmp_path / "timing.db"))

        async def scenario():
            sig = "DriveForward(25.00cm)"
            # Establish a stable baseline of identical runs (none anomalous).
            baseline = [await tracker.record_execution(sig, 0.50) for _ in range(4)]
            # One more normal run with mild jitter -- must NOT be flagged.
            jitter = await tracker.record_execution(sig, 0.55)
            return baseline, jitter

        baseline, jitter = asyncio.run(scenario())
        assert all(a is None for a in baseline)
        assert jitter is None


# ---------------------------------------------------------------------------
# run_step: the anomaly callback must fire AT MOST ONCE per execution.
# The live watchdog and the post-execution check are two independent paths;
# a slow step trips both, so the callback was firing twice.
# ---------------------------------------------------------------------------


def _seed_singleton_tracker(db_path: str):
    """Install a freshly-configured tracker as the process-wide singleton."""
    from raccoon.timing.config import TimingConfig
    from raccoon.timing.tracker import StepTimingTracker

    tracker = StepTimingTracker(TimingConfig(db_path=db_path))
    StepTimingTracker._instance = tracker
    return tracker


def _clear_singleton_tracker():
    from raccoon.timing.tracker import StepTimingTracker

    StepTimingTracker._instance = None


@requires_libstp
class TestAnomalyCallbackFiresOnce:
    def _sleep_step(self, sleep_for: float, calls: list[int]):
        from raccoon.step.base import Step

        async def _callback(step, robot):
            calls.append(1)

        class SleepStep(Step):
            def _generate_signature(self) -> str:
                return "SleepStep()"

            async def _execute_step(self, robot) -> None:
                await asyncio.sleep(sleep_for)

        step = SleepStep()
        step._anomaly_callback = _callback
        return step

    def test_slow_step_fires_callback_exactly_once(self, tmp_path: Path):
        """A slow step trips the live watchdog AND post-execution detection.

        Before the fix both paths fired, so the callback ran twice.
        """
        calls: list[int] = []

        async def scenario():
            tracker = _seed_singleton_tracker(str(tmp_path / "timing.db"))
            # Tight, fast baseline -> upper bound ~0.013s.
            for _ in range(4):
                await tracker.database.insert_execution("SleepStep()", 0.01, None)
            step = self._sleep_step(0.2, calls)
            await step.run_step(MagicMock())

        try:
            asyncio.run(scenario())
        finally:
            _clear_singleton_tracker()

        assert len(calls) == 1

    def test_fast_anomaly_fires_callback_exactly_once(self, tmp_path: Path):
        """A far-too-fast run is a (post-execution) anomaly and fires once.

        The watchdog only watches the upper bound, so it must not contribute
        a second call here.
        """
        calls: list[int] = []

        async def scenario():
            tracker = _seed_singleton_tracker(str(tmp_path / "timing.db"))
            # Slow baseline (~0.2s) so a near-instant run is FASTER than expected.
            for _ in range(4):
                await tracker.database.insert_execution("SleepStep()", 0.20, None)
            step = self._sleep_step(0.0, calls)
            await step.run_step(MagicMock())

        try:
            asyncio.run(scenario())
        finally:
            _clear_singleton_tracker()

        assert len(calls) == 1

    def test_normal_step_does_not_fire_callback(self, tmp_path: Path):
        """A run within the baseline band fires neither path."""
        calls: list[int] = []

        async def scenario():
            tracker = _seed_singleton_tracker(str(tmp_path / "timing.db"))
            for _ in range(4):
                await tracker.database.insert_execution("SleepStep()", 0.05, None)
            step = self._sleep_step(0.05, calls)
            await step.run_step(MagicMock())

        try:
            asyncio.run(scenario())
        finally:
            _clear_singleton_tracker()

        assert calls == []
