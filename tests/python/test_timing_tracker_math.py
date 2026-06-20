"""Statistics / anomaly-math tests for ``StepTimingTracker``.

These exercise the pure numeric core of the tracker:

* ``_effective_stddev`` flooring: ``max(stddev, min_relative_stddev*abs(mean))``
  including tiny/zero/negative means.
* z-score ``deviation_sigma = abs(duration - mean) / effective_stddev`` -- this is
  the ABSOLUTE z-score, a non-negative magnitude. It does NOT carry a sign for
  faster-vs-slower; the source uses ``abs(...)`` so a faster-than-mean run still
  reports a POSITIVE sigma. DIRECTION is conveyed separately via the
  ``faster_than_expected`` flag (and the FASTER/SLOWER word derived from it).
* anomaly direction (FASTER vs SLOWER) and the ``duration == mean`` boundary.
* ``get_upper_bound`` (the live watchdog bound).
* the callback-firing path on anomaly (and that callback errors are swallowed).

The async database is replaced with an ``AsyncMock`` returning controlled
duration lists -- no real sqlite is touched. Tests stay synchronous and drive
the async tracker via ``asyncio.run`` so they do not depend on pytest-asyncio.
"""

from __future__ import annotations

import asyncio
import importlib.util
import math
from unittest.mock import AsyncMock

import pytest


def _libstp_available() -> bool:
    return importlib.util.find_spec("raccoon") is not None


requires_libstp = pytest.mark.skipif(
    not _libstp_available(),
    reason="raccoon native module not installed",
)


def _tracker(durations=None, **config_kwargs):
    """Build a tracker whose database is a fully mocked async stub.

    ``durations`` is the list returned by ``fetch_recent_durations``.
    """
    from raccoon.timing.config import TimingConfig
    from raccoon.timing.tracker import StepTimingTracker

    tracker = StepTimingTracker(TimingConfig(**config_kwargs))
    db = AsyncMock()
    db.fetch_recent_durations = AsyncMock(return_value=list(durations or []))
    db.insert_execution = AsyncMock(return_value=None)
    tracker.database = db
    return tracker


# ---------------------------------------------------------------------------
# _effective_stddev: flooring behaviour
# ---------------------------------------------------------------------------


@requires_libstp
class TestEffectiveStddev:
    def test_floor_applies_when_stddev_below_relative_floor(self):
        # mean=2.0, min_relative=0.10 -> floor=0.20; measured 0.05 < floor.
        t = _tracker(min_relative_stddev=0.10)
        assert t._effective_stddev(2.0, 0.05) == pytest.approx(0.20)

    def test_measured_stddev_wins_when_above_floor(self):
        # floor = 0.10*2.0 = 0.20; measured 0.50 dominates.
        t = _tracker(min_relative_stddev=0.10)
        assert t._effective_stddev(2.0, 0.50) == pytest.approx(0.50)

    def test_exactly_at_floor_returns_floor_value(self):
        # measured == floor (0.20); max() returns 0.20 either way.
        t = _tracker(min_relative_stddev=0.10)
        assert t._effective_stddev(2.0, 0.20) == pytest.approx(0.20)

    def test_negative_mean_uses_absolute_value(self):
        # abs(mean) is used, so the floor is positive for negative means.
        t = _tracker(min_relative_stddev=0.25)
        assert t._effective_stddev(-4.0, 0.0) == pytest.approx(1.0)

    def test_zero_mean_floor_collapses_to_measured_stddev(self):
        # floor = 0.10*0 = 0; so the measured stddev wins.
        t = _tracker(min_relative_stddev=0.10)
        assert t._effective_stddev(0.0, 0.3) == pytest.approx(0.3)

    def test_zero_mean_and_zero_stddev_is_zero(self):
        t = _tracker(min_relative_stddev=0.10)
        assert t._effective_stddev(0.0, 0.0) == 0.0

    def test_tiny_mean_yields_tiny_floor(self):
        # floor scales with the (tiny) mean.
        t = _tracker(min_relative_stddev=0.5)
        assert t._effective_stddev(1e-6, 0.0) == pytest.approx(5e-7)

    def test_zero_relative_config_disables_floor(self):
        # With min_relative_stddev=0 the floor is 0 -> raw stddev passes through.
        t = _tracker(min_relative_stddev=0.0)
        assert t._effective_stddev(100.0, 0.0) == 0.0


# ---------------------------------------------------------------------------
# _compute_statistics
# ---------------------------------------------------------------------------


@requires_libstp
class TestComputeStatistics:
    def test_empty_is_none(self):
        assert _tracker()._compute_statistics([]) is None

    def test_single_sample_is_none(self):
        assert _tracker()._compute_statistics([0.5]) is None

    def test_two_samples_compute_full_stats(self):
        stats = _tracker()._compute_statistics([1.0, 3.0])
        assert stats is not None
        assert stats.mean == pytest.approx(2.0)
        # sample stdev of [1,3] = sqrt(2) ~ 1.41421356
        assert stats.stddev == pytest.approx(math.sqrt(2.0))
        assert stats.min == 1.0
        assert stats.max == 3.0
        assert stats.count == 2

    def test_values_match_statistics_module(self):
        import statistics

        data = [0.4, 0.5, 0.6, 0.5, 0.45]
        stats = _tracker()._compute_statistics(data)
        assert stats.mean == pytest.approx(statistics.mean(data))
        assert stats.stddev == pytest.approx(statistics.stdev(data))
        assert stats.count == 5


# ---------------------------------------------------------------------------
# _detect_anomaly: z-score, direction, and the duration==mean boundary
# ---------------------------------------------------------------------------


@requires_libstp
class TestDetectAnomalyMath:
    def test_none_stats_never_anomaly(self):
        assert _tracker()._detect_anomaly("S()", 999.0, None) is None

    def test_deviation_sigma_matches_zscore(self):
        # baseline [1,3] -> mean 2, stddev sqrt(2)~1.414; floor=0.1*2=0.2 < stddev
        # so effective_stddev = sqrt(2). duration 7 -> (7-2)/1.414 = 3.5355 sigma.
        t = _tracker(threshold_multiplier=3.0, min_relative_stddev=0.10)
        stats = t._compute_statistics([1.0, 3.0])
        anomaly = t._detect_anomaly("S()", 7.0, stats)
        assert anomaly is not None
        assert anomaly.deviation_sigma == pytest.approx(5.0 / math.sqrt(2.0))
        assert anomaly.faster_than_expected is False

    def test_faster_anomaly_reports_positive_absolute_sigma(self):
        # deviation_sigma is the ABSOLUTE z-score abs(duration-mean)/eff_stddev,
        # so a faster-than-mean outlier reports a POSITIVE magnitude -- the sign
        # is NOT used to encode direction (that lives in faster_than_expected).
        # baseline [5,5] -> mean 5, stddev 0, floor 0.1*5=0.5; threshold 3.
        # duration 1.0 is below lower bound (5-1.5=3.5) -> FASTER anomaly.
        # sigma = abs(1.0-5.0)/0.5 = 8.0 (positive, == the abs value).
        t = _tracker(threshold_multiplier=3.0, min_relative_stddev=0.10)
        stats = t._compute_statistics([5.0, 5.0])
        anomaly = t._detect_anomaly("S()", 1.0, stats)
        assert anomaly is not None
        # (a) sigma is positive (the absolute z-score), NOT a negative signed value.
        assert anomaly.deviation_sigma > 0
        assert anomaly.deviation_sigma == pytest.approx(8.0)
        assert anomaly.deviation_sigma == pytest.approx(abs(1.0 - 5.0) / 0.5)
        # (b) direction is conveyed via the flag, not the sign of sigma.
        assert anomaly.faster_than_expected is True

    def test_slower_outlier_just_past_upper_bound(self):
        # mean 2, eff_stddev sqrt(2), threshold 3 -> upper = 2 + 3*1.414 = 6.2426.
        t = _tracker(threshold_multiplier=3.0, min_relative_stddev=0.10)
        stats = t._compute_statistics([1.0, 3.0])
        upper = 2.0 + 3.0 * math.sqrt(2.0)
        # Just below the bound -> not an anomaly.
        assert t._detect_anomaly("S()", upper - 1e-9, stats) is None
        # Just above the bound -> SLOWER anomaly.
        a = t._detect_anomaly("S()", upper + 1e-6, stats)
        assert a is not None
        assert a.faster_than_expected is False

    def test_faster_outlier_just_past_lower_bound(self):
        t = _tracker(threshold_multiplier=3.0, min_relative_stddev=0.10)
        stats = t._compute_statistics([1.0, 3.0])
        lower = 2.0 - 3.0 * math.sqrt(2.0)
        # Just above lower bound -> normal.
        assert t._detect_anomaly("S()", lower + 1e-9, stats) is None
        # Just below lower bound -> FASTER anomaly.
        a = t._detect_anomaly("S()", lower - 1e-6, stats)
        assert a is not None
        assert a.faster_than_expected is True

    def test_anomaly_fields_are_populated(self):
        t = _tracker(threshold_multiplier=2.0, min_relative_stddev=0.10)
        stats = t._compute_statistics([1.0, 3.0])
        a = t._detect_anomaly("DriveForward(25.00cm)", 100.0, stats)
        assert a is not None
        assert a.signature == "DriveForward(25.00cm)"
        assert a.duration == 100.0
        assert a.expected_mean == pytest.approx(2.0)
        # expected_stddev is the *raw* measured stddev, not the floored one.
        assert a.expected_stddev == pytest.approx(math.sqrt(2.0))
        assert a.threshold_multiplier == 2.0
        assert a.timestamp > 0

    # --- degenerate baseline (effective_stddev == 0) ---

    def test_degenerate_baseline_exact_match_is_normal(self):
        # mean=0, stddev=0, floor=0 -> effective_stddev==0; duration==mean.
        t = _tracker(min_relative_stddev=0.10)
        stats = t._compute_statistics([0.0, 0.0, 0.0])
        assert stats.mean == 0.0
        assert t._effective_stddev(stats.mean, stats.stddev) == 0.0
        assert t._detect_anomaly("S()", 0.0, stats) is None

    def test_degenerate_baseline_any_deviation_is_infinite_sigma(self):
        t = _tracker(min_relative_stddev=0.10)
        stats = t._compute_statistics([0.0, 0.0, 0.0])
        a = t._detect_anomaly("S()", 0.001, stats)
        assert a is not None
        assert a.deviation_sigma == float("inf")
        assert a.faster_than_expected is False  # 0.001 > 0 -> slower side

    def test_degenerate_baseline_negative_duration_is_faster(self):
        t = _tracker(min_relative_stddev=0.10)
        stats = t._compute_statistics([0.0, 0.0])
        a = t._detect_anomaly("S()", -0.001, stats)
        assert a is not None
        assert a.deviation_sigma == float("inf")
        assert a.faster_than_expected is True  # below lower_bound (==mean==0)

    def test_zero_variance_nonzero_mean_uses_relative_floor(self):
        # Identical 0.5s runs -> stddev 0, but mean 0.5 -> floor 0.05.
        # A 10% jitter (0.55) stays inside mean + 3*0.05 = 0.65 -> NOT an anomaly.
        t = _tracker(threshold_multiplier=3.0, min_relative_stddev=0.10)
        stats = t._compute_statistics([0.5, 0.5, 0.5, 0.5])
        assert stats.stddev == 0.0
        assert t._effective_stddev(0.5, 0.0) == pytest.approx(0.05)
        assert t._detect_anomaly("S()", 0.55, stats) is None
        # But a gross outlier (0.9 > 0.65) is still flagged.
        a = t._detect_anomaly("S()", 0.9, stats)
        assert a is not None
        # deviation_sigma = (0.9-0.5)/0.05 = 8.0
        assert a.deviation_sigma == pytest.approx(8.0)


# ---------------------------------------------------------------------------
# get_upper_bound
# ---------------------------------------------------------------------------


@requires_libstp
class TestGetUpperBound:
    def test_no_baseline_returns_none(self):
        t = _tracker([0.5])  # single sample -> stats None
        assert asyncio.run(t.get_upper_bound("S()")) is None

    def test_empty_returns_none(self):
        t = _tracker([])
        assert asyncio.run(t.get_upper_bound("S()")) is None

    def test_bound_uses_floored_stddev(self):
        # Identical samples -> stddev 0, mean 1.0; floor 0.10 -> bound 1.3.
        t = _tracker([1.0, 1.0, 1.0], threshold_multiplier=3.0, min_relative_stddev=0.10)
        bound = asyncio.run(t.get_upper_bound("S()"))
        assert bound == pytest.approx(1.0 + 3.0 * 0.10)

    def test_bound_uses_measured_stddev_when_larger(self):
        # baseline [1,3]: mean 2, stddev sqrt(2)>floor 0.2 -> bound 2+3*sqrt(2).
        t = _tracker([1.0, 3.0], threshold_multiplier=3.0, min_relative_stddev=0.10)
        bound = asyncio.run(t.get_upper_bound("S()"))
        assert bound == pytest.approx(2.0 + 3.0 * math.sqrt(2.0))

    def test_threshold_multiplier_scales_bound(self):
        t = _tracker([1.0, 1.0], threshold_multiplier=5.0, min_relative_stddev=0.10)
        bound = asyncio.run(t.get_upper_bound("S()"))
        assert bound == pytest.approx(1.0 + 5.0 * 0.10)

    def test_passes_window_size_to_database(self):
        t = _tracker([1.0, 1.0], window_size=7)
        asyncio.run(t.get_upper_bound("Sig()"))
        t.database.fetch_recent_durations.assert_awaited_once_with("Sig()", 7)


# ---------------------------------------------------------------------------
# record_execution: end-to-end with mocked DB + callback path
# ---------------------------------------------------------------------------


@requires_libstp
class TestRecordExecution:
    def test_disabled_short_circuits_without_db_access(self):
        t = _tracker([1.0, 3.0], enabled=False)
        result = asyncio.run(t.record_execution("S()", 999.0))
        assert result is None
        t.database.fetch_recent_durations.assert_not_awaited()
        t.database.insert_execution.assert_not_awaited()

    def test_normal_run_returns_none_and_still_inserts(self):
        t = _tracker([2.0, 2.0], threshold_multiplier=3.0, min_relative_stddev=0.10)
        result = asyncio.run(t.record_execution("S()", 2.1))
        assert result is None
        # The execution is always persisted (anomaly=None here).
        t.database.insert_execution.assert_awaited_once_with("S()", 2.1, None)

    def test_anomaly_returned_and_persisted(self):
        t = _tracker([1.0, 3.0], threshold_multiplier=2.0, min_relative_stddev=0.10)
        result = asyncio.run(t.record_execution("S()", 50.0))
        assert result is not None
        assert result.faster_than_expected is False
        # insert_execution receives the same anomaly object.
        args = t.database.insert_execution.await_args.args
        assert args[0] == "S()"
        assert args[1] == 50.0
        assert args[2] is result

    def test_anomaly_fires_registered_callback(self):
        t = _tracker([1.0, 3.0], threshold_multiplier=2.0, min_relative_stddev=0.10)
        seen = []

        async def cb(anomaly):
            seen.append(anomaly)

        t.register_anomaly_callback(cb)
        result = asyncio.run(t.record_execution("S()", 50.0))
        assert len(seen) == 1
        assert seen[0] is result

    def test_normal_run_does_not_fire_callback(self):
        t = _tracker([2.0, 2.0], threshold_multiplier=3.0, min_relative_stddev=0.10)
        seen = []

        async def cb(anomaly):
            seen.append(anomaly)

        t.register_anomaly_callback(cb)
        asyncio.run(t.record_execution("S()", 2.05))
        assert seen == []

    def test_faster_anomaly_callback_receives_faster_flag(self):
        t = _tracker([5.0, 5.0], threshold_multiplier=2.0, min_relative_stddev=0.10)
        seen = []

        async def cb(anomaly):
            seen.append(anomaly)

        t.register_anomaly_callback(cb)
        asyncio.run(t.record_execution("S()", 0.01))
        assert len(seen) == 1
        assert seen[0].faster_than_expected is True

    def test_callback_exception_is_swallowed(self):
        # A raising callback must NOT propagate or block a second callback.
        t = _tracker([1.0, 3.0], threshold_multiplier=2.0, min_relative_stddev=0.10)
        order = []

        async def bad(anomaly):
            order.append("bad")
            raise RuntimeError("boom")

        async def good(anomaly):
            order.append("good")

        t.register_anomaly_callback(bad)
        t.register_anomaly_callback(good)
        result = asyncio.run(t.record_execution("S()", 50.0))
        assert result is not None  # record_execution still succeeds
        assert order == ["bad", "good"]  # both ran despite the exception

    def test_no_baseline_first_runs_not_anomalous(self):
        # fetch returns < 2 samples -> stats None -> never anomalous.
        t = _tracker([0.5], threshold_multiplier=2.0)
        result = asyncio.run(t.record_execution("S()", 99.0))
        assert result is None
        t.database.insert_execution.assert_awaited_once_with("S()", 99.0, None)


# ---------------------------------------------------------------------------
# Misc: callback registration plumbing
# ---------------------------------------------------------------------------


@requires_libstp
class TestRegistration:
    def test_register_appends_callback(self):
        t = _tracker()

        async def cb(_):
            pass

        assert t.anomaly_callbacks == []
        t.register_anomaly_callback(cb)
        assert t.anomaly_callbacks == [cb]


# ---------------------------------------------------------------------------
# Singleton + configure plumbing
# ---------------------------------------------------------------------------


@requires_libstp
class TestSingletonAndConfigure:
    def test_get_instance_is_idempotent(self):
        from raccoon.timing.tracker import StepTimingTracker

        StepTimingTracker._instance = None
        try:
            first = StepTimingTracker.get_instance()
            second = StepTimingTracker.get_instance()
            assert first is second
        finally:
            StepTimingTracker._instance = None

    def test_get_instance_creates_real_tracker_from_default_none(self):
        # The class-level default of _instance must be None and the lazy
        # creation must produce a real StepTimingTracker (not the sentinel,
        # not "", not left as None).
        from raccoon.timing.tracker import StepTimingTracker

        # Reset to the documented cold-start state.
        StepTimingTracker._instance = None
        try:
            inst = StepTimingTracker.get_instance()
            assert isinstance(inst, StepTimingTracker)
            assert inst is not None
            # The default placeholder must be None, not a truthy "" string:
            # if it were "" the `is None` check fails and get_instance would
            # return "" instead of building a tracker.
            assert StepTimingTracker._instance is inst
        finally:
            StepTimingTracker._instance = None

    def test_get_instance_does_not_replace_existing(self):
        # When an instance already exists, get_instance must NOT overwrite it
        # (kills the `is not None` and `= None` body mutants).
        from raccoon.timing.config import TimingConfig
        from raccoon.timing.tracker import StepTimingTracker

        StepTimingTracker._instance = None
        try:
            sentinel = StepTimingTracker(TimingConfig(db_path=":memory:"))
            StepTimingTracker._instance = sentinel
            got = StepTimingTracker.get_instance()
            assert got is sentinel
            assert got is not None
        finally:
            StepTimingTracker._instance = None

    def test_init_creates_real_database(self):
        # __init__ must build a real StepTimingDatabase, not None.
        from raccoon.timing.config import TimingConfig
        from raccoon.timing.database import StepTimingDatabase
        from raccoon.timing.tracker import StepTimingTracker

        t = StepTimingTracker(TimingConfig(db_path=":memory:"))
        assert isinstance(t.database, StepTimingDatabase)
        assert t.database.db_path == ":memory:"

    def test_configure_replaces_config_and_database(self):
        from raccoon.timing.config import TimingConfig
        from raccoon.timing.database import StepTimingDatabase
        from raccoon.timing.tracker import StepTimingTracker

        t = StepTimingTracker(TimingConfig(db_path=":memory:", window_size=5))
        old_db = t.database
        new_config = TimingConfig(db_path="/tmp/new_timing.db", window_size=99)
        t.configure(new_config)
        assert t.config is new_config
        assert t.config.window_size == 99
        # A fresh *real* database object is created for the new config and it
        # picks up the new db_path (kills the `database = None` mutant).
        assert t.database is not old_db
        assert isinstance(t.database, StepTimingDatabase)
        assert t.database.db_path == "/tmp/new_timing.db"


# ---------------------------------------------------------------------------
# Exact boundary behaviour of _detect_anomaly (kills < vs <= / > vs >=)
# ---------------------------------------------------------------------------


@requires_libstp
class TestDetectAnomalyExactBounds:
    def _bounds_tracker(self):
        # baseline [1,3] -> mean 2, raw stddev sqrt(2); floor 0.1*2=0.2 < sqrt(2)
        # so effective_stddev = sqrt(2); threshold 3.
        t = _tracker(threshold_multiplier=3.0, min_relative_stddev=0.10)
        stats = t._compute_statistics([1.0, 3.0])
        return t, stats

    def test_duration_exactly_at_upper_bound_is_normal(self):
        # duration == upper_bound must NOT be flagged (kills `> -> >=`).
        t, stats = self._bounds_tracker()
        upper = 2.0 + 3.0 * math.sqrt(2.0)
        assert t._detect_anomaly("S()", upper, stats) is None

    def test_duration_exactly_at_lower_bound_is_normal(self):
        # duration == lower_bound must NOT be flagged (kills `< -> <=`).
        t, stats = self._bounds_tracker()
        lower = 2.0 - 3.0 * math.sqrt(2.0)
        assert t._detect_anomaly("S()", lower, stats) is None

    def test_just_above_upper_bound_is_anomaly(self):
        t, stats = self._bounds_tracker()
        upper = 2.0 + 3.0 * math.sqrt(2.0)
        a = t._detect_anomaly("S()", math.nextafter(upper, math.inf), stats)
        assert a is not None
        assert a.faster_than_expected is False

    def test_just_below_lower_bound_is_faster_anomaly(self):
        t, stats = self._bounds_tracker()
        lower = 2.0 - 3.0 * math.sqrt(2.0)
        a = t._detect_anomaly("S()", math.nextafter(lower, -math.inf), stats)
        assert a is not None
        assert a.faster_than_expected is True


# ---------------------------------------------------------------------------
# _log_baseline_progress: cold-start log message (kills arithmetic / bound /
# string mutations on the baseline-progress branch)
# ---------------------------------------------------------------------------


@requires_libstp
class TestBaselineProgressLog:
    def _capture(self, t):
        msgs: list[str] = []
        t.debug = msgs.append  # type: ignore[assignment]
        return msgs

    def test_logs_1_of_2_after_first_sample(self):
        # prior_samples=0 -> recorded = 0+1 = 1 -> logs "(1/2)".
        t = _tracker()
        msgs = self._capture(t)
        t._log_baseline_progress("Sig()", 0)
        assert msgs == ["Establishing timing baseline for Sig() (1/2)"]

    def test_logs_2_of_2_after_second_sample(self):
        # prior_samples=1 -> recorded=2 -> still logs (boundary recorded<=2).
        t = _tracker()
        msgs = self._capture(t)
        t._log_baseline_progress("Sig()", 1)
        assert msgs == ["Establishing timing baseline for Sig() (2/2)"]

    def test_no_log_once_baseline_established(self):
        # prior_samples=2 -> recorded=3 > 2 -> nothing logged.
        t = _tracker()
        msgs = self._capture(t)
        t._log_baseline_progress("Sig()", 2)
        assert msgs == []

    def test_no_log_for_large_sample_count(self):
        t = _tracker()
        msgs = self._capture(t)
        t._log_baseline_progress("Sig()", 50)
        assert msgs == []


# ---------------------------------------------------------------------------
# Anomaly debug message content + direction word (kills the f-string mutants)
# ---------------------------------------------------------------------------


@requires_libstp
class TestAnomalyDebugMessage:
    def _run(self, durations, duration, **cfg):
        t = _tracker(durations, **cfg)
        msgs: list[str] = []
        t.debug = msgs.append  # type: ignore[assignment]
        result = asyncio.run(t.record_execution("DriveForward(25.00cm)", duration))
        return result, msgs

    def test_slower_anomaly_message_is_exact(self):
        # baseline [2,2] -> mean 2, stddev 0, floor 0.1*2=0.2; threshold 3.
        # duration 5 -> sigma = (5-2)/0.2 = 15.0; SLOWER.
        result, msgs = self._run(
            [2.0, 2.0], 5.0, threshold_multiplier=3.0, min_relative_stddev=0.10
        )
        assert result is not None and result.faster_than_expected is False
        assert len(msgs) == 1
        expected = (
            "Timing anomaly for DriveForward(25.00cm): 5.000s "
            "(expected 2.000s +/- 3.0*0.000s, 15.0 sigma) SLOWER"
        )
        assert msgs[0] == expected

    def test_faster_anomaly_message_says_faster(self):
        # baseline [5,5] -> mean 5, stddev 0, floor 0.5; threshold 3.
        # duration 0.5 -> below lower bound (5-1.5=3.5) -> FASTER.
        result, msgs = self._run(
            [5.0, 5.0], 0.5, threshold_multiplier=3.0, min_relative_stddev=0.10
        )
        assert result is not None and result.faster_than_expected is True
        assert len(msgs) == 1
        assert msgs[0].endswith(" FASTER")
        assert "Timing anomaly for DriveForward(25.00cm): 0.500s" in msgs[0]
        assert "SLOWER" not in msgs[0]


# ---------------------------------------------------------------------------
# Callback error logging (kills the error() f-string mutant)
# ---------------------------------------------------------------------------


@requires_libstp
class TestCallbackErrorLogging:
    def test_callback_error_message_is_logged_verbatim(self):
        from raccoon.timing.models import AnomalyDetection

        t = _tracker()
        errs: list[str] = []
        t.error = errs.append  # type: ignore[assignment]

        async def bad(_):
            raise RuntimeError("boom")

        t.register_anomaly_callback(bad)
        anomaly = AnomalyDetection(
            signature="S()",
            duration=1.0,
            expected_mean=1.0,
            expected_stddev=0.0,
            threshold_multiplier=3.0,
            deviation_sigma=1.0,
            faster_than_expected=False,
            timestamp=0.0,
        )
        asyncio.run(t._fire_callbacks(anomaly))
        assert errs == ["Anomaly callback error (ignored): boom"]
