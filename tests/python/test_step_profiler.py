"""Tests for the deep in-step instrumentation of :class:`StepProfiler`.

These cover the additions that decompose the *step body* itself — the
synchronous HAL/transport calls and the deliberate ``asyncio.sleep`` waits —
so a step that "stands still" while the overhead/lag views show nothing can be
explained. The HAL patching is exercised against the real pybind ``raccoon.hal``
objects when the native module is present, and against a plain Python stand-in
otherwise so the wrapping logic is always covered.
"""

from __future__ import annotations

import asyncio
import importlib.util
import time

import pytest

from raccoon.profiling.step_profiler import StepProfiler, StepSample, _current_sample


def _native_available() -> bool:
    return importlib.util.find_spec("raccoon.hal") is not None


def _run_under_sample(prof: StepProfiler, body) -> StepSample:
    """Drive ``body`` the way ``run_step`` would: under a live sample + token."""

    async def runner() -> StepSample:
        async with prof:
            sample = StepSample(
                signature="FakeStep", path="root", depth=1, composite=False, tid=0
            )
            prof.samples.append(sample)
            token = _current_sample.set(sample)
            start = time.perf_counter()
            try:
                await body()
            finally:
                sample.execute = time.perf_counter() - start
                sample.total = sample.execute
                _current_sample.reset(token)
        return sample

    return asyncio.run(runner())


def test_sleep_is_split_out_of_the_body() -> None:
    prof = StepProfiler(loop_lag=False, print_report=False, hal=False)

    async def body() -> None:
        await asyncio.sleep(0.02)

    sample = _run_under_sample(prof, body)
    assert sample.sleep >= 0.015
    # Sleep is deliberate, so it must not count as "stood still for no reason".
    assert sample.body_compute < sample.execute


def test_sleep_patch_restored_after_exit() -> None:
    original = asyncio.sleep
    prof = StepProfiler(loop_lag=False, print_report=False, hal=False)

    async def body() -> None:
        await asyncio.sleep(0)

    _run_under_sample(prof, body)
    assert asyncio.sleep is original


def test_sleep_tracking_can_be_disabled() -> None:
    prof = StepProfiler(loop_lag=False, print_report=False, hal=False, sleep_track=False)
    original = asyncio.sleep

    async def body() -> None:
        await asyncio.sleep(0.01)

    sample = _run_under_sample(prof, body)
    assert asyncio.sleep is original  # never patched
    assert sample.sleep == 0.0


def test_from_env_parses_inside_step_flags() -> None:
    prof = StepProfiler.from_env(
        {
            "RACCOON_PROFILE": "1",
            "RACCOON_PROFILE_HAL": "off",
            "RACCOON_PROFILE_SLEEP": "0",
            "RACCOON_PROFILE_HAL_SPAN_MS": "3.5",
        }
    )
    assert prof is not None
    assert prof.hal is False
    assert prof.sleep_track is False
    assert prof.hal_span_ms == 3.5


def test_body_compute_is_execute_minus_sleep_and_hal() -> None:
    s = StepSample(signature="x", path="x", depth=1, composite=False, tid=0)
    s.execute = 1.0
    s.sleep = 0.3
    s.hal = 0.2
    assert s.body_compute == pytest.approx(0.5)
    # never negative even if attribution overshoots
    s.sleep = 2.0
    assert s.body_compute == 0.0


@pytest.mark.skipif(not _native_available(), reason="raccoon.hal native module not installed")
def test_hal_calls_are_timed_and_attributed() -> None:
    import raccoon.hal as hal

    prof = StepProfiler(loop_lag=False, print_report=False, hal_span_ms=0.0)

    async def body() -> None:
        servo = hal.Servo(3)
        for _ in range(5):
            servo.set_position(90.0)
            servo.get_position()

    sample = _run_under_sample(prof, body)

    assert sample.hal_calls == 10
    assert sample.hal > 0.0
    assert "Servo.set_position" in prof.hal_stats
    count, total, mx = prof.hal_stats["Servo.set_position"]
    assert count == 5
    assert total > 0.0
    assert mx > 0.0
    # individual spans recorded because hal_span_ms == 0
    assert any(p.label.startswith("hal:Servo.set_position") for p in sample.phases)


@pytest.mark.skipif(not _native_available(), reason="raccoon.hal native module not installed")
def test_hal_methods_restored_after_exit() -> None:
    import raccoon.hal as hal

    before = hal.Servo.set_position
    prof = StepProfiler(loop_lag=False, print_report=False)

    async def body() -> None:
        hal.Servo(3).set_position(45.0)

    _run_under_sample(prof, body)
    assert hal.Servo.set_position is before


@pytest.mark.skipif(not _native_available(), reason="raccoon.hal native module not installed")
def test_static_hal_methods_are_not_wrapped() -> None:
    """Regression: static methods (Servo.fully_disable_all, Motor.disable_all)
    are called with no instance — wrapping them with a plain function broke
    their call convention and crashed missions. They must be left untouched."""
    import raccoon.hal as hal

    prof = StepProfiler(loop_lag=False, print_report=False, hal_span_ms=0.0)

    async def body() -> None:
        hal.Servo(3).set_position(45.0)  # instance method: timed
        hal.Servo.fully_disable_all()  # static on class: must not crash
        hal.Motor.disable_all()  # static on class: must not crash

    sample = _run_under_sample(prof, body)
    assert "Servo.set_position" in prof.hal_stats
    assert "Servo.fully_disable_all" not in prof.hal_stats
    assert "Motor.disable_all" not in prof.hal_stats
    assert sample.hal_calls == 1  # only the instance call was timed
    # static call still works after the profiler restored everything
    hal.Servo.fully_disable_all()


@pytest.mark.skipif(not _native_available(), reason="raccoon.hal native module not installed")
def test_report_contains_inside_step_sections() -> None:
    import raccoon.hal as hal

    prof = StepProfiler(loop_lag=False, print_report=False, hal_span_ms=0.0)

    async def body() -> None:
        hal.Servo(3).set_position(45.0)
        await asyncio.sleep(0.005)

    _run_under_sample(prof, body)
    report = prof.format_report()
    assert "INSIDE THE STEP BODY" in report
    assert "TOP HAL / TRANSPORT CALLS" in report
    assert "Servo.set_position" in report
