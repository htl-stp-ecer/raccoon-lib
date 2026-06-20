"""Tests for raccoon.timing.synchronizer.Synchronizer.

Strategy:
- get_time / delta math is verified deterministically by patching the
  synchronizer's clock source (asyncio.get_running_loop().time) via a fake
  loop, so we can assert exact elapsed/delta values without real time.
- The actual await asyncio.sleep(delta) is exercised against the real
  running loop with tiny deltas so that sleep + cancellation behaviour is
  real, while log messages are captured by monkeypatching info/warn.
"""

from __future__ import annotations

import asyncio
import importlib
import re

import pytest

import raccoon.timing.synchronizer as sync_mod

# The module is typically imported during test collection (before coverage's
# tracer is attached to this file), which leaves import-time lines (imports,
# class/def headers) recorded as "missed" even though their bodies run. Reload
# it under the active tracer so those lines are attributed correctly.
sync_mod = importlib.reload(sync_mod)
Synchronizer = sync_mod.Synchronizer


class _FakeLoop:
    """Minimal event loop whose monotonic clock we control."""

    def __init__(self, t=0.0):
        self._t = t

    def time(self):
        return self._t

    def set(self, t):
        self._t = t


@pytest.fixture
def fake_clock(monkeypatch):
    """Patch the synchronizer's clock source (asyncio.get_running_loop).

    The synchronizer reads the monotonic clock via
    ``asyncio.get_running_loop().time()``. We replace ``get_running_loop``
    with a lambda returning a controllable fake loop. This only affects the
    synchronizer's own lookups -- ``asyncio.create_task`` and ``asyncio.sleep``
    resolve the loop through ``asyncio.events.get_running_loop`` internally,
    so real task scheduling under @pytest.mark.asyncio is unaffected. Patching
    the attribute also lets the *sync* test bodies below call ``get_time()``
    without an actual running loop.
    """
    loop = _FakeLoop(0.0)
    monkeypatch.setattr(sync_mod.asyncio, "get_running_loop", lambda: loop)
    return loop


@pytest.fixture
def logs(monkeypatch):
    """Capture info/warn calls emitted by the synchronizer."""
    captured = {"info": [], "warn": []}
    monkeypatch.setattr(sync_mod, "info", lambda msg: captured["info"].append(msg))
    monkeypatch.setattr(sync_mod, "warn", lambda msg: captured["warn"].append(msg))
    return captured


@pytest.fixture(autouse=True)
def clear_no_checkpoints(monkeypatch):
    """Ensure --no-checkpoints is OFF unless a test opts in."""
    monkeypatch.delenv("LIBSTP_NO_CHECKPOINTS", raising=False)


# --------------------------------------------------------------------------
# Construction / start_recording / get_time
# --------------------------------------------------------------------------


def test_initial_start_time_is_none():
    s = Synchronizer()
    assert s.start_time is None


def test_start_recording_captures_loop_time(fake_clock):
    fake_clock.set(12.5)
    s = Synchronizer()
    s.start_recording()
    assert s.start_time == 12.5


def test_get_time_elapsed_since_start(fake_clock):
    fake_clock.set(100.0)
    s = Synchronizer()
    s.start_recording()
    fake_clock.set(103.25)
    assert s.get_time() == pytest.approx(3.25, abs=1e-9)


def test_get_time_zero_immediately_after_start(fake_clock):
    fake_clock.set(7.0)
    s = Synchronizer()
    s.start_recording()
    assert s.get_time() == pytest.approx(0.0, abs=1e-9)


def test_get_time_before_recording_raises(fake_clock):
    s = Synchronizer()
    # start_time is None -> float - None raises TypeError.
    with pytest.raises(TypeError):
        s.get_time()


def test_re_recording_resets_origin(fake_clock):
    fake_clock.set(5.0)
    s = Synchronizer()
    s.start_recording()
    fake_clock.set(20.0)
    s.start_recording()
    assert s.start_time == 20.0
    fake_clock.set(22.0)
    assert s.get_time() == pytest.approx(2.0, abs=1e-9)


# --------------------------------------------------------------------------
# wait_until_checkpoint
# --------------------------------------------------------------------------


@pytest.mark.asyncio
async def test_wait_no_checkpoints_bypass(fake_clock, logs, monkeypatch):
    monkeypatch.setenv("LIBSTP_NO_CHECKPOINTS", "1")
    fake_clock.set(0.0)
    s = Synchronizer()
    s.start_recording()

    # Even with a checkpoint far in the future, this returns immediately
    # and never reaches the delta computation or any sleep.
    await s.wait_until_checkpoint(1000.0)

    # Exactly one info line: the no-checkpoints bypass for the right checkpoint.
    # Cosmetic wording is matched loosely; the behavioral spec is that the
    # bypass branch ran (and reports the checkpoint value).
    assert len(logs["info"]) == 1
    assert "no-checkpoints" in logs["info"][0]
    assert "1000.0" in logs["info"][0]
    # No "waiting until" / "reached" messages because we short-circuited.
    assert not any("waiting until" in m for m in logs["info"])
    assert not any("reached the checkpoint" in m for m in logs["info"])
    assert logs["warn"] == []


@pytest.mark.asyncio
async def test_wait_missed_checkpoint_negative_delta(fake_clock, logs):
    fake_clock.set(0.0)
    s = Synchronizer()
    s.start_recording()
    fake_clock.set(10.0)  # 10s elapsed

    # Checkpoint at 4s, but we are already at 10s -> delta = -6.0
    await s.wait_until_checkpoint(4.0)

    # Behavioral spec: exactly one WARN (not info), reporting the negative
    # delta of -6.0 for the checkpoint at 4.0. Wording is matched loosely.
    assert logs["info"] == []
    assert len(logs["warn"]) == 1
    assert re.search(r"passed", logs["warn"][0])
    assert "4.0" in logs["warn"][0]
    assert "delta: -6.0" in logs["warn"][0]
    # Missed branch returns before emitting waiting/reached info messages.
    assert not any("waiting until" in m for m in logs["info"])
    assert not any("reached the checkpoint" in m for m in logs["info"])


@pytest.mark.asyncio
async def test_wait_exactly_at_checkpoint_delta_zero_does_not_warn(fake_clock, logs):
    fake_clock.set(0.0)
    s = Synchronizer()
    s.start_recording()
    fake_clock.set(5.0)

    # delta == 0.0 -> NOT < 0, so it must take the wait branch (sleeps 0s).
    await s.wait_until_checkpoint(5.0)

    assert logs["warn"] == []
    assert any("waiting until 5.0 seconds" in m and "delta: 0.0" in m for m in logs["info"])
    assert any("reached the checkpoint at 5.0 seconds" in m for m in logs["info"])


@pytest.mark.asyncio
async def test_wait_future_checkpoint_sleeps_and_reaches(fake_clock, logs, monkeypatch):
    fake_clock.set(0.0)
    s = Synchronizer()
    s.start_recording()
    fake_clock.set(1.0)  # delta = 0.05 - ... -> compute below

    # Capture the slept-for delta and avoid real waiting.
    slept = []

    async def fake_sleep(d):
        slept.append(d)

    monkeypatch.setattr(sync_mod.asyncio, "sleep", fake_sleep)

    # Checkpoint at 1.25s, elapsed = 1.0 -> delta = 0.25
    await s.wait_until_checkpoint(1.25)

    # Behavioral spec: slept for exactly the positive delta, two info lines in
    # order (waiting-with-delta then reached), no warn. Cosmetic phrasing loose;
    # numeric delta + checkpoint + branch are pinned.
    assert slept == [pytest.approx(0.25, abs=1e-9)]
    assert len(logs["info"]) == 2
    assert "waiting until" in logs["info"][0]
    assert "1.25" in logs["info"][0]
    assert "delta: 0.25" in logs["info"][0]
    assert "reached the checkpoint" in logs["info"][1]
    assert "1.25" in logs["info"][1]
    assert logs["warn"] == []


@pytest.mark.asyncio
async def test_wait_real_small_sleep(logs):
    """End-to-end against the real loop with a tiny positive delta."""
    s = Synchronizer()
    s.start_recording()
    # checkpoint slightly ahead of "now" -> small positive delta, real sleep.
    target = s.get_time() + 0.02
    await s.wait_until_checkpoint(target)
    assert logs["warn"] == []
    assert any("reached the checkpoint" in m for m in logs["info"])


# --------------------------------------------------------------------------
# do_until_checkpoint
# --------------------------------------------------------------------------


@pytest.mark.asyncio
async def test_do_until_missed_checkpoint_does_not_spawn_task(fake_clock, logs):
    fake_clock.set(0.0)
    s = Synchronizer()
    s.start_recording()
    fake_clock.set(8.0)

    started = []

    async def func():
        started.append(True)
        await asyncio.sleep(100)

    await s.do_until_checkpoint(3.0, func)

    # Negative-delta branch returns early -> func never scheduled.
    assert started == []
    # Behavioral spec: exactly one WARN reporting delta -5.0 for checkpoint 3.0.
    assert len(logs["warn"]) == 1
    assert re.search(r"passed", logs["warn"][0])
    assert "3.0" in logs["warn"][0]
    assert "delta: -5.0" in logs["warn"][0]
    assert not any("executing until" in m for m in logs["info"])


@pytest.mark.asyncio
async def test_do_until_spawns_runs_then_cancels(fake_clock, logs, monkeypatch):
    fake_clock.set(0.0)
    s = Synchronizer()
    s.start_recording()
    fake_clock.set(2.0)  # elapsed 2.0

    events = {"started": False, "cancelled": False, "finished": False}
    real_sleep = asyncio.sleep

    async def func():
        events["started"] = True
        try:
            await real_sleep(100)  # long-running, expected to be cancelled
            events["finished"] = True
        except asyncio.CancelledError:
            events["cancelled"] = True
            raise

    slept = []

    async def fake_sleep(d):
        # Yield control so the spawned task actually starts running.
        slept.append(d)
        await real_sleep(0)

    monkeypatch.setattr(sync_mod.asyncio, "sleep", fake_sleep)

    # checkpoint at 2.5, elapsed 2.0 -> delta 0.5
    await s.do_until_checkpoint(2.5, func)

    assert slept == [pytest.approx(0.5, abs=1e-9)]
    assert events["started"] is True
    assert events["cancelled"] is True
    assert events["finished"] is False
    # Behavioral spec: two info lines in order (executing-with-delta then
    # reached), no warn. Cosmetic phrasing loose; numeric delta + checkpoint
    # + branch pinned.
    assert len(logs["info"]) == 2
    assert "executing until" in logs["info"][0]
    assert "2.5" in logs["info"][0]
    assert "delta: 0.5" in logs["info"][0]
    assert "reached the checkpoint" in logs["info"][1]
    assert "2.5" in logs["info"][1]
    assert logs["warn"] == []


@pytest.mark.asyncio
async def test_do_until_passes_args_and_kwargs(fake_clock, logs, monkeypatch):
    fake_clock.set(0.0)
    s = Synchronizer()
    s.start_recording()

    received = {}
    real_sleep = asyncio.sleep

    async def func(a, b, c=None):
        received["args"] = (a, b)
        received["c"] = c
        await real_sleep(100)

    async def fake_sleep(d):
        await real_sleep(0)

    monkeypatch.setattr(sync_mod.asyncio, "sleep", fake_sleep)

    await s.do_until_checkpoint(1.0, func, 7, 8, c="nine")

    assert received == {"args": (7, 8), "c": "nine"}


@pytest.mark.asyncio
async def test_do_until_zero_delta_still_spawns(fake_clock, logs, monkeypatch):
    fake_clock.set(0.0)
    s = Synchronizer()
    s.start_recording()
    fake_clock.set(3.0)

    started = []
    real_sleep = asyncio.sleep

    async def func():
        started.append(True)
        await real_sleep(100)

    slept = []

    async def fake_sleep(d):
        slept.append(d)
        await real_sleep(0)

    monkeypatch.setattr(sync_mod.asyncio, "sleep", fake_sleep)

    # delta == 0.0 -> not < 0, takes the execute branch.
    await s.do_until_checkpoint(3.0, func)

    assert slept == [pytest.approx(0.0, abs=1e-9)]
    assert started == [True]
    assert logs["warn"] == []


@pytest.mark.asyncio
async def test_do_until_real_quick_func_completes_before_cancel():
    """If the func finishes before the delta elapses, cancel is a no-op."""
    s = Synchronizer()
    s.start_recording()

    done = []

    async def quick():
        done.append("done")

    target = s.get_time() + 0.02
    await s.do_until_checkpoint(target, quick)
    assert done == ["done"]


# --------------------------------------------------------------------------
# Regression: the clock must come from the *running* loop
# --------------------------------------------------------------------------


@pytest.mark.asyncio
async def test_uses_running_loop_clock_not_get_event_loop(monkeypatch):
    """Pins the get_event_loop() -> get_running_loop() fix.

    Under a real running loop, start_recording() and get_time() must both
    read time from the actual running loop's clock. We verify by (a) routing
    the synchronizer's loop lookup through a spy that records the loop object
    and (b) checking the recorded origin and elapsed time agree with the real
    running loop's monotonic clock. If the code reverted to
    asyncio.get_event_loop(), the spy below would never see get_running_loop
    being called and the recorded loop would not be the running loop.
    """
    running = asyncio.get_running_loop()

    seen_loops = []
    orig_get_running_loop = asyncio.get_running_loop

    def spy():
        loop = orig_get_running_loop()
        seen_loops.append(loop)
        return loop

    monkeypatch.setattr(sync_mod.asyncio, "get_running_loop", spy)

    s = Synchronizer()
    s.start_recording()
    # start_time is anchored to the running loop's clock.
    assert s.start_time == pytest.approx(running.time(), abs=0.05)

    await asyncio.sleep(0.02)
    before = running.time()
    elapsed = s.get_time()
    after = running.time()

    # Both calls resolved through get_running_loop, and to the SAME object,
    # which is the actual running loop.
    assert len(seen_loops) == 2  # one in start_recording, one in get_time
    assert seen_loops[0] is running
    assert seen_loops[1] is running

    # get_time() == running.time() - start_time, evaluated at some instant
    # bracketed by [before, after] readings of the same loop clock.
    assert (before - s.start_time) <= elapsed <= (after - s.start_time)
    assert elapsed >= 0.02


def test_get_time_raises_without_running_loop():
    """Outside any running loop, the clock lookup must fail (not silently fall
    back to a deprecated get_event_loop). This is a sync body, so there is no
    running loop and the unpatched asyncio.get_running_loop() must raise."""
    s = Synchronizer()
    with pytest.raises(RuntimeError):
        s.get_time()


# --------------------------------------------------------------------------
# do_until_checkpoint: func that ignores cancellation must not hang
# --------------------------------------------------------------------------


@pytest.mark.asyncio
async def test_do_until_func_ignoring_cancellation_still_returns(fake_clock, logs, monkeypatch):
    """do_until_checkpoint must return even if func swallows CancelledError.

    The checkpoint contract cancels the spawned task at the deadline. A
    well-behaved coroutine re-raises CancelledError, but a non-cooperative one
    may catch it and return normally. do_until_checkpoint awaits the task after
    cancelling, so as long as the func eventually completes (here: returns
    promptly after swallowing the cancel), the method must not hang.
    """
    fake_clock.set(0.0)
    s = Synchronizer()
    s.start_recording()
    fake_clock.set(2.0)

    events = {"started": False, "swallowed": False, "returned": False}
    real_sleep = asyncio.sleep

    async def uncooperative():
        events["started"] = True
        try:
            await real_sleep(100)
        except asyncio.CancelledError:
            # Non-cooperative: swallow the cancellation and just return.
            events["swallowed"] = True
            return
        events["returned"] = True

    async def fake_sleep(d):
        # Yield so the spawned task starts and parks in real_sleep(100).
        await real_sleep(0)

    monkeypatch.setattr(sync_mod.asyncio, "sleep", fake_sleep)

    # delta 0.5 -> spawn, "sleep", cancel, await the (uncooperative) task.
    # Bounded by a wall-clock timeout so a hang fails loudly instead of
    # stalling the suite.
    await asyncio.wait_for(s.do_until_checkpoint(2.5, uncooperative), timeout=2.0)

    assert events["started"] is True
    assert events["swallowed"] is True  # it caught the cancel and returned
    assert events["returned"] is False
    # Despite swallowing the cancel, the checkpoint was still reported reached.
    assert any("reached the checkpoint" in m for m in logs["info"])
    assert logs["warn"] == []
