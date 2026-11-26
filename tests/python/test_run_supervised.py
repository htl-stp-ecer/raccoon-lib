"""Unit tests for ``run_supervised``.

The UI methods of the step are replaced, so these tests cover the control flow
(retry, confirm, cancel, failure, mission abort) without a screen transport.
"""

from __future__ import annotations

import asyncio
import importlib.util

import pytest

requires_libstp = pytest.mark.skipif(
    importlib.util.find_spec("raccoon") is None,
    reason="raccoon native module not installed",
)


@pytest.fixture(autouse=True)
def _fake_transport(monkeypatch):
    """UIStep fetches the transport on construction; no screen is rendered here."""
    if importlib.util.find_spec("raccoon") is not None:
        import raccoon.ui.step

        monkeypatch.setattr(raccoon.ui.step, "get_transport", object)


def _make(factory, choices=("confirm",), press_cancel=False):
    """Build a RunSupervised step with a scripted operator."""
    from raccoon.step import run_supervised

    step = run_supervised(factory).resolve()
    screens = []
    remaining = list(choices)
    current = {}

    async def display(screen):
        screen._closed = False
        current["screen"] = screen

    async def pump_events(timeout=0):
        if press_cancel:
            current["screen"]._closed = True

    async def show(screen):
        screens.append(screen)
        return remaining.pop(0)

    async def close_ui():
        pass

    step.display = display
    step.pump_events = pump_events
    step.show = show
    step.close_ui = close_ui
    return step, screens


def _inner(body, resources=frozenset()):
    from raccoon.step import Step

    class _Inner(Step):
        def required_resources(self):
            return resources

        def _generate_signature(self):
            return "Inner()"

        async def _execute_step(self, robot):
            await body()

    return _Inner()


@requires_libstp
def test_retry_rebuilds_the_step_and_confirm_continues():
    runs = []

    async def body():
        runs.append(1)

    built = []

    def factory():
        built.append(_inner(body))
        return built[-1]

    step, screens = _make(factory, choices=("retry", "confirm"))
    asyncio.run(step._execute_step(None))

    assert len(runs) == 2
    assert len(built) == 3  # one resource probe + one per attempt
    assert [s._outcome for s in screens] == ["done", "done"]


@requires_libstp
def test_cancel_runs_cleanup_of_the_supervised_step():
    cleaned = []

    async def body():
        try:
            await asyncio.sleep(10)
        finally:
            cleaned.append(True)

    async def run():
        step, screens = _make(lambda: _inner(body), press_cancel=False)

        async def pump_events(timeout=0):
            await asyncio.sleep(0.01)  # let the supervised step start
            step_screen["screen"]._closed = True

        step_screen = {}

        async def display(screen):
            screen._closed = False
            step_screen["screen"] = screen

        step.display = display
        step.pump_events = pump_events
        await step._execute_step(None)
        return screens

    screens = asyncio.run(run())

    assert cleaned == [True]
    assert [s._outcome for s in screens] == ["cancelled"]


@requires_libstp
def test_failure_is_shown_on_the_result_screen():
    async def body():
        raise RuntimeError("boom")

    step, screens = _make(lambda: _inner(body))
    asyncio.run(step._execute_step(None))

    assert screens[0]._outcome == "error"
    assert "boom" in screens[0]._text


@requires_libstp
def test_mission_abort_cancels_the_supervised_step():
    cleaned = []

    async def body():
        try:
            await asyncio.sleep(10)
        finally:
            cleaned.append(True)

    async def run():
        step, screens = _make(lambda: _inner(body))
        task = asyncio.create_task(step._execute_step(None))
        await asyncio.sleep(0.1)
        task.cancel()
        with pytest.raises(asyncio.CancelledError):
            await task
        return screens

    assert asyncio.run(run()) == []
    assert cleaned == [True]


@requires_libstp
def test_resources_come_from_the_supervised_step():
    from raccoon.step import run_supervised

    async def body():
        pass

    step = run_supervised(_inner(body, frozenset({"drive"}))).resolve()

    assert step.collected_resources() == frozenset({"drive"})


@requires_libstp
def test_rejects_non_steps():
    from raccoon.step import run_supervised

    with pytest.raises(TypeError, match="factory must return a Step"):
        run_supervised(lambda: 42).resolve()
    with pytest.raises(TypeError, match="Expected a Step"):
        run_supervised(42).resolve()
