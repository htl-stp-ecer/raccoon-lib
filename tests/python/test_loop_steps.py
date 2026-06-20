"""Tests for ``raccoon.step.logic.loop`` — ``LoopForever`` and ``LoopFor``.

These two combinators are pure-Python control flow: they wrap a child step and
re-run it (forever, or a fixed count). The tests use a lightweight fake child
step that satisfies ``StepProtocol`` and records how many times its
``run_step`` is invoked. No robot, drivetrain, or C++ is touched — a sentinel
``object()`` stands in for the robot since neither loop inspects it.

The module advertised in the task also mentioned ``DoWhile``, but the actual
``loop.py`` source defines only ``LoopForever`` and ``LoopFor``. ``DoWhileActive``
lives in ``raccoon.step.logic.do_while`` and is therefore out of scope here.
"""

from __future__ import annotations

import asyncio

import pytest

from raccoon.step.base import _step_path
from raccoon.step.logic.loop import LoopFor, LoopForever


class FakeStep:
    """Minimal child step satisfying ``StepProtocol``.

    Records each ``run_step`` call and the contextvar step-path that was active
    at the moment of the call, so tests can assert per-iteration path pushing.
    """

    def __init__(self, resources: frozenset[str] = frozenset()):
        self.calls = 0
        self.paths: list[list[str]] = []
        self._resources = resources
        self.resolved = 0

    def resolve(self):
        self.resolved += 1
        return self

    def required_resources(self) -> frozenset[str]:
        return frozenset()

    def collected_resources(self) -> frozenset[str]:
        return self._resources

    async def run_step(self, robot) -> None:
        self.calls += 1
        self.paths.append(list(_step_path.get()))


ROBOT = object()


# --------------------------------------------------------------------------- #
# Construction / validation
# --------------------------------------------------------------------------- #


def test_loopforever_rejects_non_step():
    bad = object()
    with pytest.raises(TypeError) as exc:
        LoopForever(bad)
    # Exact message (pins the literal so XX-wrapping mutants are killed).
    assert str(exc.value) == f"Expected step to be a Step instance, got {type(bad)}"


def test_loopfor_rejects_non_step():
    bad = object()
    with pytest.raises(TypeError) as exc:
        LoopFor(bad, iterations=3)
    assert str(exc.value) == f"Expected step to be a Step instance, got {type(bad)}"


def test_loopforever_resolves_child_once():
    child = FakeStep()
    loop = LoopForever(child)
    assert loop.step is child
    assert child.resolved == 1


def test_loopfor_resolves_child_and_stores_iterations():
    child = FakeStep()
    loop = LoopFor(child, iterations=4)
    assert loop.step is child
    assert loop.iterations == 4
    assert child.resolved == 1


@pytest.mark.parametrize("bad", [0, -1, -100])
def test_loopfor_rejects_zero_or_negative_iterations(bad):
    with pytest.raises(ValueError) as exc:
        LoopFor(FakeStep(), iterations=bad)
    # Exact message pins the literal so XX-wrapping mutants are killed.
    assert str(exc.value) == f"Iterations must be a positive integer, got {bad}"


@pytest.mark.parametrize("bad", [True, False])
def test_loopfor_rejects_bool_iterations(bad):
    # bool is a subclass of int, but LoopFor rejects it explicitly so that
    # loop_for(step, True) is a clear error rather than silently looping once.
    with pytest.raises(ValueError) as exc:
        LoopFor(FakeStep(), iterations=bad)
    assert "Iterations must be a positive integer" in str(exc.value)


@pytest.mark.parametrize("bad", [1.0, 2.5, "3", None])
def test_loopfor_rejects_non_int_iterations(bad):
    with pytest.raises(ValueError) as exc:
        LoopFor(FakeStep(), iterations=bad)
    assert "Iterations must be a positive integer" in str(exc.value)


# --------------------------------------------------------------------------- #
# Signatures / resources / composite flag
# --------------------------------------------------------------------------- #


def test_loopforever_signature():
    sig = LoopForever(FakeStep())._generate_signature()
    assert "LoopForever" in sig


def test_loopfor_signature_includes_count():
    sig = LoopFor(FakeStep(), iterations=7)._generate_signature()
    assert "LoopFor" in sig
    assert "7" in sig


def test_both_are_composite():
    assert LoopForever._composite is True
    assert LoopFor._composite is True


@pytest.mark.parametrize("cls", [LoopForever, LoopFor])
def test_dsl_step_decorator_applied_with_exact_tags(cls):
    # The @dsl_step decorator must be present (sets __dsl_step__) and register
    # the exact tag tuple ("control", "loop"). Kills decorator-removal and
    # tag-string mutants.
    assert getattr(cls, "__dsl_step__", False) is True
    assert cls.__dsl_step_tags__ == ("control", "loop")
    assert cls.__dsl_tags__ == ("control", "loop")
    # First tag is the primary category; second is the subcategory.
    assert cls.__dsl_step_tags__[0] == "control"
    assert cls.__dsl_step_tags__[1] == "loop"


def test_collected_resources_delegates_to_child():
    res = frozenset({"motor:0", "servo:1"})
    child = FakeStep(resources=res)
    assert LoopForever(child).collected_resources() == res
    child2 = FakeStep(resources=res)
    assert LoopFor(child2, iterations=2).collected_resources() == res


# --------------------------------------------------------------------------- #
# LoopFor runtime behavior
# --------------------------------------------------------------------------- #


@pytest.mark.asyncio
@pytest.mark.parametrize("n", [1, 2, 5, 10])
async def test_loopfor_runs_child_exactly_n_times(n):
    child = FakeStep()
    await LoopFor(child, iterations=n)._execute_step(ROBOT)
    assert child.calls == n


@pytest.mark.asyncio
async def test_loopfor_pushes_indexed_path_per_iteration():
    child = FakeStep()
    await LoopFor(child, iterations=3)._execute_step(ROBOT)
    # Each iteration must have pushed a distinct "Loop[i/total]" segment.
    assert child.paths == [["Loop[1/3]"], ["Loop[2/3]"], ["Loop[3/3]"]]


@pytest.mark.asyncio
async def test_loopfor_resets_path_after_completion():
    base = _step_path.get()
    child = FakeStep()
    await LoopFor(child, iterations=2)._execute_step(ROBOT)
    # The contextvar must be restored to whatever it was before the loop ran.
    assert _step_path.get() == base


@pytest.mark.asyncio
async def test_loopfor_resets_path_even_when_child_raises():
    base = _step_path.get()

    class Boom(FakeStep):
        async def run_step(self, robot):
            self.calls += 1
            raise RuntimeError("boom")

    child = Boom()
    with pytest.raises(RuntimeError, match="boom"):
        await LoopFor(child, iterations=5)._execute_step(ROBOT)
    # Only the first iteration ran; the exception propagated out.
    assert child.calls == 1
    # The finally block must have reset the path despite the exception.
    assert _step_path.get() == base


@pytest.mark.asyncio
async def test_loopfor_nests_path_under_existing_segment():
    token = _step_path.set(["Mission"])
    try:
        child = FakeStep()
        await LoopFor(child, iterations=2)._execute_step(ROBOT)
        assert child.paths == [["Mission", "Loop[1/2]"], ["Mission", "Loop[2/2]"]]
    finally:
        _step_path.reset(token)


# --------------------------------------------------------------------------- #
# LoopForever runtime behavior
# --------------------------------------------------------------------------- #


@pytest.mark.asyncio
async def test_loopforever_runs_repeatedly_until_cancelled():
    """The loop never exits on its own; external cancellation stops it."""

    class CountingStep(FakeStep):
        async def run_step(self, robot):
            self.calls += 1
            self.paths.append(list(_step_path.get()))
            # Yield so the cancelling task can interrupt between iterations.
            await asyncio.sleep(0)

    child = CountingStep()
    task = asyncio.create_task(LoopForever(child)._execute_step(ROBOT))

    # Let the loop spin several iterations.
    for _ in range(20):
        await asyncio.sleep(0)

    task.cancel()
    with pytest.raises(asyncio.CancelledError):
        await task

    # It must have looped many times (the count strictly increases, proving
    # per-iteration re-run rather than a single call).
    assert child.calls >= 3
    # Paths are sequentially numbered: Loop[1], Loop[2], ...
    assert child.paths[0] == ["Loop[1]"]
    assert child.paths[1] == ["Loop[2]"]
    assert child.paths[2] == ["Loop[3]"]


@pytest.mark.asyncio
async def test_loopforever_resets_path_on_cancellation():
    """When cancelled mid-iteration, the finally block restores the path.

    Approach (vs. the original broken version): we must observe ``_step_path``
    in the *same* task that ran the loop, otherwise the assertion is vacuous —
    asyncio copies contextvars per task, so a ``create_task`` child can never
    mutate the parent's contextvar regardless of whether the finally runs.

    Here we ``await`` the loop's ``_execute_step`` directly (same task / same
    context). The child raises ``CancelledError`` from *inside* its first
    iteration, exactly as a real mid-iteration cancellation would. We catch
    that ``CancelledError`` at the same level and then assert the path has been
    restored to ``base``. Because the loop pushed ``Loop[1]`` before awaiting
    the child and the only thing that pops it is the ``finally`` block's
    ``_step_path.reset(token)``, deleting that reset in the source would leave
    the path stuck at ``["Loop[1]"]`` and this test would fail.
    """
    base = _step_path.get()

    class CancelMidIteration(FakeStep):
        async def run_step(self, robot):
            self.calls += 1
            # Record the path that was active *inside* the iteration: it must
            # have been pushed by the loop before the child was awaited.
            self.paths.append(list(_step_path.get()))
            # Simulate the task being cancelled while blocked in this iteration.
            raise asyncio.CancelledError

    child = CancelMidIteration()

    with pytest.raises(asyncio.CancelledError):
        # Awaited directly => runs in this task's context, so the contextvar
        # mutations performed by the loop are visible here after it unwinds.
        await LoopForever(child)._execute_step(ROBOT)

    assert child.calls == 1
    # Inside the iteration the path was pushed to Loop[1].
    assert child.paths[0] == ["Loop[1]"]
    # The finally block must have popped Loop[1] back to the pre-loop path even
    # though the iteration unwound via CancelledError.
    assert _step_path.get() == base
