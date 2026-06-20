"""Spec-driven tests for ``raccoon.step.timeout.Timeout``.

The ``Timeout`` step wraps another step with a wall-clock budget via
``asyncio.wait_for``.  Per its docstring:

* If the wrapped step completes within ``seconds`` -> the timeout step
  finishes successfully (and the wrapped step actually ran to completion).
* If the wrapped step exceeds ``seconds`` -> ``asyncio.wait_for`` cancels the
  wrapped step, gives it a chance to clean up, logs an error, and *re-raises*
  ``TimeoutError`` (a hard failure, NOT a silent swallow).
* Any exception raised by the wrapped step propagates normally.

Every expected value here is derived independently from that contract, the
parameter units (``seconds`` is wall-clock seconds, cast to ``float``), and the
``StepProtocol`` it accepts — never copied from a live run.

No C++/hardware is needed: the robot is a sentinel ``object()`` (the step only
forwards it to the child's ``run_step``) and the wrapped step is a tiny fake
coroutine driver we control directly.  Timing is kept deterministic by using
budgets that are either far above or far below the child's controlled sleep, so
the "did it finish?" branch never races on the clock.
"""

from __future__ import annotations

import asyncio

import pytest

from raccoon.step import StepProtocol
from raccoon.step.timeout import Timeout

# ---------------------------------------------------------------------------
# Test doubles
# ---------------------------------------------------------------------------


class FakeStep:
    """Minimal ``StepProtocol`` stand-in with a controllable ``run_step``.

    * ``run_seconds``  -- how long ``run_step`` awaits before "finishing".
      0.0 completes essentially immediately.
    * ``raises``       -- optional exception instance to raise instead of
      finishing normally (after the sleep).
    * Records lifecycle so tests can assert *which* branch happened:
      ``started`` / ``finished`` / ``cancelled`` / ``cleaned_up``.

    ``resolve()`` returns ``self`` (mirrors ``Step.resolve`` for leaf steps);
    ``required_resources`` / ``collected_resources`` return a fixed set so we
    can prove ``Timeout.collected_resources`` delegates to the child.
    """

    def __init__(
        self,
        run_seconds: float = 0.0,
        *,
        raises: BaseException | None = None,
        resources: frozenset[str] = frozenset(),
    ) -> None:
        self.run_seconds = run_seconds
        self._raises = raises
        self._resources = resources
        self.started = False
        self.finished = False
        self.cancelled = False
        self.cleaned_up = False
        self.robot_seen: object = None

    async def run_step(self, robot: object) -> None:
        self.started = True
        self.robot_seen = robot
        try:
            await asyncio.sleep(self.run_seconds)
            if self._raises is not None:
                raise self._raises
            self.finished = True
        except asyncio.CancelledError:
            # Emulate a real step's `finally`-style cleanup on cancellation.
            self.cancelled = True
            self.cleaned_up = True
            raise

    def resolve(self) -> "FakeStep":
        return self

    def required_resources(self) -> frozenset[str]:
        return self._resources

    def collected_resources(self) -> frozenset[str]:
        return self._resources


class _NotAStep:
    """Object that does NOT satisfy ``StepProtocol`` (no run_step, etc.)."""


_ROBOT = object()  # Timeout only forwards this to the child's run_step.


# ===========================================================================
# 1. Constructor validation & wiring
# ===========================================================================


def test_rejects_non_step_with_typeerror() -> None:
    with pytest.raises(TypeError):
        Timeout(_NotAStep(), seconds=1.0)


def test_accepts_object_satisfying_step_protocol() -> None:
    # Structural protocol: FakeStep has run_step + resource methods, so it is
    # a valid StepProtocol even though it does not subclass Step.
    assert isinstance(FakeStep(), StepProtocol)
    step = Timeout(FakeStep(), seconds=1.0)
    assert isinstance(step.step, FakeStep)


@pytest.mark.parametrize("seconds", [0, 0.0, -1, -0.001, -1000.0])
def test_non_positive_seconds_rejected(seconds: float) -> None:
    # Docstring: "Must be positive." 0 is the boundary and must be rejected.
    with pytest.raises(ValueError, match=r"positive"):
        Timeout(FakeStep(), seconds=seconds)


@pytest.mark.parametrize("seconds", [1e-9, 0.5, 1, 30, 1000.0])
def test_positive_seconds_accepted_and_cast_to_float(seconds: float) -> None:
    step = Timeout(FakeStep(), seconds=seconds)
    assert step.seconds == pytest.approx(float(seconds))
    assert isinstance(step.seconds, float)  # int input must become float


def test_int_seconds_become_float() -> None:
    step = Timeout(FakeStep(), seconds=5)
    assert step.seconds == pytest.approx(5.0)
    assert type(step.seconds) is float


def test_resolve_is_called_on_child() -> None:
    class ResolvingStep(FakeStep):
        def __init__(self) -> None:
            super().__init__()
            self.resolved_to = FakeStep()

        def resolve(self) -> FakeStep:  # type: ignore[override]
            return self.resolved_to

    raw = ResolvingStep()
    step = Timeout(raw, seconds=1.0)
    # Timeout stores the *resolved* child, not the builder/raw object.
    assert step.step is raw.resolved_to
    assert step.step is not raw


def test_result_starts_none() -> None:
    assert Timeout(FakeStep(), seconds=1.0).result is None


def test_collected_resources_delegates_to_child() -> None:
    child = FakeStep(resources=frozenset({"motor:2", "servo:0"}))
    step = Timeout(child, seconds=1.0)
    assert step.collected_resources() == frozenset({"motor:2", "servo:0"})


def test_required_resources_of_composite_is_empty() -> None:
    # Composite steps don't touch hardware themselves; the guard logic lives in
    # collected_resources.  required_resources must stay empty.
    child = FakeStep(resources=frozenset({"motor:2"}))
    step = Timeout(child, seconds=1.0)
    assert step.required_resources() == frozenset()


# ===========================================================================
# 2. DSL metadata & signature (semantic contract, not exact prose)
# ===========================================================================


def test_is_composite() -> None:
    assert Timeout._composite is True


def test_dsl_tags() -> None:
    assert Timeout.__dsl_step_tags__ == ("control", "timeout")
    assert Timeout.__dsl_step__ is True


def test_signature_carries_child_name_and_seconds() -> None:
    sig = Timeout(FakeStep(), seconds=2.5)._generate_signature()
    # Wording/format is prose (not pinned), but the identity-bearing tokens are
    # the child class name and the numeric budget.
    assert "FakeStep" in sig
    assert "2.5" in sig


# ===========================================================================
# 3. _execute_step — the timeout boundary behaviour
# ===========================================================================


@pytest.mark.asyncio
async def test_child_finishes_within_budget_completes_normally() -> None:
    # Budget far exceeds the child's near-instant run: the wrapped step runs to
    # completion and the timeout step returns without raising.
    child = FakeStep(run_seconds=0.0)
    step = Timeout(child, seconds=10.0)

    await step._execute_step(_ROBOT)

    assert child.started is True
    assert child.finished is True
    assert child.cancelled is False
    assert child.robot_seen is _ROBOT  # robot forwarded unchanged


@pytest.mark.asyncio
async def test_child_exceeding_budget_raises_timeout_and_cancels_child() -> None:
    # Child wants 10s but only gets 0.01s -> wait_for cancels it and re-raises
    # TimeoutError (the documented HARD failure, not a swallow).
    child = FakeStep(run_seconds=10.0)
    step = Timeout(child, seconds=0.01)

    with pytest.raises(TimeoutError):
        await step._execute_step(_ROBOT)

    assert child.started is True
    assert child.finished is False  # never completed
    assert child.cancelled is True  # wait_for cancelled it
    assert child.cleaned_up is True  # cleanup (finally-style) got to run


@pytest.mark.asyncio
async def test_timeout_logs_error_at_boundary() -> None:
    # The boundary logs an error before re-raising.  ClassNameLogger.error
    # routes through raccoon.log (not stdlib logging), so we capture by
    # overriding step.error (same convention as the wall_align suite).
    child = FakeStep(run_seconds=10.0)
    step = Timeout(child, seconds=0.01)
    errors: list[str] = []
    step.error = errors.append  # type: ignore[assignment]

    with pytest.raises(TimeoutError):
        await step._execute_step(_ROBOT)

    # Exactly one boundary error, carrying the child-name token (wording and
    # exact formatting are prose, not pinned).
    assert len(errors) == 1
    assert "FakeStep" in errors[0]


@pytest.mark.asyncio
async def test_success_path_logs_no_error() -> None:
    # The happy path must NOT emit a timeout error.
    child = FakeStep(run_seconds=0.0)
    step = Timeout(child, seconds=10.0)
    errors: list[str] = []
    step.error = errors.append  # type: ignore[assignment]

    await step._execute_step(_ROBOT)
    assert errors == []


@pytest.mark.asyncio
async def test_child_exception_propagates_unchanged() -> None:
    # A non-timeout error from the wrapped step must propagate as-is (not be
    # masked as a TimeoutError) when it happens within the budget.
    sentinel = RuntimeError("boom")
    child = FakeStep(run_seconds=0.0, raises=sentinel)
    step = Timeout(child, seconds=10.0)
    errors: list[str] = []
    step.error = errors.append  # type: ignore[assignment]

    with pytest.raises(RuntimeError, match="boom") as exc:
        await step._execute_step(_ROBOT)

    assert exc.value is sentinel
    assert child.started is True
    assert child.finished is False
    # A non-timeout failure within budget is NOT a timeout: the boundary
    # "exceeded timeout" error must NOT fire. (Broadening the source's
    # `except TimeoutError` to `except Exception` would log here.)
    assert errors == []


@pytest.mark.asyncio
async def test_child_exception_is_not_a_timeout_error() -> None:
    # Guard against the implementation accidentally wrapping every failure in
    # TimeoutError: a ValueError within budget stays a ValueError.
    child = FakeStep(run_seconds=0.0, raises=ValueError("bad"))
    step = Timeout(child, seconds=10.0)
    errors: list[str] = []
    step.error = errors.append  # type: ignore[assignment]

    with pytest.raises(ValueError, match="bad"):
        await step._execute_step(_ROBOT)

    # Same guard as above: a ValueError within budget must not be logged as a
    # timeout-boundary error.
    assert errors == []


@pytest.mark.asyncio
async def test_child_finishing_well_inside_budget_does_not_time_out() -> None:
    # A child that needs ~5 ms under a generous 5 s budget must complete, not
    # time out.  (The exact-at-deadline tie is inherently racy in real asyncio
    # wall-clock time and is therefore not asserted; the two clearly-separated
    # sides of the boundary are covered by the finish/timeout tests above.)
    child = FakeStep(run_seconds=0.005)
    step = Timeout(child, seconds=5.0)

    await step._execute_step(_ROBOT)
    assert child.finished is True
    assert child.cancelled is False


@pytest.mark.asyncio
async def test_each_run_starts_fresh_child_state() -> None:
    # Re-running the same Timeout instance re-invokes the child's run_step.
    child = FakeStep(run_seconds=0.0)
    step = Timeout(child, seconds=10.0)
    await step._execute_step(_ROBOT)
    assert child.finished is True

    # Reset the fake and confirm a second execution drives it again.
    child.started = False
    child.finished = False
    await step._execute_step(_ROBOT)
    assert child.started is True
    assert child.finished is True


# ===========================================================================
# 4. Integration via run_step (composite path) — end to end
# ===========================================================================


@pytest.mark.asyncio
async def test_run_step_success_path() -> None:
    # run_step is the public entry; for a composite it logs + delegates to
    # _execute_step.  Disable timing to keep this purely about the timeout.
    child = FakeStep(run_seconds=0.0)
    step = Timeout(child, seconds=10.0)
    step._skip_timing = True

    await step.run_step(_ROBOT)
    assert child.finished is True


@pytest.mark.asyncio
async def test_run_step_propagates_timeout() -> None:
    child = FakeStep(run_seconds=10.0)
    step = Timeout(child, seconds=0.01)
    step._skip_timing = True

    with pytest.raises(TimeoutError):
        await step.run_step(_ROBOT)
    assert child.cancelled is True
