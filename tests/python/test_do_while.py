"""Spec-driven tests for ``raccoon.step.logic.do_while`` — ``DoWhileActive``.

NOTE ON SCOPE: the task brief described a classic "do-while loop" (body runs at
least once, condition re-evaluated each round). The *actual* source defines no
such loop. ``do_while.py`` contains exactly one step, ``DoWhileActive``, which
is a CONCURRENT combinator, not an iterating loop. Its contract (from the
docstring) is:

  * ``reference_step`` and ``task`` start at the SAME time.
  * When ``reference_step`` finishes (normally OR via exception) the ``task`` is
    immediately cancelled.
  * If ``task`` finishes first, ``reference_step`` keeps running to completion.

These tests therefore exercise that real contract. Every expected value is
derived from the docstring / parameter semantics, not from running the code.

The tests use a lightweight ``FakeStep`` satisfying ``StepProtocol`` that
records start / completion / cancellation and can sleep or raise on demand. A
bare ``object()`` stands in for the robot — ``DoWhileActive._execute_step``
never inspects it, and step timing is disabled by default so ``run_step`` runs
without a real ``GenericRobot`` or C++ drivetrain.
"""

from __future__ import annotations

import asyncio
import importlib

import pytest

# The conftest pre-imports the logic package before pytest-cov attaches its
# tracer, so the module's import-time def/docstring lines never get counted.
# Reload the module here, under coverage, so those definition statements
# re-execute while traced. DoWhileActive is bound from the reloaded module so
# identity stays consistent within this file.
import raccoon.step.logic.do_while as do_while
from raccoon.step.base import Step
from raccoon.step.resource import ResourceConflictError

do_while = importlib.reload(do_while)
DoWhileActive = do_while.DoWhileActive

ROBOT = object()


# --------------------------------------------------------------------------- #
# Test double
# --------------------------------------------------------------------------- #
class FakeStep(Step):
    """Minimal child step satisfying ``StepProtocol`` for DoWhileActive.

    Records how many times it is started, completes normally, and is
    cancelled. ``sleep`` makes the step take wall-clock time; ``raises`` makes
    it finish by raising. ``resources`` is what ``collected_resources`` reports
    (used to drive the static conflict check). ``resolve`` is counted.
    """

    def __init__(
        self,
        *,
        resources: frozenset[str] = frozenset(),
        sleep: float = 0.0,
        raises: BaseException | None = None,
    ) -> None:
        super().__init__()
        self._resources = resources
        self._sleep = sleep
        self._raises = raises
        self.started = 0
        self.completed = 0
        self.cancelled = 0
        self.resolved = 0

    def resolve(self) -> "FakeStep":
        self.resolved += 1
        return self

    def required_resources(self) -> frozenset[str]:
        return frozenset()

    def collected_resources(self) -> frozenset[str]:
        return self._resources

    def _generate_signature(self) -> str:
        return "FakeStep"

    async def run_step(self, robot) -> None:
        self.started += 1
        try:
            if self._sleep:
                await asyncio.sleep(self._sleep)
            if self._raises is not None:
                raise self._raises
            self.completed += 1
        except asyncio.CancelledError:
            self.cancelled += 1
            raise


# --------------------------------------------------------------------------- #
# 1. Construction / type validation
# --------------------------------------------------------------------------- #
class TestConstructionValidation:
    def test_rejects_non_step_reference(self):
        # A bare object() is not a StepProtocol -> TypeError (token: type name).
        with pytest.raises(TypeError, match=r"reference_step"):
            DoWhileActive(object(), FakeStep())

    def test_rejects_non_step_task(self):
        with pytest.raises(TypeError, match=r"task"):
            DoWhileActive(FakeStep(), object())

    def test_reference_type_error_names_offending_type(self):
        # Carries the offending type token (not pinning full prose).
        with pytest.raises(TypeError, match=r"object"):
            DoWhileActive(object(), FakeStep())

    def test_resolves_both_children_once(self):
        ref = FakeStep()
        task = FakeStep()
        dw = DoWhileActive(ref, task)
        # Each child resolved exactly once and stored as the resolved object.
        assert ref.resolved == 1
        assert task.resolved == 1
        assert dw.reference_step is ref
        assert dw.task is task

    def test_is_composite(self):
        # Composite steps don't push the step-path / re-log as a leaf.
        assert DoWhileActive._composite is True

    def test_signature_is_stable_token(self):
        # Identity token only; exact formatting is prose (not pinned).
        sig = DoWhileActive(FakeStep(), FakeStep())._generate_signature()
        assert "DoWhileActive" in sig


# --------------------------------------------------------------------------- #
# 2. Resource conflict detection (static, at construction)
# --------------------------------------------------------------------------- #
class TestResourceConflicts:
    def test_overlapping_resource_raises(self):
        # Both branches claim "drive" -> static validation must reject.
        with pytest.raises(ResourceConflictError, match=r"drive"):
            DoWhileActive(
                FakeStep(resources=frozenset({"drive"})),
                FakeStep(resources=frozenset({"drive"})),
            )

    def test_wildcard_servo_conflicts_with_specific(self):
        # "servo:*" must conflict with "servo:0" (wildcard expansion path).
        with pytest.raises(ResourceConflictError, match=r"servo:0"):
            DoWhileActive(
                FakeStep(resources=frozenset({"servo:*"})),
                FakeStep(resources=frozenset({"servo:0"})),
            )

    def test_disjoint_resources_ok(self):
        # Different resources -> construction succeeds, no raise.
        dw = DoWhileActive(
            FakeStep(resources=frozenset({"motor:0"})),
            FakeStep(resources=frozenset({"servo:1"})),
        )
        assert dw is not None

    def test_collected_resources_is_union(self):
        # Composite resources = union of both branches (spec: collected_resources).
        dw = DoWhileActive(
            FakeStep(resources=frozenset({"motor:0"})),
            FakeStep(resources=frozenset({"servo:1"})),
        )
        assert dw.collected_resources() == frozenset({"motor:0", "servo:1"})

    def test_collected_resources_empty_when_no_children_resources(self):
        dw = DoWhileActive(FakeStep(), FakeStep())
        assert dw.collected_resources() == frozenset()


# --------------------------------------------------------------------------- #
# 3. Runtime contract — concurrency, cancellation, ordering
# --------------------------------------------------------------------------- #
class TestRuntimeContract:
    @pytest.mark.asyncio
    async def test_both_steps_start(self):
        # Spec: both steps start executing at the same time.
        ref = FakeStep(sleep=0.02)
        task = FakeStep(sleep=10.0)
        await DoWhileActive(ref, task).run_step(ROBOT)
        assert ref.started == 1
        assert task.started == 1

    @pytest.mark.asyncio
    async def test_reference_finish_cancels_long_task(self):
        # Spec: when reference finishes, the still-running task is cancelled.
        ref = FakeStep(sleep=0.02)
        task = FakeStep(sleep=10.0)  # would never finish on its own
        await DoWhileActive(ref, task).run_step(ROBOT)
        assert ref.completed == 1  # reference ran to normal completion
        assert task.cancelled == 1  # task was cancelled, not completed
        assert task.completed == 0

    @pytest.mark.asyncio
    async def test_returns_after_reference_completes(self):
        # run_step must not hang waiting on the (long) task — it returns once
        # the reference is done and the task is cancelled. A short timeout
        # guards against a regression that awaits the task without cancelling.
        ref = FakeStep(sleep=0.01)
        task = FakeStep(sleep=100.0)
        await asyncio.wait_for(DoWhileActive(ref, task).run_step(ROBOT), timeout=2.0)

    @pytest.mark.asyncio
    async def test_task_finishing_first_does_not_stop_reference(self):
        # Spec: if task finishes before reference, reference keeps running and
        # completes on its own. Task completes normally (never cancelled).
        ref = FakeStep(sleep=0.05)
        task = FakeStep(sleep=0.0)  # finishes immediately
        await DoWhileActive(ref, task).run_step(ROBOT)
        assert ref.completed == 1
        assert task.completed == 1
        assert task.cancelled == 0

    @pytest.mark.asyncio
    async def test_already_finished_task_is_cancelled_harmlessly(self):
        # When the reference finishes after the task already completed, the
        # step still calls task.cancel(); cancelling a done task is a no-op and
        # must NOT raise. (CancelledError is suppressed.)
        ref = FakeStep(sleep=0.02)
        task = FakeStep(sleep=0.0)
        # Should complete cleanly with no exception.
        await DoWhileActive(ref, task).run_step(ROBOT)
        assert task.cancelled == 0  # it had already completed, not cancelled

    @pytest.mark.asyncio
    async def test_instant_reference_still_runs_task_then_cancels(self):
        # Edge: reference finishes essentially immediately. The task is started
        # (both create_task'd) and then cancelled. The task records a start; it
        # may or may not have reached its body, but it must end cancelled (it
        # sleeps long) and never complete.
        ref = FakeStep(sleep=0.0)
        task = FakeStep(sleep=10.0)
        await DoWhileActive(ref, task).run_step(ROBOT)
        assert ref.completed == 1
        assert task.started == 1
        assert task.completed == 0


# --------------------------------------------------------------------------- #
# 4. Reference raising — exception propagation (and the spec's "via exception")
# --------------------------------------------------------------------------- #
class TestReferenceException:
    @pytest.mark.asyncio
    async def test_reference_exception_propagates(self):
        # The reference step's exception must surface out of run_step (it is not
        # swallowed). The exception TYPE is the contract, not its wording.
        ref = FakeStep(raises=ValueError("boom"))
        task = FakeStep(sleep=10.0)
        with pytest.raises(ValueError):
            await DoWhileActive(ref, task).run_step(ROBOT)

    @pytest.mark.asyncio
    async def test_reference_exception_cancels_task(self):
        # Per the docstring, a reference failure must still cancel the task
        # (fixed: the await is wrapped in try/finally so cancel runs on both paths).
        ref = FakeStep(raises=ValueError("boom"))
        task = FakeStep(sleep=10.0)
        with pytest.raises(ValueError):
            await DoWhileActive(ref, task).run_step(ROBOT)
        # Give any scheduled cancellation a chance to run.
        await asyncio.sleep(0)
        assert task.cancelled == 1
        assert task.completed == 0


# --------------------------------------------------------------------------- #
# 5. Outer cancellation — cancelling the whole DoWhileActive
# --------------------------------------------------------------------------- #
class TestOuterCancellation:
    @pytest.mark.asyncio
    async def test_cancel_propagates_to_running_step(self):
        # Cancelling the task that runs DoWhileActive must propagate the
        # CancelledError outward (run_step does not swallow outer cancels).
        ref = FakeStep(sleep=10.0)
        task = FakeStep(sleep=10.0)
        dw = DoWhileActive(ref, task)

        runner = asyncio.create_task(dw.run_step(ROBOT))
        await asyncio.sleep(0.02)  # let both children start
        assert ref.started == 1
        assert task.started == 1

        runner.cancel()
        with pytest.raises(asyncio.CancelledError):
            await runner


# --------------------------------------------------------------------------- #
# 6. DSL metadata (tags carried for documentation grouping)
# --------------------------------------------------------------------------- #
class TestDslMetadata:
    def test_tags(self):
        # First tag = primary category; pair documents grouping contract.
        assert DoWhileActive.__dsl__.tags == ("control", "concurrent")
        assert DoWhileActive.__dsl_step_tags__ == ("control", "concurrent")
        assert DoWhileActive.__dsl_step__ is True
