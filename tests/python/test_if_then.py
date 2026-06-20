"""Spec-driven tests for the ``IfThen`` runtime-branching control step.

``raccoon.step.logic.if_then.IfThen`` evaluates a predicate ``condition``
once at execution time and then runs ``then_step`` (truthy) or ``else_step``
(falsy, optional). It is a composite step: ``collected_resources()`` reports
the *union* of both branches because either may run, and overlapping branch
resources are rejected at construction time via ``validate_no_overlap``.

Every expected value below is derived from the docstring contract, not from
running the code:

* condition is called EXACTLY ONCE, with the robot, when the step executes;
* truthy => then branch only; falsy => else branch only (or nothing);
* union resource collection, with the else branch omitted when absent;
* construction rejects a non-callable condition / non-Step branches;
* children are ``resolve()``-d (builders lowered to real steps) before use.

No hardware / mock bundle is needed: branch steps are fake objects that
satisfy ``StepProtocol`` (``run_step`` / ``required_resources`` /
``collected_resources``) and record whether they ran; the robot is an opaque
sentinel except where the condition reads it.
"""

from __future__ import annotations

import asyncio

import pytest

from raccoon.step import Step, StepProtocol
from raccoon.step.base import _step_path
from raccoon.step.logic import if_then as if_then_factory
from raccoon.step.logic.if_then import IfThen
from raccoon.step.logic.if_then_dsl import IfThenBuilder
from raccoon.step.resource import ResourceConflictError


# ---------------------------------------------------------------------------
# Test doubles
# ---------------------------------------------------------------------------
class FakeStep:
    """A minimal executable step satisfying ``StepProtocol``.

    Records whether ``run_step`` ran and with which robot, declares a set of
    resources, and (importantly) returns *itself* from ``resolve()`` so it can
    be stored by ``IfThen`` like a real concrete step.
    """

    def __init__(self, resources: frozenset[str] = frozenset()) -> None:
        self._resources = resources
        self.ran = False
        self.run_count = 0
        self.seen_robot: object | None = None
        self.path_at_run: list[str] | None = None

    def resolve(self) -> "FakeStep":
        return self

    def required_resources(self) -> frozenset[str]:
        return self._resources

    def collected_resources(self) -> frozenset[str]:
        return self._resources

    async def run_step(self, robot: object) -> None:
        self.ran = True
        self.run_count += 1
        self.seen_robot = robot
        # Capture the contextvar path so we can prove the branch label was
        # pushed before this child executed.
        self.path_at_run = list(_step_path.get())


class RecordingCondition:
    """A callable predicate that records its call count and argument."""

    def __init__(self, result: object) -> None:
        self._result = result
        self.calls = 0
        self.seen_robot: object | None = None

    def __call__(self, robot: object) -> object:
        self.calls += 1
        self.seen_robot = robot
        return self._result


class ResolveTrackingStep(FakeStep):
    """A fake step that records how many times ``resolve()`` was invoked and
    returns a *different* concrete step, mimicking a builder being lowered."""

    def __init__(self, target: FakeStep) -> None:
        super().__init__()
        self._target = target
        self.resolve_calls = 0

    def resolve(self) -> FakeStep:
        self.resolve_calls += 1
        return self._target


SENTINEL_ROBOT = object()


def _run(step: IfThen, robot: object = SENTINEL_ROBOT) -> None:
    """Drive the composite branch logic directly (bypasses timing/run_step).

    NOTE: ``asyncio.run`` executes the coroutine in a *copied* contextvars
    context, so any ``_step_path.set(...)`` mutation inside ``_execute_step``
    is invisible to the caller after this returns. That is fine for the tests
    that only assert which branch ran / what the child observed mid-execution.
    Tests that need to prove the ``finally: _step_path.reset(token)``
    restoration contract MUST instead await ``_execute_step`` directly inside a
    *single* running loop (see ``_run_in_loop_observing_path`` below), where a
    missing reset genuinely leaks into the post-await read.
    """
    asyncio.run(step._execute_step(robot))


def _run_in_loop_observing_path(
    make_step, robot: object = SENTINEL_ROBOT
) -> tuple[list[str], list[str], "BaseException | None"]:
    """Build + execute an IfThen inside ONE running loop and report the path.

    ``make_step`` is a zero-arg callable returning a fresh ``IfThen`` — it is
    invoked inside the loop so that the contextvar read before execution, the
    ``_execute_step`` mutation/reset, and the read after execution all share
    the same (non-copied) context. Returns ``(before, after, raised)`` where
    ``raised`` is the exception that escaped ``_execute_step`` (or ``None``).

    This is the only harness that is mutation-sensitive to deletion of
    ``_step_path.reset(token)``: without the reset, ``after`` retains the
    pushed branch label and diverges from ``before``.
    """

    async def driver():
        step = make_step()
        before = list(_step_path.get())
        raised: BaseException | None = None
        try:
            await step._execute_step(robot)
        except BaseException as exc:
            raised = exc
        after = list(_step_path.get())
        return before, after, raised

    return asyncio.run(driver())


# ---------------------------------------------------------------------------
# 1. Construction validation
# ---------------------------------------------------------------------------
class TestConstructionValidation:
    def test_non_callable_condition_rejected(self):
        with pytest.raises(TypeError, match=r"callable"):
            IfThen(condition=123, then_step=FakeStep())

    def test_non_step_then_rejected(self):
        with pytest.raises(TypeError, match=r"then_step"):
            IfThen(condition=lambda r: True, then_step=object())

    def test_non_step_else_rejected(self):
        with pytest.raises(TypeError, match=r"else_step"):
            IfThen(condition=lambda r: True, then_step=FakeStep(), else_step=object())

    def test_none_else_is_allowed(self):
        step = IfThen(condition=lambda r: True, then_step=FakeStep(), else_step=None)
        assert step.else_step is None

    def test_callable_condition_stored_verbatim(self):
        cond = RecordingCondition(True)
        then = FakeStep()
        step = IfThen(condition=cond, then_step=then)
        assert step.condition is cond

    def test_lambda_passes_callable_check(self):
        # A lambda is callable; construction must succeed.
        step = IfThen(condition=lambda robot: False, then_step=FakeStep())
        assert callable(step.condition)


# ---------------------------------------------------------------------------
# 2. resolve() is applied to children at construction
# ---------------------------------------------------------------------------
class TestChildResolution:
    def test_then_step_is_resolved(self):
        concrete = FakeStep()
        wrapper = ResolveTrackingStep(concrete)
        step = IfThen(condition=lambda r: True, then_step=wrapper)
        assert wrapper.resolve_calls == 1
        # The stored step is the resolved concrete one, not the wrapper.
        assert step.then_step is concrete

    def test_else_step_is_resolved_when_present(self):
        concrete = FakeStep()
        wrapper = ResolveTrackingStep(concrete)
        step = IfThen(condition=lambda r: True, then_step=FakeStep(), else_step=wrapper)
        assert wrapper.resolve_calls == 1
        assert step.else_step is concrete

    def test_else_step_resolve_not_called_when_none(self):
        # No else => resolve() must not be invoked for it (no AttributeError on None).
        step = IfThen(condition=lambda r: True, then_step=FakeStep(), else_step=None)
        assert step.else_step is None


# ---------------------------------------------------------------------------
# 3. Resource collection — union of both branches
# ---------------------------------------------------------------------------
class TestCollectedResources:
    def test_union_of_both_branches(self):
        then = FakeStep(frozenset({"motor:0"}))
        els = FakeStep(frozenset({"servo:3"}))
        step = IfThen(condition=lambda r: True, then_step=then, else_step=els)
        assert step.collected_resources() == frozenset({"motor:0", "servo:3"})

    def test_only_then_when_no_else(self):
        then = FakeStep(frozenset({"drive"}))
        step = IfThen(condition=lambda r: True, then_step=then)
        assert step.collected_resources() == frozenset({"drive"})

    def test_empty_when_neither_branch_has_resources(self):
        step = IfThen(
            condition=lambda r: True,
            then_step=FakeStep(frozenset()),
            else_step=FakeStep(frozenset()),
        )
        assert step.collected_resources() == frozenset()

    def test_overlapping_resources_share_is_deduplicated_in_union(self):
        # Union of identical sets collapses to one entry (set semantics). This
        # only reaches collected_resources when the two branches do NOT both
        # claim — but they DO here, so construction must reject first.
        then = FakeStep(frozenset({"motor:0", "drive"}))
        els = FakeStep(frozenset({"servo:1", "drive"}))
        with pytest.raises(ResourceConflictError):
            IfThen(condition=lambda r: True, then_step=then, else_step=els)


# ---------------------------------------------------------------------------
# 4. Construction rejects overlapping branch resources (validate_no_overlap)
# ---------------------------------------------------------------------------
class TestResourceConflictAtConstruction:
    def test_direct_overlap_raises(self):
        then = FakeStep(frozenset({"drive"}))
        els = FakeStep(frozenset({"drive"}))
        with pytest.raises(ResourceConflictError, match=r"drive"):
            IfThen(condition=lambda r: True, then_step=then, else_step=els)

    def test_wildcard_overlap_raises(self):
        # "servo:*" conflicts with any "servo:N".
        then = FakeStep(frozenset({"servo:*"}))
        els = FakeStep(frozenset({"servo:2"}))
        with pytest.raises(ResourceConflictError, match=r"servo"):
            IfThen(condition=lambda r: True, then_step=then, else_step=els)

    def test_disjoint_resources_construct_fine(self):
        then = FakeStep(frozenset({"motor:0"}))
        els = FakeStep(frozenset({"motor:1"}))
        step = IfThen(condition=lambda r: True, then_step=then, else_step=els)
        assert step.collected_resources() == frozenset({"motor:0", "motor:1"})

    def test_single_branch_never_conflicts(self):
        # With no else there is only one branch — pairwise check is a no-op
        # even if that branch claims many resources.
        then = FakeStep(frozenset({"drive", "servo:*", "motor:0"}))
        step = IfThen(condition=lambda r: True, then_step=then)
        assert step.collected_resources() == frozenset({"drive", "servo:*", "motor:0"})

    def test_conflict_keys_off_collected_not_required_resources(self):
        # validate_no_overlap (resource.py) iterates collected_resources(),
        # NOT required_resources(). Use a double whose two accessors return
        # DIFFERENT sets to pin which one the conflict check consults: the
        # overlap lives only in collected_resources, so construction must
        # reject; if the source erroneously read required_resources (which is
        # disjoint here) it would NOT reject and this test would fail.
        class SplitResourceStep(FakeStep):
            def __init__(self, required: frozenset[str], collected: frozenset[str]):
                super().__init__()
                self._required = required
                self._collected = collected

            def required_resources(self) -> frozenset[str]:
                return self._required

            def collected_resources(self) -> frozenset[str]:
                return self._collected

        then = SplitResourceStep(required=frozenset({"motor:0"}), collected=frozenset({"drive"}))
        els = SplitResourceStep(required=frozenset({"motor:1"}), collected=frozenset({"drive"}))
        with pytest.raises(ResourceConflictError, match=r"drive"):
            IfThen(condition=lambda r: True, then_step=then, else_step=els)

    def test_required_resources_overlap_alone_does_not_conflict(self):
        # The mirror image: required_resources overlap ("motor:0") but
        # collected_resources are disjoint. Since the check reads
        # collected_resources, construction must SUCCEED.
        class SplitResourceStep(FakeStep):
            def __init__(self, required: frozenset[str], collected: frozenset[str]):
                super().__init__()
                self._required = required
                self._collected = collected

            def required_resources(self) -> frozenset[str]:
                return self._required

            def collected_resources(self) -> frozenset[str]:
                return self._collected

        then = SplitResourceStep(required=frozenset({"motor:0"}), collected=frozenset({"servo:1"}))
        els = SplitResourceStep(required=frozenset({"motor:0"}), collected=frozenset({"servo:2"}))
        step = IfThen(condition=lambda r: True, then_step=then, else_step=els)
        assert step.collected_resources() == frozenset({"servo:1", "servo:2"})


# ---------------------------------------------------------------------------
# 5. Branch selection — the core contract
# ---------------------------------------------------------------------------
class TestBranchSelection:
    def test_truthy_runs_then_only(self):
        then = FakeStep()
        els = FakeStep()
        step = IfThen(condition=RecordingCondition(True), then_step=then, else_step=els)
        _run(step)
        assert then.ran is True
        assert els.ran is False

    def test_falsy_runs_else_only(self):
        then = FakeStep()
        els = FakeStep()
        step = IfThen(condition=RecordingCondition(False), then_step=then, else_step=els)
        _run(step)
        assert then.ran is False
        assert els.ran is True

    def test_falsy_without_else_runs_nothing(self):
        then = FakeStep()
        step = IfThen(condition=RecordingCondition(False), then_step=then)
        # Must complete without error and run neither branch.
        _run(step)
        assert then.ran is False

    def test_truthy_without_else_runs_then(self):
        then = FakeStep()
        step = IfThen(condition=RecordingCondition(True), then_step=then)
        _run(step)
        assert then.ran is True

    def test_condition_called_exactly_once(self):
        cond = RecordingCondition(True)
        step = IfThen(condition=cond, then_step=FakeStep(), else_step=FakeStep())
        _run(step)
        assert cond.calls == 1

    def test_condition_called_once_on_false_branch(self):
        cond = RecordingCondition(False)
        step = IfThen(condition=cond, then_step=FakeStep(), else_step=FakeStep())
        _run(step)
        assert cond.calls == 1

    def test_condition_receives_robot(self):
        cond = RecordingCondition(True)
        step = IfThen(condition=cond, then_step=FakeStep())
        robot = object()
        _run(step, robot)
        assert cond.seen_robot is robot

    def test_branch_receives_robot(self):
        then = FakeStep()
        step = IfThen(condition=RecordingCondition(True), then_step=then)
        robot = object()
        _run(step, robot)
        assert then.seen_robot is robot

    @pytest.mark.parametrize("truthy_value", [1, "x", [0], object(), 0.1, -1])
    def test_non_bool_truthy_values_take_then(self, truthy_value):
        # "returns truthy" — any truthy value, not just True, picks then.
        then = FakeStep()
        els = FakeStep()
        step = IfThen(condition=RecordingCondition(truthy_value), then_step=then, else_step=els)
        _run(step)
        assert then.ran is True
        assert els.ran is False

    @pytest.mark.parametrize("falsy_value", [0, "", [], None, 0.0, False])
    def test_non_bool_falsy_values_take_else(self, falsy_value):
        then = FakeStep()
        els = FakeStep()
        step = IfThen(condition=RecordingCondition(falsy_value), then_step=then, else_step=els)
        _run(step)
        assert then.ran is False
        assert els.ran is True

    def test_condition_reading_state_selects_branch(self):
        # The predicate may read robot state computed earlier; prove dynamic
        # dispatch by switching the robot attribute between two runs.
        then = FakeStep()
        els = FakeStep()

        class Bot:
            def __init__(self, v):
                self.value = v

        step_true = IfThen(condition=lambda r: r.value > 5, then_step=then, else_step=els)
        _run(step_true, Bot(10))
        assert then.ran and not els.ran

        then2 = FakeStep()
        els2 = FakeStep()
        step_false = IfThen(condition=lambda r: r.value > 5, then_step=then2, else_step=els2)
        _run(step_false, Bot(1))
        assert els2.ran and not then2.ran


# ---------------------------------------------------------------------------
# 6. Step-path labelling for the chosen branch
# ---------------------------------------------------------------------------
class TestStepPathLabelling:
    def test_then_branch_pushes_label(self):
        then = FakeStep()
        step = IfThen(condition=RecordingCondition(True), then_step=then)
        _run(step)
        # A branch label was pushed onto the path before the child ran.
        assert then.path_at_run is not None
        assert any("then" in seg.lower() for seg in then.path_at_run)

    def test_else_branch_pushes_label(self):
        els = FakeStep()
        step = IfThen(condition=RecordingCondition(False), then_step=FakeStep(), else_step=els)
        _run(step)
        assert els.path_at_run is not None
        assert any("else" in seg.lower() for seg in els.path_at_run)

    def test_path_restored_after_then_branch(self):
        # The contextvar must be reset to its prior value after execution.
        # Driven inside a single running loop so a missing reset would LEAK
        # the "If[then]" label into the post-await read (mutation-sensitive).
        before, after, raised = _run_in_loop_observing_path(
            lambda: IfThen(condition=RecordingCondition(True), then_step=FakeStep())
        )
        assert raised is None
        assert after == before

    def test_path_restored_after_else_branch(self):
        before, after, raised = _run_in_loop_observing_path(
            lambda: IfThen(
                condition=RecordingCondition(False),
                then_step=FakeStep(),
                else_step=FakeStep(),
            )
        )
        assert raised is None
        assert after == before

    def test_path_restored_even_if_branch_raises(self):
        # finally block resets the token despite the exception.
        class Boom(FakeStep):
            async def run_step(self, robot):
                raise RuntimeError("branch failed")

        before, after, raised = _run_in_loop_observing_path(
            lambda: IfThen(condition=RecordingCondition(True), then_step=Boom())
        )
        assert isinstance(raised, RuntimeError)
        assert "branch failed" in str(raised)
        assert after == before

    def test_path_not_pushed_when_condition_itself_raises(self):
        # The predicate runs BEFORE any _push_path; if it raises, no branch
        # label was pushed and the path is unchanged. Neither branch runs.
        then = FakeStep()
        els = FakeStep()

        class Boom(Exception):
            pass

        def exploding(_robot):
            raise Boom("predicate blew up")

        before, after, raised = _run_in_loop_observing_path(
            lambda: IfThen(condition=exploding, then_step=then, else_step=els)
        )
        assert isinstance(raised, Boom)
        assert after == before
        assert then.ran is False
        assert els.ran is False


# ---------------------------------------------------------------------------
# 7. Async lifecycle / cancellation
# ---------------------------------------------------------------------------
class TestAsyncLifecycle:
    def test_execute_step_is_awaitable_and_returns_none(self):
        step = IfThen(condition=RecordingCondition(True), then_step=FakeStep())
        result = asyncio.run(step._execute_step(SENTINEL_ROBOT))
        assert result is None

    def test_cancellation_propagates_from_branch(self):
        # CancelledError from the branch must propagate AND the finally block
        # must still reset the path. Driven inside one loop so the reset is
        # observable (mutation-sensitive to deleting _step_path.reset).
        class Cancelling(FakeStep):
            async def run_step(self, robot):
                raise asyncio.CancelledError

        before, after, raised = _run_in_loop_observing_path(
            lambda: IfThen(condition=RecordingCondition(True), then_step=Cancelling())
        )
        assert isinstance(raised, asyncio.CancelledError)
        assert after == before

    def test_branch_step_actually_awaited(self):
        # An async branch that yields control still completes before the
        # composite returns (run_count incremented exactly once).
        then = FakeStep()
        step = IfThen(condition=RecordingCondition(True), then_step=then)

        class Yielding(FakeStep):
            async def run_step(self, robot):
                await asyncio.sleep(0)
                await super().run_step(robot)

        y = Yielding()
        step2 = IfThen(condition=RecordingCondition(True), then_step=y)
        _run(step2)
        assert y.run_count == 1


# ---------------------------------------------------------------------------
# 8. Composite metadata / signature contract
# ---------------------------------------------------------------------------
class TestStepMetadata:
    def test_is_composite(self):
        # Composite steps don't acquire hardware themselves; the flag drives
        # run_step's resource handling and logging branch.
        assert IfThen._composite is True

    def test_is_a_step(self):
        step = IfThen(condition=lambda r: True, then_step=FakeStep())
        assert isinstance(step, Step)

    def test_signature_is_a_string(self):
        # Wording is not pinned (prose); only that it returns a str.
        sig = IfThen(condition=lambda r: True, then_step=FakeStep())._generate_signature()
        assert isinstance(sig, str)
        assert sig != ""

    def test_dsl_tags_on_class(self):
        # Tag tuple is a documentation grouping contract.
        assert IfThen.__dsl_step_tags__ == ("control", "logic")
        assert IfThen.__dsl_step__ is True

    def test_base_class_is_hidden(self):
        # @dsl_step applies hidden=True so users go through the factory.
        assert IfThen.__dsl_hidden__ is True


# ---------------------------------------------------------------------------
# 9. Public factory / builder wiring
# ---------------------------------------------------------------------------
class TestFactory:
    def test_factory_returns_builder(self):
        b = if_then_factory(lambda r: True, FakeStep())
        assert isinstance(b, IfThenBuilder)

    def test_builder_builds_ifthen_with_branches(self):
        then = FakeStep()
        els = FakeStep()
        b = if_then_factory(RecordingCondition(True), then, els)
        built = b._build()
        assert isinstance(built, IfThen)
        assert built.then_step is then
        assert built.else_step is els

    def test_builder_default_else_is_none(self):
        b = if_then_factory(lambda r: True, FakeStep())
        built = b._build()
        assert built.else_step is None

    def test_builder_fluent_methods_set_fields(self):
        then = FakeStep()
        els = FakeStep()
        cond = RecordingCondition(False)
        built = IfThenBuilder().condition(cond).then_step(then).else_step(els)._build()
        assert built.condition is cond
        assert built.then_step is then
        assert built.else_step is els

    def test_builder_built_step_executes_correct_branch(self):
        then = FakeStep()
        els = FakeStep()
        built = if_then_factory(RecordingCondition(False), then, els)._build()
        _run(built)
        assert els.ran and not then.ran

    def test_real_stepbuilder_branch_is_lowered_and_executed(self):
        # A real StepBuilder (not a FakeStep) passed as then/else must be
        # resolve()-d to its underlying step at construction, and that
        # underlying step must run when the branch is selected. This exercises
        # the genuine StepBuilder.resolve() -> _build() path end-to-end, not a
        # FakeStep stand-in.
        from raccoon.step.step_builder import StepBuilder

        then_target = FakeStep(frozenset({"motor:0"}))
        else_target = FakeStep(frozenset({"servo:1"}))

        class _LoweringBuilder(StepBuilder):
            def __init__(self, target: FakeStep):
                super().__init__()
                self._target = target

            def _build(self) -> object:
                return self._target

        then_builder = _LoweringBuilder(then_target)
        else_builder = _LoweringBuilder(else_target)

        step = IfThen(
            condition=RecordingCondition(True),
            then_step=then_builder,
            else_step=else_builder,
        )
        # Stored steps are the lowered targets, not the builders.
        assert step.then_step is then_target
        assert step.else_step is else_target
        # Union resources came from the resolved targets, not empty builders.
        assert step.collected_resources() == frozenset({"motor:0", "servo:1"})

        _run(step)
        assert then_target.ran is True
        assert else_target.ran is False


# ---------------------------------------------------------------------------
# 10. StepProtocol conformance of the fake (sanity for our doubles)
# ---------------------------------------------------------------------------
def test_fake_step_satisfies_step_protocol():
    assert isinstance(FakeStep(), StepProtocol)
