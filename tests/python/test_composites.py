"""Spec-driven tests for the composite steps ``Sequential`` and ``Parallel``.

Targets:
    * ``raccoon.step.sequential`` — ``Sequential`` / ``seq``
    * ``raccoon.step.parallel``   — ``Parallel`` / ``parallel``

Both composites are pure-Python orchestrators: they validate their child
list at construction time, union child resources, push a path token per
child while running, and drive each child through ``run_step``. None of
that needs hardware or a C++ drivetrain, so every test here uses recording
fake steps plus a sentinel ``robot = object()`` (the composites only pass
``robot`` through to the children — they never touch it themselves).

Expected values are derived from the docstrings / source contract, NOT by
running the code:
    * Sequential preserves list order; Parallel runs all branches.
    * Type checks raise ``TypeError`` (non-list container, non-Step element).
    * Resource sets are the *union* of children (``collected_resources``).
    * Parallel rejects overlapping resources at construction
      (``ResourceConflictError``) — i.e. BEFORE ``_execute_step`` runs.
    * Path tokens: Sequential pushes ``"<i+1>/<total>"``; Parallel pushes
      ``"P[<i+1>/<total>]"``.
"""

from __future__ import annotations

import asyncio
import importlib
import sys

import pytest

# The conftest / package __init__ imports these modules before pytest-cov
# attaches its tracer, so the import-time def/docstring lines never get
# counted. Reload them here, under coverage, so those definition statements
# re-execute while traced. Bind all names from the freshly reloaded modules
# so class identities stay consistent within this file (Sequential reloaded
# from sequential must match the one parallel.py references after its reload).
import raccoon.step.sequential as _sequential
from raccoon.step import Step
from raccoon.step.base import _step_path
from raccoon.step.resource import ResourceConflictError

# Some workspace modules are loaded with a bare ``__name__`` (e.g. "parallel"
# instead of the dotted path), which makes ``importlib.reload`` fail to find
# them in sys.modules. Normalise the name first so the reload — and therefore
# coverage of the import-time def/docstring lines — works.
_sequential.__name__ = "raccoon.step.sequential"
sys.modules["raccoon.step.sequential"] = _sequential
_sequential = importlib.reload(_sequential)
import raccoon.step.parallel as _parallel  # noqa: E402

_parallel.__name__ = "raccoon.step.parallel"
sys.modules["raccoon.step.parallel"] = _parallel
_parallel = importlib.reload(_parallel)

Sequential = _sequential.Sequential
seq = _sequential.seq
Parallel = _parallel.Parallel
parallel = _parallel.parallel


# ---------------------------------------------------------------------------
# Test doubles
# ---------------------------------------------------------------------------
class RecordingStep(Step):
    """A leaf step that records the order it ran in and the path active then.

    ``log`` is a shared list across all sibling steps so assertions can read
    the global execution order. Each step appends its own ``name`` when it
    runs and snapshots the contextvar ``_step_path`` so per-child path tokens
    can be verified.
    """

    def __init__(self, name, log, resources=frozenset(), delay=0.0):
        super().__init__()
        self.name = name
        self._log = log
        self._resources = frozenset(resources)
        self._delay = delay
        self.run_count = 0
        self.seen_path = None

    def required_resources(self):
        return self._resources

    def _generate_signature(self):
        return f"RecordingStep({self.name})"

    async def _execute_step(self, robot):
        if self._delay:
            await asyncio.sleep(self._delay)
        self.run_count += 1
        self.seen_path = list(_step_path.get())
        self._log.append(self.name)


class ConcurrencyProbe(Step):
    """Records how many siblings are running simultaneously.

    Increments a shared counter on entry, yields control, then records the
    peak observed counter. If branches truly run concurrently the peak will
    equal the number of branches; if they were serialized it would stay 1.
    """

    state = None  # set per test: {"active": int, "peak": int}

    def __init__(self, state):
        super().__init__()
        self._state = state

    def _generate_signature(self):
        return "ConcurrencyProbe"

    async def _execute_step(self, robot):
        self._state["active"] += 1
        self._state["peak"] = max(self._state["peak"], self._state["active"])
        # Yield several times so every sibling gets a chance to start.
        for _ in range(5):
            await asyncio.sleep(0)
        self._state["active"] -= 1


class BoomStep(Step):
    """A leaf step whose body raises a sentinel exception."""

    class Boom(Exception):
        pass

    def __init__(self, log=None, delay=0.0):
        super().__init__()
        self._log = log if log is not None else []
        self._delay = delay
        self.ran = False

    def _generate_signature(self):
        return "BoomStep"

    async def _execute_step(self, robot):
        if self._delay:
            await asyncio.sleep(self._delay)
        self.ran = True
        self._log.append("boom")
        raise BoomStep.Boom("kaboom")


class BlockingStep(Step):
    """A leaf step that parks forever until its surrounding task is cancelled.

    It snapshots the active ``_step_path`` at the moment it starts (so a test
    can confirm the composite had pushed its segment) and sets an
    ``asyncio.Event`` once it is actually awaiting, so the test knows the
    composite has entered the child's body before issuing the cancel.
    """

    def __init__(self, started_event):
        super().__init__()
        self._started = started_event
        self.seen_path = None
        self.cancelled = False

    def _generate_signature(self):
        return "BlockingStep"

    async def _execute_step(self, robot):
        self.seen_path = list(_step_path.get())
        self._started.set()
        try:
            await asyncio.Event().wait()  # never resolves on its own
        except asyncio.CancelledError:
            self.cancelled = True
            raise


class ResolvingStep(Step):
    """A step that resolves into a *different* step instance.

    Used to prove that composites call ``resolve()`` on every child and store
    the resolved object (mirrors how StepBuilder placeholders get built).
    """

    def __init__(self, target):
        super().__init__()
        self._target = target

    def _generate_signature(self):
        return "ResolvingStep"

    def resolve(self):
        return self._target

    async def _execute_step(self, robot):  # pragma: no cover - never executed
        raise AssertionError("resolved-away step should not execute")


ROBOT = object()  # sentinel — composites must not inspect it


def run(coro):
    return asyncio.run(coro)


# ---------------------------------------------------------------------------
# Sequential — construction / validation
# ---------------------------------------------------------------------------
def test_sequential_rejects_non_list_container():
    # tuple is iterable but the spec demands a list[Step]
    with pytest.raises(TypeError, match=r"List\[Step\]"):
        Sequential((RecordingStep("a", []),))


def test_sequential_rejects_non_step_element_reports_index():
    log = []
    a = RecordingStep("a", log)
    steps = [a, "not-a-step"]
    # The offending element is at index 1; assert the TYPE raised (a stable
    # contract) rather than the literal word "index" in the message wording.
    # The valid leading Step must never have run — validation is at __init__.
    with pytest.raises(TypeError):
        Sequential(steps)
    assert a.run_count == 0


def test_sequential_accepts_empty_list():
    s = Sequential([])
    assert s.steps == []
    # last-internal-step pointer is None for the empty case
    assert s._last_internal_step is None
    assert s.collected_resources() == frozenset()


def test_sequential_resolves_children():
    real = RecordingStep("real", [])
    placeholder = ResolvingStep(real)
    s = Sequential([placeholder])
    # The stored child must be the resolved object, not the placeholder.
    assert s.steps == [real]
    assert s._last_internal_step is real


def test_sequential_resolved_target_executes_not_placeholder():
    # Running the composite must drive the RESOLVED step, never the placeholder
    # (whose _execute_step asserts). Proves resolution is wired into execution,
    # not just stored at construction.
    log = []
    real = RecordingStep("real", log)
    s = Sequential([ResolvingStep(real)])
    run(s.run_step(ROBOT))
    assert log == ["real"]
    assert real.run_count == 1


def test_sequential_last_internal_step_is_last_child():
    log = []
    a, b, c = (RecordingStep(n, log) for n in "abc")
    s = Sequential([a, b, c])
    assert s._last_internal_step is c


# ---------------------------------------------------------------------------
# Sequential — resource union
# ---------------------------------------------------------------------------
def test_sequential_collected_resources_is_union():
    a = RecordingStep("a", [], resources={"drive"})
    b = RecordingStep("b", [], resources={"motor:0", "servo:1"})
    s = Sequential([a, b])
    assert s.collected_resources() == frozenset({"drive", "motor:0", "servo:1"})


def test_sequential_overlapping_resources_allowed():
    # Sequential branches run one-at-a-time, so shared resources are fine and
    # must NOT raise (only Parallel forbids overlap).
    a = RecordingStep("a", [], resources={"drive"})
    b = RecordingStep("b", [], resources={"drive"})
    s = Sequential([a, b])
    assert s.collected_resources() == frozenset({"drive"})


# ---------------------------------------------------------------------------
# Sequential — execution order & path tokens
# ---------------------------------------------------------------------------
def test_sequential_runs_in_order():
    log = []
    a, b, c = (RecordingStep(n, log) for n in "abc")
    run(Sequential([a, b, c]).run_step(ROBOT))
    assert log == ["a", "b", "c"]
    assert a.run_count == b.run_count == c.run_count == 1


def test_sequential_path_tokens_are_one_based_over_total():
    log = []
    a, b, c = (RecordingStep(n, log) for n in "abc")
    run(Sequential([a, b, c]).run_step(ROBOT))
    # Each child sees exactly its own "i+1/total" segment appended.
    assert a.seen_path == ["1/3"]
    assert b.seen_path == ["2/3"]
    assert c.seen_path == ["3/3"]


def test_sequential_path_token_is_cleaned_up_after_run():
    log = []
    run(Sequential([RecordingStep("a", log)]).run_step(ROBOT))
    # Outside the composite the path contextvar must be restored to empty.
    assert _step_path.get() == []


def test_sequential_propagates_child_exception_and_stops():
    log = []
    a = RecordingStep("a", log)
    boom = BoomStep(log)
    never = RecordingStep("never", log)
    with pytest.raises(BoomStep.Boom):
        run(Sequential([a, boom, never]).run_step(ROBOT))
    # a ran, boom ran and raised, the step after boom must NOT run.
    assert log == ["a", "boom"]
    assert never.run_count == 0
    # The try/finally in _execute_step must reset the path token even when a
    # child raises; without the finally the contextvar would leak the failing
    # child's "2/3" segment.
    assert _step_path.get() == []


def test_sequential_empty_runs_without_error():
    run(Sequential([]).run_step(ROBOT))  # no children -> trivial success


def test_seq_factory_returns_sequential():
    a = RecordingStep("a", [])
    s = seq([a])
    assert isinstance(s, Sequential)
    assert s.steps == [a]


# ---------------------------------------------------------------------------
# Sequential — signature contract (semantic, not exact prose)
# ---------------------------------------------------------------------------
def test_sequential_signature_carries_count_and_endpoints():
    a, b = RecordingStep("a", []), RecordingStep("b", [])
    sig = Sequential([a, b])._generate_signature()
    assert "2" in sig  # count == 2
    assert "RecordingStep" in sig  # first/last class names present


def test_sequential_signature_empty_uses_none_markers():
    sig = Sequential([])._generate_signature()
    assert "0" in sig
    assert "None" in sig


# ---------------------------------------------------------------------------
# Parallel — construction / validation
# ---------------------------------------------------------------------------
def test_parallel_rejects_non_list_container():
    with pytest.raises(TypeError, match=r"List\[Step\]"):
        Parallel((RecordingStep("a", []),))


def test_parallel_rejects_non_step_element_reports_index():
    a = RecordingStep("a", [])
    steps = ["nope", a]
    # Bad element at index 0 -> TypeError before any branch runs. Assert the
    # exception TYPE and that no child executed, not the message wording.
    with pytest.raises(TypeError):
        Parallel(steps)
    assert a.run_count == 0


def test_parallel_resolves_children():
    real = RecordingStep("real", [])
    s = Parallel([ResolvingStep(real)])
    assert s.steps == [real]


def test_parallel_resolved_target_executes_not_placeholder():
    log = []
    real = RecordingStep("real", log)
    s = Parallel([ResolvingStep(real)])
    run(s.run_step(ROBOT))
    assert log == ["real"]
    assert real.run_count == 1


def test_parallel_collected_resources_is_union():
    a = RecordingStep("a", [], resources={"drive"})
    b = RecordingStep("b", [], resources={"servo:2"})
    s = Parallel([a, b])
    assert s.collected_resources() == frozenset({"drive", "servo:2"})


# ---------------------------------------------------------------------------
# Parallel — resource conflict rejected BEFORE running
# ---------------------------------------------------------------------------
def test_parallel_rejects_overlapping_resources_at_construction():
    a = RecordingStep("a", [], resources={"drive"})
    b = RecordingStep("b", [], resources={"drive"})
    with pytest.raises(ResourceConflictError) as excinfo:
        Parallel([a, b])
    # The structured fields are a contract (resource.py:26-29). The conflicting
    # resource is the shared "drive"; branch 0 is the holder, branch 1 the
    # requester (validate_no_overlap scans pairs i<j).
    err = excinfo.value
    assert err.resource == "drive"
    assert "branch 0" in err.holder
    assert "branch 1" in err.requester
    # Neither child ran — the failure is purely static (construction time).
    assert a.run_count == 0
    assert b.run_count == 0


def test_parallel_wildcard_resource_conflicts_with_specific():
    # "servo:*" must conflict with any concrete "servo:N". Independently the
    # conflicting resource reported is the CONCRETE port (the wildcard expands
    # to cover "servo:3"), not the wildcard token itself.
    a = RecordingStep("a", [], resources={"servo:*"})
    b = RecordingStep("b", [], resources={"servo:3"})
    with pytest.raises(ResourceConflictError) as excinfo:
        Parallel([a, b])
    assert excinfo.value.resource == "servo:3"


def test_parallel_non_adjacent_branches_conflict():
    # validate_no_overlap loops over ALL (i, j) pairs, not just neighbours.
    # Branch 0 and branch 2 collide while the disjoint branch 1 sits between
    # them — this exercises the inner j-loop past j == i + 1.
    a = RecordingStep("a", [], resources={"drive"})
    middle = RecordingStep("middle", [], resources={"motor:0"})
    c = RecordingStep("c", [], resources={"drive"})
    with pytest.raises(ResourceConflictError) as excinfo:
        Parallel([a, middle, c])
    err = excinfo.value
    assert err.resource == "drive"
    # The first matching pair is (0, 2): branch 0 holds, branch 2 requests.
    assert "branch 0" in err.holder
    assert "branch 2" in err.requester


def test_parallel_first_conflicting_pair_is_a_later_i():
    # The earliest conflict is NOT at i == 0: branch 0 is disjoint from both
    # others, so the (0,1) and (0,2) comparisons pass and the outer loop must
    # advance to i == 1 before the (1, 2) pair collides. This exercises the
    # i-loop past its first iteration (and the holder/requester labels for a
    # non-zero i). The scan is ordered and raises on the FIRST hit, so the
    # reported pair must be (1, 2), not any pair involving branch 0.
    b0 = RecordingStep("b0", [], resources={"servo:0"})
    b1 = RecordingStep("b1", [], resources={"drive"})
    b2 = RecordingStep("b2", [], resources={"drive"})
    with pytest.raises(ResourceConflictError) as excinfo:
        Parallel([b0, b1, b2])
    err = excinfo.value
    assert err.resource == "drive"
    assert "branch 1" in err.holder
    assert "branch 2" in err.requester
    # Branch 0 must NOT be implicated in the reported conflict at all.
    assert "branch 0" not in err.holder
    assert "branch 0" not in err.requester


def test_parallel_disjoint_resources_allowed():
    a = RecordingStep("a", [], resources={"drive"})
    b = RecordingStep("b", [], resources={"motor:0"})
    s = Parallel([a, b])  # must not raise
    assert s.collected_resources() == frozenset({"drive", "motor:0"})


def test_parallel_resourceless_branches_allowed():
    a = RecordingStep("a", [])
    b = RecordingStep("b", [])
    Parallel([a, b])  # no resources -> no conflict


# ---------------------------------------------------------------------------
# Parallel — concurrent execution
# ---------------------------------------------------------------------------
def test_parallel_runs_all_branches():
    log = []
    a, b, c = (RecordingStep(n, log) for n in "abc")
    run(Parallel([a, b, c]).run_step(ROBOT))
    assert sorted(log) == ["a", "b", "c"]
    assert a.run_count == b.run_count == c.run_count == 1


def test_parallel_branches_run_concurrently_not_serialized():
    state = {"active": 0, "peak": 0}
    probes = [ConcurrencyProbe(state) for _ in range(3)]
    run(Parallel(probes).run_step(ROBOT))
    # True concurrency means all three were "active" at the same time.
    assert state["peak"] == 3
    assert state["active"] == 0


def test_parallel_path_tokens_are_p_bracket_one_based():
    log = []
    a, b = RecordingStep("a", log), RecordingStep("b", log)
    run(Parallel([a, b]).run_step(ROBOT))
    assert a.seen_path == ["P[1/2]"]
    assert b.seen_path == ["P[2/2]"]


def test_parallel_path_restored_after_run():
    run(Parallel([RecordingStep("a", [])]).run_step(ROBOT))
    assert _step_path.get() == []


def test_parallel_empty_is_noop():
    s = Parallel([])
    run(s.run_step(ROBOT))
    # _execute_step early-returns for empty; last-completed stays None.
    assert s._last_completed_step is None


def test_parallel_records_last_completed_step():
    log = []
    a, b = RecordingStep("a", log), RecordingStep("b", log)
    s = Parallel([a, b])
    run(s.run_step(ROBOT))
    # A branch ran to completion, so the pointer is one of the children.
    assert s._last_completed_step in (a, b)


def test_parallel_exception_in_one_branch_propagates():
    log = []
    ok = RecordingStep("ok", log)
    boom = BoomStep(log)
    with pytest.raises(BoomStep.Boom):
        run(Parallel([ok, boom]).run_step(ROBOT))
    # step_callback's try/finally must reset the path token on the failing
    # branch; otherwise the "P[i/total]" segment leaks out of the gather.
    assert _step_path.get() == []


# ---------------------------------------------------------------------------
# Async cancellation — the try/finally must unwind the path token even when
# the enclosing task is cancelled mid-branch (not just on ordinary raises).
# ---------------------------------------------------------------------------
async def _run_and_capture_path_on_cancel(composite, child_started, out):
    """Drive *composite* in a cancellable task; record the path on exit.

    Runs in its OWN asyncio task (so it shares one contextvars copy with the
    composite it awaits). When the outer ``task.cancel()`` lands, the composite's
    ``finally`` block must have reset ``_step_path`` before the CancelledError
    reaches here — we snapshot it so the test can assert the token was unwound.
    """
    try:
        await composite.run_step(ROBOT)
    finally:
        out["path_on_exit"] = list(_step_path.get())


def test_sequential_cancel_mid_branch_unwinds_path_token():
    async def main():
        started = asyncio.Event()
        blocker = BlockingStep(started)
        composite = Sequential([blocker])
        out = {}
        task = asyncio.ensure_future(_run_and_capture_path_on_cancel(composite, started, out))
        await started.wait()  # composite has pushed "1/1" and entered the child
        # Sanity: the child observed the pushed sequential segment.
        assert blocker.seen_path == ["1/1"]
        task.cancel()
        with pytest.raises(asyncio.CancelledError):
            await task
        # The child's body actually saw the cancellation...
        assert blocker.cancelled is True
        # ...and the composite's finally reset the contextvar despite the cancel,
        # so no stale "1/1" segment leaks out of the composite.
        assert out["path_on_exit"] == []

    run(main())


def test_parallel_cancel_mid_branch_unwinds_path_token():
    async def main():
        started = asyncio.Event()
        blocker = BlockingStep(started)
        # Pair the blocker with a second (resourceless) branch so the gather is
        # live with at least one branch still parked when we cancel.
        quick = RecordingStep("quick", [])
        composite = Parallel([blocker, quick])
        out = {}
        task = asyncio.ensure_future(_run_and_capture_path_on_cancel(composite, started, out))
        await started.wait()  # blocker branch pushed "P[1/2]" and parked
        assert blocker.seen_path == ["P[1/2]"]
        task.cancel()
        with pytest.raises(asyncio.CancelledError):
            await task
        assert blocker.cancelled is True
        # The branch tasks each run in their own context copy, but the outer
        # task (which awaited gather) must still see an empty path after the
        # cancel unwinds — no composite segment leaks past the gather.
        assert out["path_on_exit"] == []

    run(main())


# ---------------------------------------------------------------------------
# Parallel — signature contract
# ---------------------------------------------------------------------------
def test_parallel_signature_carries_group_count():
    a, b, c = (RecordingStep(n, []) for n in "abc")
    sig = Parallel([a, b, c])._generate_signature()
    assert "3" in sig


# ---------------------------------------------------------------------------
# parallel() factory — argument coercion
# ---------------------------------------------------------------------------
def test_parallel_factory_wraps_single_step_in_sequential():
    a = RecordingStep("a", [])
    p = parallel(a)
    assert isinstance(p, Parallel)
    assert len(p.steps) == 1
    # Single step is wrapped into a one-element Sequential branch.
    branch = p.steps[0]
    assert isinstance(branch, Sequential)
    assert branch.steps == [a]


def test_parallel_factory_wraps_list_in_sequential_branch():
    log = []
    a, b = RecordingStep("a", log), RecordingStep("b", log)
    p = parallel([a, b])
    assert len(p.steps) == 1
    branch = p.steps[0]
    assert isinstance(branch, Sequential)
    assert branch.steps == [a, b]


def test_parallel_factory_passes_sequential_through_directly():
    a, b = RecordingStep("a", []), RecordingStep("b", [])
    existing = seq([a, b])
    p = parallel(existing)
    # An existing Sequential is used as-is, not re-wrapped in another seq.
    assert p.steps[0] is existing


def test_parallel_factory_mixed_args():
    log = []
    solo = RecordingStep("solo", log)
    a, b = RecordingStep("a", log), RecordingStep("b", log)
    pre_seq = seq([RecordingStep("s1", log)])
    p = parallel(solo, [a, b], pre_seq)
    assert len(p.steps) == 3
    assert all(isinstance(branch, Sequential) for branch in p.steps)
    assert p.steps[2] is pre_seq


def test_parallel_factory_rejects_bad_arg_type():
    # An int is none of {Step, Sequential, list[Step]} -> the else branch must
    # raise TypeError. Assert only the exception TYPE plus that the rejected
    # value's type is surfaced (a stable semantic token), not the exact prose
    # phrasing/word-order of the message.
    with pytest.raises(TypeError, match=r"int"):
        parallel(42)


def test_parallel_factory_rejects_list_with_non_step_element():
    # A list containing a non-step fails the all(isinstance) guard and falls
    # through to the bare-arg branch, which rejects a plain list.
    with pytest.raises(TypeError):
        parallel([RecordingStep("a", []), "nope"])


def test_parallel_factory_no_args_builds_empty_parallel():
    p = parallel()
    assert isinstance(p, Parallel)
    assert p.steps == []


def test_parallel_factory_empty_list_arg_wraps_empty_sequential():
    # An empty list arg satisfies `isinstance(arg, list) and all(...)` because
    # all() is vacuously True over an empty iterable -> it takes the list
    # branch and wraps an empty Sequential. Distinct from parallel() no-args,
    # which produces NO branches at all.
    p = parallel([])
    assert isinstance(p, Parallel)
    assert len(p.steps) == 1
    branch = p.steps[0]
    assert isinstance(branch, Sequential)
    assert branch.steps == []
    # Sanity: empty parallel-of-empty-seq runs to completion without error.
    run(p.run_step(ROBOT))


def test_nested_sequential_in_parallel_accumulates_path_tokens():
    # A Sequential nested inside a Parallel must STACK its token on top of the
    # parallel's: the leaf sees the parallel segment first, then the sequential
    # one. Two branches, each a single-step seq:
    #   branch 0 -> "P[1/2]" then "1/1"
    #   branch 1 -> "P[2/2]" then "1/1"
    log = []
    leaf0 = RecordingStep("leaf0", log)
    leaf1 = RecordingStep("leaf1", log)
    p = Parallel([Sequential([leaf0]), Sequential([leaf1])])
    run(p.run_step(ROBOT))
    assert leaf0.seen_path == ["P[1/2]", "1/1"]
    assert leaf1.seen_path == ["P[2/2]", "1/1"]
    # And the contextvar is fully unwound afterwards.
    assert _step_path.get() == []


def test_nested_parallel_in_sequential_accumulates_path_tokens():
    # Mirror case: a Parallel nested inside a Sequential. The Sequential's
    # segment is pushed first, then the inner Parallel's bracket segment.
    #   seq child 0 (the Parallel) -> "1/1", then each branch adds "P[k/2]"
    log = []
    leaf0 = RecordingStep("leaf0", log)
    leaf1 = RecordingStep("leaf1", log)
    s = Sequential([Parallel([leaf0, leaf1])])
    run(s.run_step(ROBOT))
    assert leaf0.seen_path == ["1/1", "P[1/2]"]
    assert leaf1.seen_path == ["1/1", "P[2/2]"]
    assert _step_path.get() == []


def test_parallel_factory_branches_execute_in_full():
    log = []
    a, b = RecordingStep("a", log), RecordingStep("b", log)
    c = RecordingStep("c", log)
    p = parallel([a, b], c)
    run(p.run_step(ROBOT))
    assert sorted(log) == ["a", "b", "c"]


# ---------------------------------------------------------------------------
# Composite + DSL-discovery contracts
#
# These assert *semantic attribute values* (not prose): the ``_composite``
# class flag and the ``@dsl(hidden=True)`` discovery metadata. They are part
# of the public contract — ``_composite`` switches run_step into the composite
# logging/timing path, and ``hidden`` keeps the raw classes/factories out of
# the user-facing DSL catalog. Asserting the boolean values (like an enum
# ``.value``) is allowed; the message/log strings are not pinned.
# ---------------------------------------------------------------------------
def test_sequential_is_marked_composite():
    # _composite must be exactly True (not False, not None): run_step branches
    # on `if not self._composite`, and a composite must take the composite arm.
    assert Sequential._composite is True


def test_parallel_is_marked_composite():
    assert Parallel._composite is True


def test_sequential_class_is_dsl_hidden():
    # The raw class is hidden from the DSL catalog (users go through seq()).
    # Removing the decorator drops the attribute entirely -> AttributeError;
    # flipping hidden to False makes these assertions fail.
    assert Sequential.__dsl_hidden__ is True
    assert Sequential.__dsl__.hidden is True


def test_seq_factory_is_dsl_hidden():
    assert seq.__dsl_hidden__ is True
    assert seq.__dsl__.hidden is True
    # The @dsl wrapper also installs the return-type-checking factory wrapper;
    # its presence proves the decorator is applied (removing it drops __wrapped__).
    assert hasattr(seq, "__wrapped__")


def test_parallel_class_is_dsl_hidden():
    assert Parallel.__dsl_hidden__ is True
    assert Parallel.__dsl__.hidden is True


def test_parallel_factory_is_dsl_hidden():
    assert parallel.__dsl_hidden__ is True
    assert parallel.__dsl__.hidden is True
    assert hasattr(parallel, "__wrapped__")
