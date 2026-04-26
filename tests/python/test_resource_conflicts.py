"""Tests for hardware resource conflict detection in parallel step execution."""

from __future__ import annotations

import asyncio
import importlib.util
from unittest.mock import MagicMock

import pytest


def libstp_available() -> bool:
    """Check raccoon availability without importing the module."""
    return importlib.util.find_spec("raccoon") is not None


requires_libstp = pytest.mark.skipif(
    not libstp_available(),
    reason="raccoon native module not installed",
)


# ---------------------------------------------------------------------------
# Helpers: lightweight step stubs that declare resources
# ---------------------------------------------------------------------------


def _make_step_classes():
    """Import Step and build stub classes. Call inside tests only."""
    from raccoon.step.base import Step

    class DriveStep(Step):
        def required_resources(self):
            return frozenset({"drive"})

        async def _execute_step(self, robot):
            pass

    class MotorStep(Step):
        def __init__(self, port):
            super().__init__()
            self.port = port

        def required_resources(self):
            return frozenset({f"motor:{self.port}"})

        async def _execute_step(self, robot):
            pass

    class ServoStep(Step):
        def __init__(self, port):
            super().__init__()
            self.port = port

        def required_resources(self):
            return frozenset({f"servo:{self.port}"})

        async def _execute_step(self, robot):
            pass

    class AllServosStep(Step):
        def required_resources(self):
            return frozenset({"servo:*"})

        async def _execute_step(self, robot):
            pass

    class NoResourceStep(Step):
        async def _execute_step(self, robot):
            pass

    class SlowStep(Step):
        def __init__(self, resources, duration=0.1):
            super().__init__()
            self._resources = resources
            self._duration = duration

        def required_resources(self):
            return self._resources

        async def _execute_step(self, robot):
            await asyncio.sleep(self._duration)

    return DriveStep, MotorStep, ServoStep, AllServosStep, NoResourceStep, SlowStep


def _make_robot():
    robot = MagicMock()
    del robot._resource_manager
    return robot


# ---------------------------------------------------------------------------
# Unit tests: _resources_overlap
# ---------------------------------------------------------------------------


@requires_libstp
class TestResourcesOverlap:
    def test_no_overlap(self):
        from raccoon.step.resource import _resources_overlap

        assert _resources_overlap(frozenset({"drive"}), frozenset({"motor:0"})) == []

    def test_direct_overlap(self):
        from raccoon.step.resource import _resources_overlap

        assert _resources_overlap(frozenset({"drive"}), frozenset({"drive"})) == ["drive"]

    def test_multiple_overlap(self):
        from raccoon.step.resource import _resources_overlap

        result = _resources_overlap(
            frozenset({"drive", "motor:0"}),
            frozenset({"drive", "motor:0"}),
        )
        assert sorted(result) == ["drive", "motor:0"]

    def test_wildcard_conflicts_with_specific(self):
        from raccoon.step.resource import _resources_overlap

        result = _resources_overlap(frozenset({"servo:*"}), frozenset({"servo:3"}))
        assert "servo:3" in result

    def test_specific_conflicts_with_wildcard(self):
        from raccoon.step.resource import _resources_overlap

        result = _resources_overlap(frozenset({"servo:0"}), frozenset({"servo:*"}))
        assert "servo:0" in result

    def test_wildcard_vs_wildcard(self):
        from raccoon.step.resource import _resources_overlap

        result = _resources_overlap(frozenset({"servo:*"}), frozenset({"servo:*"}))
        assert "servo:*" in result

    def test_empty_sets(self):
        from raccoon.step.resource import _resources_overlap

        assert _resources_overlap(frozenset(), frozenset()) == []
        assert _resources_overlap(frozenset({"drive"}), frozenset()) == []


# ---------------------------------------------------------------------------
# Unit tests: validate_no_overlap
# ---------------------------------------------------------------------------


@requires_libstp
class TestValidateNoOverlap:
    def test_no_conflict(self):
        from raccoon.step.resource import validate_no_overlap

        DriveStep, _, ServoStep, *_ = _make_step_classes()
        validate_no_overlap([DriveStep(), ServoStep(0)])

    def test_conflict_raises(self):
        from raccoon.step.resource import ResourceConflictError, validate_no_overlap

        DriveStep, *_ = _make_step_classes()
        with pytest.raises(ResourceConflictError, match="drive"):
            validate_no_overlap([DriveStep(), DriveStep()])

    def test_conflict_message_includes_context(self):
        from raccoon.step.resource import ResourceConflictError, validate_no_overlap

        DriveStep, *_ = _make_step_classes()
        with pytest.raises(ResourceConflictError) as exc_info:
            validate_no_overlap([DriveStep(), DriveStep()], context="Parallel")
        assert "Parallel" in str(exc_info.value)


# ---------------------------------------------------------------------------
# Integration tests: Parallel construction validation
# ---------------------------------------------------------------------------


@requires_libstp
class TestParallelValidation:
    def test_independent_resources_ok(self):
        from raccoon.step.parallel import parallel

        DriveStep, MotorStep, ServoStep, *_ = _make_step_classes()
        parallel(DriveStep(), ServoStep(0), MotorStep(3))

    def test_two_drive_steps_conflict(self):
        from raccoon.step.parallel import parallel
        from raccoon.step.resource import ResourceConflictError

        DriveStep, *_ = _make_step_classes()
        with pytest.raises(ResourceConflictError, match="drive"):
            parallel(DriveStep(), DriveStep())

    def test_same_motor_port_conflict(self):
        from raccoon.step.parallel import parallel
        from raccoon.step.resource import ResourceConflictError

        _, MotorStep, *_ = _make_step_classes()
        with pytest.raises(ResourceConflictError, match="motor:0"):
            parallel(MotorStep(0), MotorStep(0))

    def test_different_motor_ports_ok(self):
        from raccoon.step.parallel import parallel

        _, MotorStep, *_ = _make_step_classes()
        parallel(MotorStep(0), MotorStep(1))

    def test_same_servo_port_conflict(self):
        from raccoon.step.parallel import parallel
        from raccoon.step.resource import ResourceConflictError

        _, _, ServoStep, *_ = _make_step_classes()
        with pytest.raises(ResourceConflictError, match="servo:2"):
            parallel(ServoStep(2), ServoStep(2))

    def test_different_servo_ports_ok(self):
        from raccoon.step.parallel import parallel

        _, _, ServoStep, *_ = _make_step_classes()
        parallel(ServoStep(0), ServoStep(1))

    def test_all_servos_conflicts_with_specific(self):
        from raccoon.step.parallel import parallel
        from raccoon.step.resource import ResourceConflictError

        _, _, ServoStep, AllServosStep, *_ = _make_step_classes()
        with pytest.raises(ResourceConflictError, match="servo"):
            parallel(AllServosStep(), ServoStep(0))

    def test_sequential_branch_unions_resources(self):
        from raccoon.step.parallel import parallel
        from raccoon.step.resource import ResourceConflictError
        from raccoon.step.sequential import seq

        DriveStep, _, ServoStep, *_ = _make_step_classes()
        branch_a = seq([DriveStep(), ServoStep(0)])
        branch_b = seq([DriveStep()])
        with pytest.raises(ResourceConflictError, match="drive"):
            parallel(branch_a, branch_b)

    def test_sequential_branch_disjoint_ok(self):
        from raccoon.step.parallel import parallel
        from raccoon.step.sequential import seq

        DriveStep, MotorStep, ServoStep, *_ = _make_step_classes()
        branch_a = seq([DriveStep(), ServoStep(0)])
        branch_b = seq([MotorStep(3), ServoStep(1)])
        parallel(branch_a, branch_b)

    def test_no_resource_steps_always_ok(self):
        from raccoon.step.parallel import parallel

        DriveStep, _, _, _, NoResourceStep, _ = _make_step_classes()
        parallel(NoResourceStep(), NoResourceStep(), DriveStep())

    def test_nested_parallel_resources_propagate(self):
        from raccoon.step.parallel import parallel
        from raccoon.step.resource import ResourceConflictError

        _, MotorStep, ServoStep, *_ = _make_step_classes()
        inner = parallel(ServoStep(0), MotorStep(1))
        with pytest.raises(ResourceConflictError, match="servo:0"):
            parallel(inner, ServoStep(0))


# ---------------------------------------------------------------------------
# Integration tests: DoWhileActive validation
# ---------------------------------------------------------------------------


@requires_libstp
class TestDoWhileActiveValidation:
    def test_disjoint_ok(self):
        from raccoon.step.logic.do_while import DoWhileActive

        DriveStep, _, ServoStep, *_ = _make_step_classes()
        DoWhileActive(DriveStep(), ServoStep(0))

    def test_overlap_raises(self):
        from raccoon.step.logic.do_while import DoWhileActive
        from raccoon.step.resource import ResourceConflictError

        DriveStep, *_ = _make_step_classes()
        with pytest.raises(ResourceConflictError, match="drive"):
            DoWhileActive(DriveStep(), DriveStep())


# ---------------------------------------------------------------------------
# Integration tests: composite step resource propagation
# ---------------------------------------------------------------------------


@requires_libstp
class TestCompositeResourcePropagation:
    def test_sequential_collects(self):
        from raccoon.step.sequential import seq

        DriveStep, MotorStep, ServoStep, *_ = _make_step_classes()
        s = seq([DriveStep(), MotorStep(0), ServoStep(1)])
        assert s.collected_resources() == frozenset({"drive", "motor:0", "servo:1"})
        # Composite's own resources are empty
        assert s.required_resources() == frozenset()

    def test_parallel_collects(self):
        from raccoon.step.parallel import parallel

        _, MotorStep, ServoStep, *_ = _make_step_classes()
        p = parallel(ServoStep(0), MotorStep(1))
        assert p.collected_resources() == frozenset({"servo:0", "motor:1"})
        assert p.required_resources() == frozenset()

    def test_loop_delegates(self):
        from raccoon.step.logic.loop import LoopFor, LoopForever

        DriveStep, MotorStep, *_ = _make_step_classes()
        lf = LoopFor(DriveStep(), iterations=3)
        assert lf.collected_resources() == frozenset({"drive"})
        linf = LoopForever(MotorStep(2))
        assert linf.collected_resources() == frozenset({"motor:2"})

    def test_timeout_delegates(self):
        from raccoon.step.timeout import Timeout

        _, _, ServoStep, *_ = _make_step_classes()
        t = Timeout(ServoStep(0), seconds=5.0)
        assert t.collected_resources() == frozenset({"servo:0"})

    def test_no_resource_step(self):
        *_, NoResourceStep, _ = _make_step_classes()
        assert NoResourceStep().required_resources() == frozenset()
        assert NoResourceStep().collected_resources() == frozenset()


# ---------------------------------------------------------------------------
# Runtime ResourceManager tests
# ---------------------------------------------------------------------------


@requires_libstp
class TestResourceManager:
    def test_acquire_release(self):
        from raccoon.step.resource import ResourceManager

        mgr = ResourceManager()
        mgr.acquire(frozenset({"drive"}), "DriveForward")
        mgr.release(frozenset({"drive"}))
        mgr.acquire(frozenset({"drive"}), "TurnLeft")

    def test_double_acquire_raises(self):
        from raccoon.step.resource import ResourceConflictError, ResourceManager

        mgr = ResourceManager()
        mgr.acquire(frozenset({"drive"}), "DriveForward")
        with pytest.raises(ResourceConflictError, match="drive"):
            mgr.acquire(frozenset({"drive"}), "TurnLeft")

    def test_independent_resources_ok(self):
        from raccoon.step.resource import ResourceManager

        mgr = ResourceManager()
        mgr.acquire(frozenset({"drive"}), "DriveForward")
        mgr.acquire(frozenset({"servo:0"}), "SetServo")

    def test_wildcard_conflicts(self):
        from raccoon.step.resource import ResourceConflictError, ResourceManager

        mgr = ResourceManager()
        mgr.acquire(frozenset({"servo:0"}), "SetServo")
        with pytest.raises(ResourceConflictError):
            mgr.acquire(frozenset({"servo:*"}), "FullyDisableServos")

    def test_wildcard_held_blocks_specific(self):
        from raccoon.step.resource import ResourceConflictError, ResourceManager

        mgr = ResourceManager()
        mgr.acquire(frozenset({"servo:*"}), "FullyDisableServos")
        with pytest.raises(ResourceConflictError):
            mgr.acquire(frozenset({"servo:0"}), "SetServo")


# ---------------------------------------------------------------------------
# Runtime integration: steps acquire/release through run_step
# ---------------------------------------------------------------------------


@requires_libstp
class TestRuntimeResourceGuard:
    @pytest.mark.asyncio
    async def test_sequential_reuse_ok(self):
        """Sequential steps release resources before the next step acquires."""
        from raccoon.step.sequential import seq

        DriveStep, *_ = _make_step_classes()
        robot = _make_robot()
        s = seq([DriveStep(), DriveStep()])
        await s.run_step(robot)

    @pytest.mark.asyncio
    async def test_parallel_conflict_at_runtime(self):
        """Runtime guard catches conflicts from dynamically created steps."""
        from raccoon.step.base import Step
        from raccoon.step.parallel import Parallel
        from raccoon.step.resource import ResourceConflictError

        *_, SlowStep = _make_step_classes()
        robot = _make_robot()

        step_a = SlowStep(frozenset({"drive"}), duration=0.2)
        step_b = SlowStep(frozenset({"drive"}), duration=0.1)

        # Bypass __init__ validation (simulates Defer-constructed steps)
        p = Parallel.__new__(Parallel)
        Step.__init__(p)
        p.steps = [step_a, step_b]
        p._last_completed_step = None

        with pytest.raises(ResourceConflictError, match="drive"):
            await p.run_step(robot)

    @pytest.mark.asyncio
    async def test_parallel_independent_runtime_ok(self):
        """Independent resources work fine at runtime."""
        from raccoon.step.parallel import parallel

        *_, SlowStep = _make_step_classes()
        robot = _make_robot()
        p = parallel(
            SlowStep(frozenset({"servo:0"}), duration=0.05),
            SlowStep(frozenset({"motor:1"}), duration=0.05),
        )
        await p.run_step(robot)
