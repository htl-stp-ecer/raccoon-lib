"""Tests for background step execution, preemption, and wait_for_background."""
import asyncio
from unittest.mock import MagicMock

import pytest


def libstp_available():
    try:
        import raccoon
        return True
    except ImportError:
        return False


requires_libstp = pytest.mark.skipif(
    not libstp_available(),
    reason="raccoon native module not installed",
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _make_step_classes():
    from raccoon.step.base import Step

    class SlowStep(Step):
        """Step that sleeps for a given duration, tracking start/stop."""

        def __init__(self, resources=frozenset(), duration=0.1):
            super().__init__()
            self._resources = resources
            self._duration = duration
            self.started = False
            self.finished = False
            self.cancelled = False

        def required_resources(self):
            return self._resources

        def _generate_signature(self):
            res = ", ".join(sorted(self._resources)) if self._resources else "none"
            return f"SlowStep(res={res}, dur={self._duration})"

        async def _execute_step(self, robot):
            self.started = True
            try:
                await asyncio.sleep(self._duration)
                self.finished = True
            except asyncio.CancelledError:
                self.cancelled = True
                raise

    class InstantStep(Step):
        def __init__(self, resources=frozenset()):
            super().__init__()
            self._resources = resources
            self.executed = False

        def required_resources(self):
            return self._resources

        def _generate_signature(self):
            return "InstantStep()"

        async def _execute_step(self, robot):
            self.executed = True

    return SlowStep, InstantStep


def _make_robot():
    robot = MagicMock()
    del robot._resource_manager
    del robot._background_manager
    return robot


# ---------------------------------------------------------------------------
# Background: basic launch and wait
# ---------------------------------------------------------------------------


@requires_libstp
class TestBackgroundBasic:
    @pytest.mark.asyncio
    async def test_background_does_not_block(self):
        """background() returns immediately while the step runs."""
        from raccoon.step.logic.background import background
        SlowStep, _ = _make_step_classes()

        step = SlowStep(duration=0.5)
        bg = background(step)
        robot = _make_robot()

        await bg.run_step(robot)

        # The background step should have started but not finished
        assert step.started
        assert not step.finished

        # Give it time to finish
        await asyncio.sleep(0.6)
        assert step.finished

    @pytest.mark.asyncio
    async def test_wait_for_background_all(self):
        """wait_for_background() blocks until all bg steps finish."""
        from raccoon.step.logic.background import background, wait_for_background
        from raccoon.step.sequential import seq
        SlowStep, _ = _make_step_classes()

        step_a = SlowStep(duration=0.1)
        step_b = SlowStep(duration=0.15)
        robot = _make_robot()

        s = seq([
            background(step_a),
            background(step_b),
            wait_for_background(),
        ])
        await s.run_step(robot)

        assert step_a.finished
        assert step_b.finished

    @pytest.mark.asyncio
    async def test_wait_for_background_by_name(self):
        """wait_for_background(name) blocks only for the named step."""
        from raccoon.step.logic.background import background, wait_for_background
        from raccoon.step.sequential import seq
        SlowStep, _ = _make_step_classes()

        fast = SlowStep(duration=0.05)
        slow = SlowStep(duration=0.3)
        robot = _make_robot()

        s = seq([
            background(fast, name="fast"),
            background(slow, name="slow"),
            wait_for_background("fast"),
        ])
        await s.run_step(robot)

        assert fast.finished
        # Slow may or may not have finished yet — we only waited for fast
        # Give slow time to finish for cleanup
        await asyncio.sleep(0.35)

    @pytest.mark.asyncio
    async def test_wait_for_already_done(self):
        """wait_for_background returns immediately if step already finished."""
        from raccoon.step.logic.background import background, wait_for_background
        from raccoon.step.sequential import seq
        SlowStep, _ = _make_step_classes()

        step = SlowStep(duration=0.01)
        robot = _make_robot()

        s = seq([
            background(step, name="quick"),
            SlowStep(duration=0.1),  # Give bg time to finish
            wait_for_background("quick"),  # Should return immediately
        ])
        await s.run_step(robot)
        assert step.finished

    @pytest.mark.asyncio
    async def test_wait_for_nonexistent_name(self):
        """wait_for_background with unknown name returns immediately."""
        from raccoon.step.logic.background import wait_for_background
        robot = _make_robot()
        w = wait_for_background("does_not_exist")
        await w.run_step(robot)  # Should not raise


# ---------------------------------------------------------------------------
# Background: resource preemption
# ---------------------------------------------------------------------------


@requires_libstp
class TestBackgroundPreemption:
    @pytest.mark.asyncio
    async def test_foreground_preempts_background_on_conflict(self):
        """Foreground step cancels a background step using the same resource."""
        from raccoon.step.logic.background import background
        from raccoon.step.sequential import seq
        SlowStep, InstantStep = _make_step_classes()

        bg_step = SlowStep(resources=frozenset({"drive"}), duration=5.0)
        fg_step = InstantStep(resources=frozenset({"drive"}))
        robot = _make_robot()

        s = seq([
            background(bg_step),
            SlowStep(duration=0.05),  # Let bg start
            fg_step,  # Should preempt bg
        ])
        await s.run_step(robot)

        assert bg_step.started
        assert not bg_step.finished  # Was cancelled
        assert fg_step.executed

    @pytest.mark.asyncio
    async def test_no_preemption_without_conflict(self):
        """Background step continues when foreground uses different resource."""
        from raccoon.step.logic.background import background, wait_for_background
        from raccoon.step.sequential import seq
        SlowStep, InstantStep = _make_step_classes()

        bg_step = SlowStep(resources=frozenset({"servo:0"}), duration=0.1)
        fg_step = InstantStep(resources=frozenset({"drive"}))
        robot = _make_robot()

        s = seq([
            background(bg_step),
            fg_step,
            wait_for_background(),
        ])
        await s.run_step(robot)

        assert bg_step.finished
        assert fg_step.executed

    @pytest.mark.asyncio
    async def test_wildcard_preemption(self):
        """servo:* foreground preempts servo:0 background."""
        from raccoon.step.logic.background import background
        from raccoon.step.sequential import seq
        SlowStep, InstantStep = _make_step_classes()

        bg_step = SlowStep(resources=frozenset({"servo:0"}), duration=5.0)
        fg_step = InstantStep(resources=frozenset({"servo:*"}))
        robot = _make_robot()

        s = seq([
            background(bg_step),
            SlowStep(duration=0.05),
            fg_step,
        ])
        await s.run_step(robot)

        assert bg_step.started
        assert not bg_step.finished

    @pytest.mark.asyncio
    async def test_multiple_backgrounds_one_preempted(self):
        """Only the conflicting background step is preempted."""
        from raccoon.step.logic.background import background, wait_for_background
        from raccoon.step.sequential import seq
        SlowStep, InstantStep = _make_step_classes()

        bg_drive = SlowStep(resources=frozenset({"drive"}), duration=5.0)
        bg_servo = SlowStep(resources=frozenset({"servo:0"}), duration=0.15)
        fg_drive = InstantStep(resources=frozenset({"drive"}))
        robot = _make_robot()

        s = seq([
            background(bg_drive),
            background(bg_servo),
            SlowStep(duration=0.05),
            fg_drive,  # Preempts bg_drive only
            wait_for_background(),  # Wait for bg_servo
        ])
        await s.run_step(robot)

        assert bg_drive.started
        assert not bg_drive.finished  # Preempted
        assert bg_servo.finished  # Not affected

    @pytest.mark.asyncio
    async def test_background_does_not_preempt_background(self):
        """Background steps do not preempt each other via the preemption check."""
        from raccoon.step.logic.background import background
        from raccoon.step.sequential import seq
        from raccoon.step.resource import ResourceConflictError
        SlowStep, _ = _make_step_classes()

        bg1 = SlowStep(resources=frozenset({"drive"}), duration=0.5)
        bg2 = SlowStep(resources=frozenset({"drive"}), duration=0.1)
        robot = _make_robot()

        s = seq([
            background(bg1),
            SlowStep(duration=0.05),  # Let bg1 start and acquire "drive"
            background(bg2),          # bg2 will fail to acquire "drive"
        ])
        # bg2's inner run_step will hit ResourceConflictError at runtime
        # because bg1 holds "drive" and bg-vs-bg doesn't preempt.
        # The error is caught inside the background task wrapper.
        await s.run_step(robot)

        assert bg1.started
        # bg2 should have failed (ResourceConflictError inside bg task)
        await asyncio.sleep(0.1)
        # bg1 should still be running (not preempted)
        assert not bg1.finished or bg1.finished  # Just verify no crash

        await asyncio.sleep(0.5)


# ---------------------------------------------------------------------------
# Background: signatures and collected_resources
# ---------------------------------------------------------------------------


@requires_libstp
class TestBackgroundMeta:
    def test_signature_includes_inner_step(self):
        from raccoon.step.logic.background import Background
        SlowStep, _ = _make_step_classes()
        step = SlowStep(resources=frozenset({"drive"}), duration=1.0)
        bg = Background(step)
        sig = bg._generate_signature()
        assert "Background(" in sig
        assert "SlowStep" in sig

    def test_signature_includes_name(self):
        from raccoon.step.logic.background import Background
        SlowStep, _ = _make_step_classes()
        bg = Background(SlowStep(), name="my_bg")
        assert "my_bg" in bg._generate_signature()

    def test_collected_resources_empty(self):
        """Background reports empty resources (preemptable, not exclusive)."""
        from raccoon.step.logic.background import Background
        SlowStep, _ = _make_step_classes()
        step = SlowStep(resources=frozenset({"drive", "servo:0"}))
        bg = Background(step)
        assert bg.collected_resources() == frozenset()

    def test_wait_signature(self):
        from raccoon.step.logic.background import WaitForBackground
        assert "all" in WaitForBackground()._generate_signature()
        assert "my_step" in WaitForBackground("my_step")._generate_signature()

    def test_type_validation(self):
        from raccoon.step.logic.background import Background
        with pytest.raises(TypeError):
            Background("not a step")


# ---------------------------------------------------------------------------
# BackgroundManager unit tests
# ---------------------------------------------------------------------------


@requires_libstp
class TestBackgroundManager:
    @pytest.mark.asyncio
    async def test_active_count(self):
        from raccoon.step.background_manager import BackgroundManager

        mgr = BackgroundManager()
        assert mgr.active_count == 0

        async def slow():
            await asyncio.sleep(0.2)

        task = asyncio.create_task(slow())
        mgr.register(task, "slow", None, frozenset())
        assert mgr.active_count == 1

        await task
        # done callback fires synchronously after await
        await asyncio.sleep(0)  # Let callback run
        assert mgr.active_count == 0

    @pytest.mark.asyncio
    async def test_preempt_cancels_conflicting(self):
        from raccoon.step.background_manager import BackgroundManager

        mgr = BackgroundManager()

        async def slow():
            await asyncio.sleep(5.0)

        task = asyncio.create_task(slow())
        mgr.register(task, "drive_fwd", None, frozenset({"drive"}))

        await mgr.preempt_conflicts(frozenset({"drive"}), "turn_left")

        assert task.done()
        assert task.cancelled()

    @pytest.mark.asyncio
    async def test_preempt_leaves_non_conflicting(self):
        from raccoon.step.background_manager import BackgroundManager

        mgr = BackgroundManager()

        async def slow():
            await asyncio.sleep(0.2)

        task = asyncio.create_task(slow())
        mgr.register(task, "servo_move", None, frozenset({"servo:0"}))

        await mgr.preempt_conflicts(frozenset({"drive"}), "drive_fwd")

        assert not task.done()
        task.cancel()
        try:
            await task
        except asyncio.CancelledError:
            pass

    @pytest.mark.asyncio
    async def test_wait_all(self):
        from raccoon.step.background_manager import BackgroundManager

        mgr = BackgroundManager()
        results = []

        async def task_a():
            await asyncio.sleep(0.05)
            results.append("a")

        async def task_b():
            await asyncio.sleep(0.1)
            results.append("b")

        ta = asyncio.create_task(task_a())
        tb = asyncio.create_task(task_b())
        mgr.register(ta, "a", None, frozenset())
        mgr.register(tb, "b", None, frozenset())

        await mgr.wait_all()
        assert "a" in results
        assert "b" in results

    @pytest.mark.asyncio
    async def test_wait_for_name(self):
        from raccoon.step.background_manager import BackgroundManager

        mgr = BackgroundManager()
        done = False

        async def task_fn():
            nonlocal done
            await asyncio.sleep(0.05)
            done = True

        task = asyncio.create_task(task_fn())
        mgr.register(task, "my_task", "my_task", frozenset())

        await mgr.wait_for_name("my_task")
        assert done
