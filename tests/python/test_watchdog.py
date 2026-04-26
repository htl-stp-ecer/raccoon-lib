"""Tests for the watchdog manager, DSL steps, and mission time_budget."""

from __future__ import annotations

import asyncio
import contextlib
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


def _make_robot():
    robot = MagicMock()
    # Strip any auto-generated attributes the managers would otherwise collide with
    for attr in ("_resource_manager", "_background_manager", "_watchdog_manager"):
        if hasattr(robot, attr):
            with contextlib.suppress(KeyError):
                del robot.__dict__[attr]
    return robot


# ---------------------------------------------------------------------------
# WatchdogManager: low-level behavior
# ---------------------------------------------------------------------------


@requires_libstp
class TestWatchdogManager:
    @pytest.mark.asyncio
    async def test_arm_and_expire_cancels_main_task(self):
        """A watchdog that isn't fed fires and cancels the attached main task."""
        from raccoon.step.watchdog_manager import WatchdogManager

        mgr = WatchdogManager()

        async def long_running():
            await asyncio.sleep(5.0)

        main = asyncio.create_task(long_running())
        mgr.attach_main_task(main)
        mgr.arm("test", timeout=0.05)

        with pytest.raises(asyncio.CancelledError):
            await main

        assert main.cancelled()
        assert mgr.expired_name == "test"

    @pytest.mark.asyncio
    async def test_feed_prevents_expiry(self):
        """Feeding a watchdog before its deadline keeps the main task alive."""
        from raccoon.step.watchdog_manager import WatchdogManager

        mgr = WatchdogManager()

        async def short_work():
            await asyncio.sleep(0.2)
            return "done"

        main = asyncio.create_task(short_work())
        mgr.attach_main_task(main)
        mgr.arm("test", timeout=0.1)

        # Feed repeatedly so the watchdog never fires
        for _ in range(5):
            await asyncio.sleep(0.05)
            mgr.feed("test")

        result = await main
        assert result == "done"
        assert mgr.expired_name is None
        mgr.disarm("test", missing_ok=True)
        await mgr.cancel_all()

    @pytest.mark.asyncio
    async def test_disarm_stops_expiry(self):
        """A disarmed watchdog never fires."""
        from raccoon.step.watchdog_manager import WatchdogManager

        mgr = WatchdogManager()

        async def short_work():
            await asyncio.sleep(0.2)
            return "done"

        main = asyncio.create_task(short_work())
        mgr.attach_main_task(main)
        mgr.arm("test", timeout=0.05)
        mgr.disarm("test")

        result = await main
        assert result == "done"
        assert mgr.expired_name is None
        await mgr.cancel_all()

    @pytest.mark.asyncio
    async def test_multiple_watchdogs_coexist(self):
        """Named watchdogs are independent."""
        from raccoon.step.watchdog_manager import WatchdogManager

        mgr = WatchdogManager()

        async def long_running():
            await asyncio.sleep(5.0)

        main = asyncio.create_task(long_running())
        mgr.attach_main_task(main)
        mgr.arm("a", timeout=1.0)
        mgr.arm("b", timeout=0.05)

        with pytest.raises(asyncio.CancelledError):
            await main

        # Watchdog "b" should have fired first
        assert mgr.expired_name == "b"
        await mgr.cancel_all()

    @pytest.mark.asyncio
    async def test_arm_rejects_non_positive_timeout(self):
        from raccoon.step.watchdog_manager import WatchdogManager

        mgr = WatchdogManager()
        with pytest.raises(ValueError):
            mgr.arm("bad", timeout=0)
        with pytest.raises(ValueError):
            mgr.arm("bad", timeout=-1.0)

    @pytest.mark.asyncio
    async def test_feed_unarmed_does_not_raise(self):
        from raccoon.step.watchdog_manager import WatchdogManager

        mgr = WatchdogManager()
        mgr.feed("nothing")  # should log but not raise

    @pytest.mark.asyncio
    async def test_disarm_unarmed_with_missing_ok(self):
        from raccoon.step.watchdog_manager import WatchdogManager

        mgr = WatchdogManager()
        mgr.disarm("nothing", missing_ok=True)  # silent
        mgr.disarm("nothing")  # warn but don't raise

    @pytest.mark.asyncio
    async def test_active_names_filters_by_source(self):
        from raccoon.step.watchdog_manager import WatchdogManager

        mgr = WatchdogManager()
        mgr.arm("user1", timeout=10.0, source="user")
        mgr.arm("user2", timeout=10.0, source="user")
        mgr.arm("mission:X", timeout=10.0, source="mission")

        assert set(mgr.active_names("user")) == {"user1", "user2"}
        assert set(mgr.active_names("mission")) == {"mission:X"}
        assert set(mgr.active_names()) == {"user1", "user2", "mission:X"}
        await mgr.cancel_all()


# ---------------------------------------------------------------------------
# DSL steps
# ---------------------------------------------------------------------------


@requires_libstp
class TestWatchdogSteps:
    @pytest.mark.asyncio
    async def test_start_stop_watchdog_steps(self):
        """start_watchdog + stop_watchdog arm/disarm via the manager."""
        from raccoon.step import start_watchdog, stop_watchdog
        from raccoon.step.watchdog_manager import get_watchdog_manager

        robot = _make_robot()
        await start_watchdog("dsl_test", timeout=5.0).run_step(robot)
        mgr = get_watchdog_manager(robot)
        assert "dsl_test" in mgr.active_names("user")

        await stop_watchdog("dsl_test").run_step(robot)
        assert "dsl_test" not in mgr.active_names("user")
        await mgr.cancel_all()

    @pytest.mark.asyncio
    async def test_feed_watchdog_step(self):
        """feed_watchdog step pushes the deadline out."""
        from raccoon.step import feed_watchdog, start_watchdog, stop_watchdog
        from raccoon.step.watchdog_manager import get_watchdog_manager

        robot = _make_robot()
        await start_watchdog("dsl_feed", timeout=0.2).run_step(robot)
        mgr = get_watchdog_manager(robot)

        async def long_running():
            await asyncio.sleep(2.0)

        main = asyncio.create_task(long_running())
        mgr.attach_main_task(main)

        for _ in range(3):
            await asyncio.sleep(0.1)
            await feed_watchdog("dsl_feed").run_step(robot)

        # Stop before any expiry
        await stop_watchdog("dsl_feed").run_step(robot)
        assert mgr.expired_name is None

        main.cancel()
        with contextlib.suppress(asyncio.CancelledError):
            await main
        await mgr.cancel_all()

    def test_start_watchdog_rejects_invalid_args(self):
        from raccoon.step import start_watchdog

        with pytest.raises(ValueError):
            start_watchdog("", timeout=1.0)
        with pytest.raises(ValueError):
            start_watchdog("ok", timeout=0)
        with pytest.raises(ValueError):
            start_watchdog("ok", timeout=-1)


# ---------------------------------------------------------------------------
# Mission time_budget integration
# ---------------------------------------------------------------------------


@requires_libstp
class TestMissionTimeBudget:
    def test_mission_base_has_default_none_budget(self):
        from raccoon.mission.api import Mission

        class EmptyMission(Mission):
            def sequence(self):
                from raccoon.step.base import Step

                class Noop(Step):
                    def _generate_signature(self):
                        return "Noop()"

                    async def _execute_step(self, robot):
                        pass

                return Noop()

        assert EmptyMission.time_budget is None

    def test_mission_subclass_can_set_budget(self):
        from raccoon.mission.api import Mission

        class BudgetedMission(Mission):
            time_budget = 12.5

            def sequence(self):
                from raccoon.step.base import Step

                class Noop(Step):
                    def _generate_signature(self):
                        return "Noop()"

                    async def _execute_step(self, robot):
                        pass

                return Noop()

        assert BudgetedMission.time_budget == 12.5
