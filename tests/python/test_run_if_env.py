"""Tests for environment-gated step execution (run_if_env and guards)."""

from __future__ import annotations

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


def _make_marker_step():
    from raccoon.step.base import Step

    class MarkerStep(Step):
        def __init__(self):
            super().__init__()
            self.executed = False

        def _generate_signature(self):
            return "MarkerStep()"

        async def _execute_step(self, robot):
            self.executed = True

    return MarkerStep()


def _make_robot():
    robot = MagicMock()
    del robot._resource_manager
    del robot._background_manager
    return robot


@requires_libstp
class TestRunIfEnvGate:
    @pytest.mark.asyncio
    async def test_runs_when_var_matches(self, monkeypatch):
        from raccoon.step.logic.run_if_env import run_if_env

        monkeypatch.setenv("MY_FLAG", "1")
        step = _make_marker_step()
        await run_if_env(step, "MY_FLAG").run_step(_make_robot())
        assert step.executed

    @pytest.mark.asyncio
    async def test_skips_when_var_unset(self, monkeypatch):
        from raccoon.step.logic.run_if_env import run_if_env

        monkeypatch.delenv("MY_FLAG", raising=False)
        step = _make_marker_step()
        await run_if_env(step, "MY_FLAG").run_step(_make_robot())
        assert not step.executed

    @pytest.mark.asyncio
    async def test_skips_when_var_differs(self, monkeypatch):
        from raccoon.step.logic.run_if_env import run_if_env

        monkeypatch.setenv("MY_FLAG", "0")
        step = _make_marker_step()
        await run_if_env(step, "MY_FLAG", equals="1").run_step(_make_robot())
        assert not step.executed

    @pytest.mark.asyncio
    async def test_equals_none_means_unset(self, monkeypatch):
        from raccoon.step.logic.run_if_env import run_if_env

        monkeypatch.delenv("MY_FLAG", raising=False)
        ran = _make_marker_step()
        await run_if_env(ran, "MY_FLAG", equals=None).run_step(_make_robot())
        assert ran.executed  # unset matches equals=None

        monkeypatch.setenv("MY_FLAG", "anything")
        skipped = _make_marker_step()
        await run_if_env(skipped, "MY_FLAG", equals=None).run_step(_make_robot())
        assert not skipped.executed  # set -> gate closed

    @pytest.mark.asyncio
    async def test_negate_inverts_gate(self, monkeypatch):
        from raccoon.step.logic.run_if_env import run_if_env

        monkeypatch.setenv("MY_FLAG", "1")
        skipped = _make_marker_step()
        await run_if_env(skipped, "MY_FLAG", equals="1", negate=True).run_step(_make_robot())
        assert not skipped.executed

        monkeypatch.delenv("MY_FLAG", raising=False)
        ran = _make_marker_step()
        await run_if_env(ran, "MY_FLAG", equals="1", negate=True).run_step(_make_robot())
        assert ran.executed


@requires_libstp
class TestNamedGuards:
    @pytest.mark.asyncio
    async def test_run_unless_no_calibrate(self, monkeypatch):
        from raccoon.step.logic.run_if_env import run_unless_no_calibrate

        monkeypatch.delenv("LIBSTP_NO_CALIBRATE", raising=False)
        ran = _make_marker_step()
        await run_unless_no_calibrate(ran).run_step(_make_robot())
        assert ran.executed

        monkeypatch.setenv("LIBSTP_NO_CALIBRATE", "1")
        skipped = _make_marker_step()
        await run_unless_no_calibrate(skipped).run_step(_make_robot())
        assert not skipped.executed

    @pytest.mark.asyncio
    async def test_run_unless_no_checkpoints(self, monkeypatch):
        from raccoon.step.logic.run_if_env import run_unless_no_checkpoints

        monkeypatch.setenv("LIBSTP_NO_CHECKPOINTS", "1")
        skipped = _make_marker_step()
        await run_unless_no_checkpoints(skipped).run_step(_make_robot())
        assert not skipped.executed

    @pytest.mark.asyncio
    async def test_run_if_debug(self, monkeypatch):
        from raccoon.step.logic.run_if_env import run_if_debug

        monkeypatch.setenv("LIBSTP_DEBUG", "1")
        ran = _make_marker_step()
        await run_if_debug(ran).run_step(_make_robot())
        assert ran.executed

        monkeypatch.delenv("LIBSTP_DEBUG", raising=False)
        skipped = _make_marker_step()
        await run_if_debug(skipped).run_step(_make_robot())
        assert not skipped.executed

    @pytest.mark.asyncio
    async def test_run_if_dev(self, monkeypatch):
        from raccoon.step.logic.run_if_env import run_if_dev

        monkeypatch.setenv("LIBSTP_DEV_MODE", "1")
        ran = _make_marker_step()
        await run_if_dev(ran).run_step(_make_robot())
        assert ran.executed


@requires_libstp
class TestRunIfEnvMeta:
    def test_signature_includes_gate_and_inner(self):
        from raccoon.step.logic.run_if_env import run_if_debug

        sig = run_if_debug(_make_marker_step())._generate_signature()
        assert "LIBSTP_DEBUG" in sig
        assert "MarkerStep" in sig

    def test_collected_resources_delegates(self):
        from raccoon.step.base import Step
        from raccoon.step.logic.run_if_env import run_if_env

        class ResStep(Step):
            def required_resources(self):
                return frozenset({"drive"})

            def _generate_signature(self):
                return "ResStep()"

            async def _execute_step(self, robot):
                pass

        guard = run_if_env(ResStep(), "MY_FLAG")
        assert guard.collected_resources() == frozenset({"drive"})

    def test_type_validation(self):
        from raccoon.step.logic.run_if_env import run_if_env

        with pytest.raises(TypeError):
            run_if_env("not a step", "MY_FLAG")

    def test_empty_var_rejected(self):
        from raccoon.step.logic.run_if_env import run_if_env

        with pytest.raises(ValueError):
            run_if_env(_make_marker_step(), "")
