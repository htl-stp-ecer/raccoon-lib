"""Unit tests for the Phase-3 :class:`AbsolutePathExecutor` dispatcher.

These tests pin down the executor's contract without spinning up a real
robot: a fake ``robot.localization`` returns a scripted pose, and the
relative DSL entry points (``drive_forward`` / ``turn_left`` /
``turn_right``) are monkey-patched to record the calls the dispatcher
made instead of actually running motions. That isolates the
node-to-step dispatch logic from the relative motion stack.
"""

from __future__ import annotations

import asyncio
import math
from dataclasses import dataclass
from typing import Any

import pytest

from raccoon.step import Step
from raccoon.step.motion.path import (
    AbsolutePathExecutor,
    Action,
    Goto,
    Resync,
    TurnTo,
    compile_plan,
)
from raccoon.step.motion.path import abs_executor as _abs_exec_mod

# ---------------------------------------------------------------------------
# Test doubles
# ---------------------------------------------------------------------------


@dataclass
class _FakePose:
    position: tuple[float, float]
    heading: float


class _FakeLocalization:
    def __init__(self, pose: _FakePose) -> None:
        self._pose = pose

    def get_pose(self) -> _FakePose:
        return self._pose


class _FakeRobot:
    def __init__(self, pose: _FakePose | None) -> None:
        if pose is None:
            self.localization = None
        else:
            self.localization = _FakeLocalization(pose)


class _RecordingStub:
    """Replacement for a DSL builder factory.

    Calling the stub captures kwargs into ``calls`` and returns a tiny
    object whose ``run_step`` is an awaitable no-op. The dispatcher
    treats the stub like the real ``drive_forward(...) -> Builder``
    pipeline.
    """

    def __init__(self, name: str, sink: list[tuple[str, dict[str, Any]]]) -> None:
        self._name = name
        self._sink = sink

    def __call__(self, **kwargs: Any) -> Any:
        self._sink.append((self._name, kwargs))

        class _Awaitable:
            async def run_step(self, _robot: Any) -> None:
                return None

        return _Awaitable()


@pytest.fixture
def patch_dsl(monkeypatch: pytest.MonkeyPatch):
    """Swap drive_forward / turn_left / turn_right inside their dsl modules.

    The executor does ``from ..drive_dsl import drive_forward`` lazily, so
    patching the *source* modules guarantees the patched stubs show up
    regardless of import order.
    """

    def _install() -> list[tuple[str, dict[str, Any]]]:
        from raccoon.step.motion import drive_dsl, turn_dsl

        sink: list[tuple[str, dict[str, Any]]] = []
        monkeypatch.setattr(drive_dsl, "drive_forward", _RecordingStub("drive_forward", sink))
        monkeypatch.setattr(turn_dsl, "turn_left", _RecordingStub("turn_left", sink))
        monkeypatch.setattr(turn_dsl, "turn_right", _RecordingStub("turn_right", sink))
        return sink

    return _install


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _run(coro):
    return asyncio.get_event_loop().run_until_complete(coro) if False else asyncio.run(coro)


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------


def test_run_requires_localization() -> None:
    """A robot without ``localization`` is a clear configuration error."""
    robot = _FakeRobot(pose=None)
    plan = compile_plan([Goto(x_m=0.5, y_m=0.0)])
    with pytest.raises(RuntimeError, match="localization"):
        _run(AbsolutePathExecutor().run(robot, plan))


def test_goto_x_axis_drives_forward(patch_dsl) -> None:
    """``Goto(1, 0)`` from origin: tiny / no turn, drive 100 cm forward."""
    sink = patch_dsl()
    robot = _FakeRobot(_FakePose(position=(0.0, 0.0), heading=0.0))
    plan = compile_plan([Goto(x_m=1.0, y_m=0.0)])

    _run(AbsolutePathExecutor().run(robot, plan))

    # Heading already faces +x → turn skipped (within ~1° tolerance).
    drives = [c for c in sink if c[0] == "drive_forward"]
    turns = [c for c in sink if c[0] in ("turn_left", "turn_right")]
    assert len(drives) == 1, sink
    assert drives[0][1]["cm"] == pytest.approx(100.0)
    assert turns == [], "no rotation needed when already aimed at the target"


def test_goto_y_axis_turns_left_then_drives(patch_dsl) -> None:
    """``Goto(0, 1)`` from origin: turn_left(90°) + drive_forward(100 cm)."""
    sink = patch_dsl()
    robot = _FakeRobot(_FakePose(position=(0.0, 0.0), heading=0.0))
    plan = compile_plan([Goto(x_m=0.0, y_m=1.0)])

    _run(AbsolutePathExecutor().run(robot, plan))

    assert [c[0] for c in sink] == ["turn_left", "drive_forward"]
    assert sink[0][1]["degrees"] == pytest.approx(90.0)
    assert sink[1][1]["cm"] == pytest.approx(100.0)


def test_turn_to_pi_from_zero_dispatches_180_degrees(patch_dsl) -> None:
    """``TurnTo(π)`` from heading 0 → a single 180° rotation, either side."""
    sink = patch_dsl()
    robot = _FakeRobot(_FakePose(position=(0.0, 0.0), heading=0.0))
    plan = compile_plan([TurnTo(theta_rad=math.pi)])

    _run(AbsolutePathExecutor().run(robot, plan))

    assert len(sink) == 1
    name, kwargs = sink[0]
    assert name in ("turn_left", "turn_right")
    assert abs(kwargs["degrees"]) == pytest.approx(180.0, abs=0.5)


def test_turn_to_handles_heading_wrap(patch_dsl) -> None:
    """Wrap edge cases: TurnTo(π) from π/2 → 90°; TurnTo(-π) from π → 0° (skip)."""
    # Case 1: π/2 → π is a +90° rotation.
    sink = patch_dsl()
    robot = _FakeRobot(_FakePose(position=(0.0, 0.0), heading=math.pi / 2))
    _run(AbsolutePathExecutor().run(robot, compile_plan([TurnTo(theta_rad=math.pi)])))
    assert len(sink) == 1
    assert sink[0][0] == "turn_left"
    assert sink[0][1]["degrees"] == pytest.approx(90.0)

    # Case 2: π → -π is the same heading; delta wraps to 0 → no turn issued.
    sink2 = patch_dsl()
    robot2 = _FakeRobot(_FakePose(position=(0.0, 0.0), heading=math.pi))
    _run(AbsolutePathExecutor().run(robot2, compile_plan([TurnTo(theta_rad=-math.pi)])))
    assert sink2 == []


def test_resync_raises_not_implemented(patch_dsl) -> None:
    """Resync is Phase-5 work — Phase-3 dispatcher must refuse it loudly."""
    patch_dsl()
    robot = _FakeRobot(_FakePose(position=(0.0, 0.0), heading=0.0))
    plan = compile_plan([Resync(method="wall_align")])
    with pytest.raises(NotImplementedError, match="Phase 5"):
        _run(AbsolutePathExecutor().run(robot, plan))


def test_blocking_action_is_awaited(patch_dsl) -> None:
    """``Action(blocking=True)`` runs the wrapped step's ``run_step``."""
    patch_dsl()
    calls: list[str] = []

    class _RecordingStep(Step):
        def _generate_signature(self) -> str:
            return "RecordingStep()"

        async def run_step(self, robot: Any) -> None:  # type: ignore[override]
            calls.append("ran")

    robot = _FakeRobot(_FakePose(position=(0.0, 0.0), heading=0.0))
    plan = compile_plan([Action(step=_RecordingStep(), blocking=True)])
    _run(AbsolutePathExecutor().run(robot, plan))

    assert calls == ["ran"]


def test_non_blocking_action_raises_not_implemented(patch_dsl) -> None:
    """Non-blocking actions are Phase-4 — the stub raises clearly today."""
    patch_dsl()

    class _NoopStep(Step):
        def _generate_signature(self) -> str:
            return "NoopStep()"

        async def run_step(self, robot: Any) -> None:  # type: ignore[override]
            return None

    robot = _FakeRobot(_FakePose(position=(0.0, 0.0), heading=0.0))
    plan = compile_plan([Action(step=_NoopStep(), blocking=False)])
    with pytest.raises(NotImplementedError, match="Phase-4"):
        _run(AbsolutePathExecutor().run(robot, plan))


def test_wrap_angle_convention() -> None:
    """``_wrap_angle`` keeps π positive — anchors the turn-direction tie-break."""
    assert _abs_exec_mod._wrap_angle(0.0) == pytest.approx(0.0)
    assert _abs_exec_mod._wrap_angle(math.pi) == pytest.approx(math.pi)
    assert _abs_exec_mod._wrap_angle(-math.pi) == pytest.approx(math.pi)
    assert _abs_exec_mod._wrap_angle(3 * math.pi / 2) == pytest.approx(-math.pi / 2)
