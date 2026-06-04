from __future__ import annotations

import importlib.util
import sys
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest


def libstp_available() -> bool:
    return importlib.util.find_spec("raccoon.step.servo.steps") is not None


requires_libstp = pytest.mark.skipif(
    not libstp_available(),
    reason="raccoon native module not installed",
)


def _make_servo(*, position: float = 90.0):
    return SimpleNamespace(
        port=3,
        get_position=MagicMock(return_value=position),
        set_position=MagicMock(),
        set_smooth_position=MagicMock(),
        enable=MagicMock(),
        disable=MagicMock(),
    )


def _load_workspace_servo_steps():
    import raccoon.step.servo as servo_pkg

    repo_root = Path(__file__).resolve().parents[2]
    module_path = repo_root / "modules/libstp-servo/python/raccoon/step/servo/steps.py"

    sys.modules.pop("raccoon.step.servo.steps", None)
    spec = importlib.util.spec_from_file_location("raccoon.step.servo.steps", module_path)
    if spec is None or spec.loader is None:
        msg = f"could not load workspace servo steps from {module_path}"
        raise RuntimeError(msg)

    module = importlib.util.module_from_spec(spec)
    sys.modules["raccoon.step.servo.steps"] = module
    spec.loader.exec_module(module)
    servo_pkg.steps = module
    return module


@requires_libstp
class TestServoStepsOnlyCallTheirIntendedServoApis:
    @pytest.mark.asyncio
    async def test_set_servo_position_only_sets_position(self):
        SetServoPosition = _load_workspace_servo_steps().SetServoPosition

        servo = _make_servo(position=15.0)
        step = SetServoPosition(servo, 200.0, duration=0.0)

        await step._execute_step(robot=None)

        servo.set_position.assert_called_once_with(200.0)
        servo.set_smooth_position.assert_not_called()
        servo.enable.assert_not_called()
        servo.disable.assert_not_called()

    @pytest.mark.asyncio
    async def test_slow_servo_only_uses_smooth_servo_for_builtin_easing(self):
        steps = _load_workspace_servo_steps()
        Easing = steps.Easing
        SlowServo = steps.SlowServo

        servo = _make_servo(position=10.0)
        step = SlowServo(servo, 160.0, speed=5000.0, easing=Easing.EASE_IN_OUT)

        await step._execute_step(robot=None)

        servo.set_smooth_position.assert_called_once_with(160.0, 5000.0, 3)
        servo.set_position.assert_not_called()
        servo.enable.assert_not_called()
        servo.disable.assert_not_called()

    @pytest.mark.asyncio
    async def test_shake_servo_zero_duration_only_sets_position(self):
        ShakeServo = _load_workspace_servo_steps().ShakeServo

        servo = _make_servo(position=30.0)
        step = ShakeServo(servo, duration=0.0, angle_a=40.0, angle_b=120.0)

        await step._execute_step(robot=None)

        servo.set_position.assert_called_once_with(40.0)
        servo.set_smooth_position.assert_not_called()
        servo.enable.assert_not_called()
        servo.disable.assert_not_called()
