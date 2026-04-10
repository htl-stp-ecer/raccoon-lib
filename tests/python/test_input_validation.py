"""Tests for construction-time input validation across all step and condition classes."""
import asyncio
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest


def libstp_available():
    try:
        from raccoon.step.base import Step
        return True
    except (ImportError, ModuleNotFoundError):
        return False


requires_libstp = pytest.mark.skipif(
    not libstp_available(),
    reason="raccoon native module not installed",
)


# ---------------------------------------------------------------------------
# Sensor / motor / servo fakes for validation tests
# ---------------------------------------------------------------------------


def _make_ir_sensor():
    """Fake IRSensor with probabilityOfBlack/White."""
    s = SimpleNamespace(
        port=0,
        probabilityOfBlack=lambda: 0.9,
        probabilityOfWhite=lambda: 0.1,
        read=lambda: 500,
    )
    return s


def _make_digital_sensor():
    s = SimpleNamespace(port=0, read=lambda: True)
    return s


def _make_analog_sensor():
    s = SimpleNamespace(port=0, read=lambda: 512)
    return s


def _make_motor():
    m = MagicMock()
    m.port = 0
    m.get_position = MagicMock(return_value=0)
    m.is_done = MagicMock(return_value=True)
    m.set_speed = MagicMock()
    m.set_velocity = MagicMock()
    m.brake = MagicMock()
    m.off = MagicMock()
    m.move_to_position = MagicMock()
    m.move_relative = MagicMock()
    m.get_calibration = MagicMock()
    return m


def _make_servo():
    s = MagicMock()
    s.port = 0
    s.set_position = MagicMock()
    s.get_position = MagicMock(return_value=90.0)
    s.enable = MagicMock()
    s.disable = MagicMock()
    return s


def _make_step():
    """Create a minimal valid Step for composite step tests."""
    from raccoon.step.base import Step

    class DummyStep(Step):
        async def _execute_step(self, robot):
            pass
    return DummyStep()


# ===========================================================================
# StopCondition validation tests
# ===========================================================================


@requires_libstp
class TestOnBlackValidation:
    def test_valid(self):
        from raccoon.step.condition import on_black
        on_black(_make_ir_sensor())

    def test_wrong_sensor_type(self):
        from raccoon.step.condition import on_black
        with pytest.raises(TypeError, match="IRSensor"):
            on_black("not a sensor")

    def test_motor_instead_of_sensor(self):
        from raccoon.step.condition import on_black
        with pytest.raises(TypeError, match="IRSensor"):
            on_black(_make_motor())

    def test_threshold_out_of_range(self):
        from raccoon.step.condition import on_black
        with pytest.raises(ValueError, match="threshold"):
            on_black(_make_ir_sensor(), threshold=1.5)

    def test_threshold_negative(self):
        from raccoon.step.condition import on_black
        with pytest.raises(ValueError, match="threshold"):
            on_black(_make_ir_sensor(), threshold=-0.1)


@requires_libstp
class TestOnWhiteValidation:
    def test_valid(self):
        from raccoon.step.condition import on_white
        on_white(_make_ir_sensor())

    def test_wrong_sensor_type(self):
        from raccoon.step.condition import on_white
        with pytest.raises(TypeError, match="IRSensor"):
            on_white(42)

    def test_threshold_out_of_range(self):
        from raccoon.step.condition import on_white
        with pytest.raises(ValueError, match="threshold"):
            on_white(_make_ir_sensor(), threshold=2.0)


@requires_libstp
class TestAfterSecondsValidation:
    def test_valid(self):
        from raccoon.step.condition import after_seconds
        after_seconds(5.0)
        after_seconds(0)  # zero is valid

    def test_negative(self):
        from raccoon.step.condition import after_seconds
        with pytest.raises(ValueError, match="seconds"):
            after_seconds(-1.0)

    def test_string(self):
        from raccoon.step.condition import after_seconds
        with pytest.raises(TypeError, match="number"):
            after_seconds("five")


@requires_libstp
class TestAfterCmValidation:
    def test_valid(self):
        from raccoon.step.condition import after_cm
        after_cm(10.0)

    def test_zero(self):
        from raccoon.step.condition import after_cm
        with pytest.raises(ValueError, match="cm"):
            after_cm(0)

    def test_negative(self):
        from raccoon.step.condition import after_cm
        with pytest.raises(ValueError, match="cm"):
            after_cm(-5)

    def test_string(self):
        from raccoon.step.condition import after_cm
        with pytest.raises(TypeError, match="number"):
            after_cm("ten")


@requires_libstp
class TestAfterDegreesValidation:
    def test_valid(self):
        from raccoon.step.condition import after_degrees
        after_degrees(90)

    def test_zero(self):
        from raccoon.step.condition import after_degrees
        with pytest.raises(ValueError, match="degrees"):
            after_degrees(0)

    def test_negative(self):
        from raccoon.step.condition import after_degrees
        with pytest.raises(ValueError, match="degrees"):
            after_degrees(-45)

    def test_string(self):
        from raccoon.step.condition import after_degrees
        with pytest.raises(TypeError, match="number"):
            after_degrees("ninety")


@requires_libstp
class TestOnDigitalValidation:
    def test_valid(self):
        from raccoon.step.condition import on_digital
        on_digital(_make_digital_sensor())

    def test_wrong_type(self):
        from raccoon.step.condition import on_digital
        with pytest.raises(TypeError, match="DigitalSensor"):
            on_digital("not a sensor")

    def test_int_instead_of_sensor(self):
        from raccoon.step.condition import on_digital
        with pytest.raises(TypeError, match="DigitalSensor"):
            on_digital(42)


@requires_libstp
class TestOnAnalogValidation:
    def test_above_valid(self):
        from raccoon.step.condition import on_analog_above
        on_analog_above(_make_analog_sensor(), 500)

    def test_above_wrong_sensor(self):
        from raccoon.step.condition import on_analog_above
        with pytest.raises(TypeError, match="AnalogSensor"):
            on_analog_above("sensor", 500)

    def test_above_wrong_threshold(self):
        from raccoon.step.condition import on_analog_above
        with pytest.raises(TypeError, match="number"):
            on_analog_above(_make_analog_sensor(), "high")

    def test_below_valid(self):
        from raccoon.step.condition import on_analog_below
        on_analog_below(_make_analog_sensor(), 300)

    def test_below_wrong_sensor(self):
        from raccoon.step.condition import on_analog_below
        with pytest.raises(TypeError, match="AnalogSensor"):
            on_analog_below(123, 300)


@requires_libstp
class TestStallDetectedValidation:
    def test_valid(self):
        from raccoon.step.condition import stall_detected
        stall_detected(_make_motor())

    def test_wrong_motor(self):
        from raccoon.step.condition import stall_detected
        with pytest.raises(TypeError, match="Motor"):
            stall_detected("not a motor")

    def test_threshold_zero(self):
        from raccoon.step.condition import stall_detected
        with pytest.raises(ValueError, match="threshold_tps"):
            stall_detected(_make_motor(), threshold_tps=0)

    def test_duration_negative(self):
        from raccoon.step.condition import stall_detected
        with pytest.raises(ValueError, match="duration"):
            stall_detected(_make_motor(), duration=-1.0)


@requires_libstp
class TestCustomConditionValidation:
    def test_valid_lambda(self):
        from raccoon.step.condition import custom
        custom(lambda robot: True)

    def test_not_callable(self):
        from raccoon.step.condition import custom
        with pytest.raises(TypeError, match="callable"):
            custom(42)

    def test_string_not_callable(self):
        from raccoon.step.condition import custom
        with pytest.raises(TypeError, match="callable"):
            custom("some string")


@requires_libstp
class TestConditionCompositionValidation:
    def test_or_valid(self):
        from raccoon.step.condition import after_seconds
        after_seconds(1) | after_seconds(2)

    def test_or_wrong_type(self):
        from raccoon.step.condition import after_seconds
        with pytest.raises(TypeError, match="StopCondition"):
            after_seconds(1) | "not a condition"

    def test_and_wrong_type(self):
        from raccoon.step.condition import after_seconds
        with pytest.raises(TypeError, match="StopCondition"):
            after_seconds(1) & 42

    def test_then_wrong_type(self):
        from raccoon.step.condition import after_seconds
        with pytest.raises(TypeError, match="StopCondition"):
            after_seconds(1) > "next"

    def test_chained_valid(self):
        from raccoon.step.condition import after_seconds, after_cm
        # a > b > c
        after_seconds(1) > after_cm(10) > after_seconds(2)
        # a | b & c
        after_seconds(1) | (after_cm(10) & after_seconds(5))


# ===========================================================================
# Drive step validation tests
# ===========================================================================


@requires_libstp
class TestDriveValidation:
    def test_valid_cm(self):
        from raccoon.step.motion.drive import DriveForward
        DriveForward(cm=25)

    def test_valid_until(self):
        from raccoon.step.motion.drive import DriveForward
        from raccoon.step.condition import after_seconds
        DriveForward(until=after_seconds(5))

    def test_neither_cm_nor_until(self):
        from raccoon.step.motion.drive import DriveForward
        with pytest.raises(ValueError, match="requires either"):
            DriveForward()

    def test_cm_negative(self):
        from raccoon.step.motion.drive import DriveForward
        with pytest.raises(ValueError, match="cm"):
            DriveForward(cm=-5)

    def test_cm_zero(self):
        from raccoon.step.motion.drive import DriveForward
        with pytest.raises(ValueError, match="cm"):
            DriveForward(cm=0)

    def test_cm_string(self):
        from raccoon.step.motion.drive import DriveForward
        with pytest.raises(TypeError, match="cm"):
            DriveForward(cm="twenty")

    def test_speed_too_high(self):
        from raccoon.step.motion.drive import DriveForward
        with pytest.raises(ValueError, match="speed"):
            DriveForward(cm=25, speed=1.5)

    def test_speed_zero(self):
        from raccoon.step.motion.drive import DriveForward
        with pytest.raises(ValueError, match="speed"):
            DriveForward(cm=25, speed=0.0)

    def test_speed_negative(self):
        from raccoon.step.motion.drive import DriveForward
        with pytest.raises(ValueError, match="speed"):
            DriveForward(cm=25, speed=-0.5)

    def test_speed_string(self):
        from raccoon.step.motion.drive import DriveForward
        with pytest.raises(TypeError, match="speed"):
            DriveForward(cm=25, speed="fast")

    def test_until_wrong_type(self):
        from raccoon.step.motion.drive import DriveForward
        with pytest.raises(TypeError, match="StopCondition"):
            DriveForward(until="stop when black")

    def test_until_int(self):
        from raccoon.step.motion.drive import DriveForward
        with pytest.raises(TypeError, match="StopCondition"):
            DriveForward(until=42)

    def test_all_drive_variants(self):
        """All drive/strafe variants share the same validation."""
        from raccoon.step.motion.drive import (
            DriveForward, DriveBackward, StrafeLeft, StrafeRight,
        )
        for cls in (DriveForward, DriveBackward, StrafeLeft, StrafeRight):
            with pytest.raises(ValueError, match="speed"):
                cls(cm=25, speed=5.0)
            with pytest.raises(TypeError, match="StopCondition"):
                cls(until="bad")


# ===========================================================================
# Turn step validation tests
# ===========================================================================


@requires_libstp
class TestTurnValidation:
    def test_valid_degrees(self):
        from raccoon.step.motion.turn import TurnLeft
        TurnLeft(degrees=90)

    def test_degrees_negative(self):
        from raccoon.step.motion.turn import TurnLeft
        with pytest.raises(ValueError, match="degrees"):
            TurnLeft(degrees=-45)

    def test_degrees_zero(self):
        from raccoon.step.motion.turn import TurnLeft
        with pytest.raises(ValueError, match="degrees"):
            TurnLeft(degrees=0)

    def test_speed_out_of_range(self):
        from raccoon.step.motion.turn import TurnRight
        with pytest.raises(ValueError, match="speed"):
            TurnRight(degrees=90, speed=2.0)

    def test_until_wrong_type(self):
        from raccoon.step.motion.turn import TurnLeft
        with pytest.raises(TypeError, match="StopCondition"):
            TurnLeft(until="stop")

    def test_all_turn_variants(self):
        from raccoon.step.motion.turn import TurnLeft, TurnRight
        for cls in (TurnLeft, TurnRight):
            with pytest.raises(ValueError, match="speed"):
                cls(degrees=90, speed=0.0)
            with pytest.raises(TypeError, match="StopCondition"):
                cls(until=123)


# ===========================================================================
# Wall align validation tests
# ===========================================================================


@requires_libstp
class TestWallAlignValidation:
    def test_valid(self):
        from raccoon.step.motion.wall_align import WallAlignForward
        WallAlignForward()

    def test_speed_zero(self):
        from raccoon.step.motion.wall_align import WallAlignForward
        with pytest.raises(ValueError, match="speed"):
            WallAlignForward(speed=0)

    def test_speed_negative(self):
        from raccoon.step.motion.wall_align import WallAlignBackward
        with pytest.raises(ValueError, match="speed"):
            WallAlignBackward(speed=-1.0)

    def test_accel_threshold_zero(self):
        from raccoon.step.motion.wall_align import WallAlignForward
        with pytest.raises(ValueError, match="accel_threshold"):
            WallAlignForward(accel_threshold=0)

    def test_settle_duration_negative(self):
        from raccoon.step.motion.wall_align import WallAlignForward
        with pytest.raises(ValueError, match="settle_duration"):
            WallAlignForward(settle_duration=-0.1)

    def test_max_duration_zero(self):
        from raccoon.step.motion.wall_align import WallAlignForward
        with pytest.raises(ValueError, match="max_duration"):
            WallAlignForward(max_duration=0)

    def test_grace_period_negative(self):
        from raccoon.step.motion.wall_align import WallAlignForward
        with pytest.raises(ValueError, match="grace_period"):
            WallAlignForward(grace_period=-1)

    def test_all_directions(self):
        from raccoon.step.motion.wall_align import (
            WallAlignForward, WallAlignBackward,
            WallAlignStrafeLeft, WallAlignStrafeRight,
        )
        for cls in (WallAlignForward, WallAlignBackward,
                    WallAlignStrafeLeft, WallAlignStrafeRight):
            with pytest.raises(ValueError, match="speed"):
                cls(speed=-1)


# ===========================================================================
# Motor step validation tests
# ===========================================================================


@requires_libstp
class TestMotorStepValidation:
    def test_valid(self):
        from raccoon.step.motor.steps import SetMotorPower
        SetMotorPower(_make_motor(), 50)

    def test_not_a_motor(self):
        from raccoon.step.motor.steps import SetMotorPower
        with pytest.raises(TypeError, match="Motor"):
            SetMotorPower("not a motor", 50)

    def test_int_instead_of_motor(self):
        from raccoon.step.motor.steps import SetMotorPower
        with pytest.raises(TypeError, match="Motor"):
            SetMotorPower(42, 50)

    def test_power_out_of_range(self):
        from raccoon.step.motor.steps import SetMotorPower
        with pytest.raises(ValueError, match="percent"):
            SetMotorPower(_make_motor(), 150)

    def test_velocity_motor_check(self):
        from raccoon.step.motor.steps import SetMotorVelocity
        with pytest.raises(TypeError, match="Motor"):
            SetMotorVelocity("not a motor", 800)

    def test_dps_motor_check(self):
        from raccoon.step.motor.steps import SetMotorDps
        with pytest.raises(TypeError, match="Motor"):
            SetMotorDps("not a motor", 180.0)

    def test_move_to_motor_check(self):
        from raccoon.step.motor.steps import MoveMotorTo
        with pytest.raises(TypeError, match="Motor"):
            MoveMotorTo("bad", 500)

    def test_move_to_velocity_zero(self):
        from raccoon.step.motor.steps import MoveMotorTo
        with pytest.raises(ValueError, match="velocity"):
            MoveMotorTo(_make_motor(), 500, velocity=0)

    def test_move_to_timeout_negative(self):
        from raccoon.step.motor.steps import MoveMotorTo
        with pytest.raises(ValueError, match="timeout"):
            MoveMotorTo(_make_motor(), 500, timeout=-1.0)

    def test_move_relative_motor_check(self):
        from raccoon.step.motor.steps import MoveMotorRelative
        with pytest.raises(TypeError, match="Motor"):
            MoveMotorRelative("bad", 200)

    def test_move_relative_velocity_zero(self):
        from raccoon.step.motor.steps import MoveMotorRelative
        with pytest.raises(ValueError, match="velocity"):
            MoveMotorRelative(_make_motor(), 200, velocity=0)

    def test_move_relative_timeout_negative(self):
        from raccoon.step.motor.steps import MoveMotorRelative
        with pytest.raises(ValueError, match="timeout"):
            MoveMotorRelative(_make_motor(), 200, timeout=-0.5)

    def test_stop_motor_check(self):
        from raccoon.step.motor.steps import MotorOff, MotorBrake, MotorPassiveBrake
        for cls in (MotorOff, MotorBrake, MotorPassiveBrake):
            with pytest.raises(TypeError, match="Motor"):
                cls("not a motor")

    def test_all_motor_steps_accept_valid_motor(self):
        from raccoon.step.motor.steps import (
            SetMotorPower, SetMotorVelocity, SetMotorDps,
            MoveMotorTo, MoveMotorRelative,
            MotorOff, MotorPassiveBrake, MotorBrake,
        )
        m = _make_motor()
        SetMotorPower(m, 50)
        SetMotorVelocity(m, 800)
        SetMotorDps(m, 180.0)
        MoveMotorTo(m, 500)
        MoveMotorRelative(m, 200)
        MotorOff(m)
        MotorPassiveBrake(m)
        MotorBrake(m)


# ===========================================================================
# Servo step validation tests
# ===========================================================================


@requires_libstp
class TestServoStepValidation:
    def test_valid_set_position(self):
        from raccoon.step.servo.steps import SetServoPosition
        SetServoPosition(_make_servo(), 90.0)

    def test_wrong_servo_type(self):
        from raccoon.step.servo.steps import SetServoPosition
        with pytest.raises(TypeError, match="Servo"):
            SetServoPosition("not a servo", 90.0)

    def test_int_instead_of_servo(self):
        from raccoon.step.servo.steps import SetServoPosition
        with pytest.raises(TypeError, match="Servo"):
            SetServoPosition(42, 90.0)

    def test_negative_duration(self):
        from raccoon.step.servo.steps import SetServoPosition
        with pytest.raises(ValueError, match="Duration"):
            SetServoPosition(_make_servo(), 90.0, duration=-1.0)

    def test_slow_servo_valid(self):
        from raccoon.step.servo.steps import SlowServo
        SlowServo(_make_servo(), 90.0)

    def test_slow_servo_wrong_servo(self):
        from raccoon.step.servo.steps import SlowServo
        with pytest.raises(TypeError, match="Servo"):
            SlowServo("bad", 90.0)

    def test_slow_servo_speed_zero(self):
        from raccoon.step.servo.steps import SlowServo
        with pytest.raises(ValueError, match="Speed"):
            SlowServo(_make_servo(), 90.0, speed=0.0)

    def test_slow_servo_speed_negative(self):
        from raccoon.step.servo.steps import SlowServo
        with pytest.raises(ValueError, match="Speed"):
            SlowServo(_make_servo(), 90.0, speed=-10.0)

    def test_slow_servo_bad_easing(self):
        from raccoon.step.servo.steps import SlowServo
        with pytest.raises(TypeError, match="easing"):
            SlowServo(_make_servo(), 90.0, easing="linear")

    def test_slow_servo_angle_not_number(self):
        from raccoon.step.servo.steps import SlowServo
        with pytest.raises(TypeError, match="angle"):
            SlowServo(_make_servo(), "ninety")

    def test_shake_servo_valid(self):
        from raccoon.step.servo.steps import ShakeServo
        ShakeServo(_make_servo(), 3.0, 60.0, 120.0)

    def test_shake_servo_wrong_servo(self):
        from raccoon.step.servo.steps import ShakeServo
        with pytest.raises(TypeError, match="Servo"):
            ShakeServo("bad", 3.0, 60.0, 120.0)

    def test_shake_servo_negative_duration(self):
        from raccoon.step.servo.steps import ShakeServo
        with pytest.raises(ValueError, match="Duration"):
            ShakeServo(_make_servo(), -1.0, 60.0, 120.0)


# ===========================================================================
# Logic step validation tests
# ===========================================================================


@requires_libstp
class TestDeferValidation:
    def test_valid(self):
        from raccoon.step.logic.defer import Defer
        Defer(lambda robot: _make_step())

    def test_not_callable(self):
        from raccoon.step.logic.defer import Defer
        with pytest.raises(TypeError, match="callable"):
            Defer("not a function")

    def test_int_not_callable(self):
        from raccoon.step.logic.defer import Defer
        with pytest.raises(TypeError, match="callable"):
            Defer(42)


@requires_libstp
class TestRunValidation:
    def test_valid(self):
        from raccoon.step.logic.defer import Run
        Run(lambda robot: None)

    def test_not_callable(self):
        from raccoon.step.logic.defer import Run
        with pytest.raises(TypeError, match="callable"):
            Run("not a function")


@requires_libstp
class TestDoWhileActiveValidation:
    def test_valid(self):
        from raccoon.step.logic.do_while import DoWhileActive
        DoWhileActive(_make_step(), _make_step())

    def test_reference_not_step(self):
        from raccoon.step.logic.do_while import DoWhileActive
        with pytest.raises(TypeError, match="reference_step"):
            DoWhileActive("not a step", _make_step())

    def test_task_not_step(self):
        from raccoon.step.logic.do_while import DoWhileActive
        with pytest.raises(TypeError, match="task must be"):
            DoWhileActive(_make_step(), "not a step")


# ===========================================================================
# Timing step validation tests
# ===========================================================================


@requires_libstp
class TestWaitForCheckpointValidation:
    def test_valid(self):
        from raccoon.step.timing.wait_for_checkpoint import WaitForCheckpoint
        WaitForCheckpoint(10.0)
        WaitForCheckpoint(0)

    def test_negative(self):
        from raccoon.step.timing.wait_for_checkpoint import WaitForCheckpoint
        with pytest.raises(ValueError):
            WaitForCheckpoint(-5.0)


@requires_libstp
class TestDoUntilCheckpointValidation:
    def test_valid(self):
        from raccoon.step.timing.do_until_checkpoint import DoUntilCheckpoint
        DoUntilCheckpoint(45.0, _make_step())

    def test_checkpoint_negative(self):
        from raccoon.step.timing.do_until_checkpoint import DoUntilCheckpoint
        with pytest.raises(ValueError, match="checkpoint"):
            DoUntilCheckpoint(-1.0, _make_step())

    def test_checkpoint_string(self):
        from raccoon.step.timing.do_until_checkpoint import DoUntilCheckpoint
        with pytest.raises(TypeError, match="number"):
            DoUntilCheckpoint("ten", _make_step())

    def test_step_not_step(self):
        from raccoon.step.timing.do_until_checkpoint import DoUntilCheckpoint
        with pytest.raises(TypeError, match="Step"):
            DoUntilCheckpoint(45.0, "not a step")


# ===========================================================================
# Composite step validation (Sequential, Parallel, Timeout, Loop)
# These already had validation — confirm it still works.
# ===========================================================================


@requires_libstp
class TestCompositeValidation:
    def test_sequential_rejects_non_steps(self):
        from raccoon.step.sequential import Sequential
        with pytest.raises(TypeError, match="not a Step"):
            Sequential(["not a step"])

    def test_sequential_rejects_non_list(self):
        from raccoon.step.sequential import Sequential
        with pytest.raises(TypeError, match="List"):
            Sequential("not a list")

    def test_parallel_rejects_non_steps(self):
        from raccoon.step.parallel import Parallel
        with pytest.raises(TypeError, match="not a Step"):
            Parallel(["not a step"])

    def test_timeout_rejects_non_step(self):
        from raccoon.step.timeout import Timeout
        with pytest.raises(TypeError, match="Step"):
            Timeout("not a step", 5.0)

    def test_timeout_rejects_non_positive_seconds(self):
        from raccoon.step.timeout import Timeout
        with pytest.raises(ValueError, match="positive"):
            Timeout(_make_step(), 0)

    def test_loop_forever_rejects_non_step(self):
        from raccoon.step.logic.loop import LoopForever
        with pytest.raises(TypeError, match="Step"):
            LoopForever("not a step")

    def test_loop_for_rejects_non_step(self):
        from raccoon.step.logic.loop import LoopFor
        with pytest.raises(TypeError, match="Step"):
            LoopFor("not a step", 3)

    def test_loop_for_rejects_bad_iterations(self):
        from raccoon.step.logic.loop import LoopFor
        with pytest.raises(ValueError, match="positive"):
            LoopFor(_make_step(), 0)
        with pytest.raises(ValueError, match="positive"):
            LoopFor(_make_step(), -1)


# ===========================================================================
# Builder validation tests — construction-time checks on setter methods
# ===========================================================================


@requires_libstp
class TestDriveBuilderValidation:
    def test_until_wrong_type(self):
        from raccoon.step.motion.drive_dsl import DriveForwardBuilder
        b = DriveForwardBuilder()
        with pytest.raises(TypeError, match="StopCondition"):
            b.until("stop when black")

    def test_until_int(self):
        from raccoon.step.motion.drive_dsl import DriveForwardBuilder
        b = DriveForwardBuilder()
        with pytest.raises(TypeError, match="StopCondition"):
            b.until(42)

    def test_speed_too_high(self):
        from raccoon.step.motion.drive_dsl import DriveForwardBuilder
        b = DriveForwardBuilder()
        with pytest.raises(ValueError, match="speed"):
            b.speed(5.0)

    def test_speed_zero(self):
        from raccoon.step.motion.drive_dsl import DriveForwardBuilder
        b = DriveForwardBuilder()
        with pytest.raises(ValueError, match="speed"):
            b.speed(0.0)

    def test_speed_string(self):
        from raccoon.step.motion.drive_dsl import DriveForwardBuilder
        b = DriveForwardBuilder()
        with pytest.raises(TypeError, match="speed"):
            b.speed("fast")

    def test_cm_negative(self):
        from raccoon.step.motion.drive_dsl import DriveForwardBuilder
        b = DriveForwardBuilder()
        with pytest.raises(ValueError, match="cm"):
            b.cm(-5)

    def test_cm_zero(self):
        from raccoon.step.motion.drive_dsl import DriveForwardBuilder
        b = DriveForwardBuilder()
        with pytest.raises(ValueError, match="cm"):
            b.cm(0)

    def test_cm_string(self):
        from raccoon.step.motion.drive_dsl import DriveForwardBuilder
        b = DriveForwardBuilder()
        with pytest.raises(TypeError, match="cm"):
            b.cm("twenty")

    def test_valid_chaining(self):
        from raccoon.step.motion.drive_dsl import DriveForwardBuilder
        from raccoon.step.condition import after_seconds
        b = DriveForwardBuilder()
        result = b.speed(0.5).until(after_seconds(5))
        assert result is b

    def test_all_drive_builders(self):
        """All drive/strafe builders share the same validation."""
        from raccoon.step.motion.drive_dsl import (
            DriveForwardBuilder, DriveBackwardBuilder,
            StrafeLeftBuilder, StrafeRightBuilder,
        )
        for cls in (DriveForwardBuilder, DriveBackwardBuilder,
                    StrafeLeftBuilder, StrafeRightBuilder):
            b = cls()
            with pytest.raises(TypeError, match="StopCondition"):
                b.until("bad")
            with pytest.raises(ValueError, match="speed"):
                b.speed(2.0)
            with pytest.raises(ValueError, match="cm"):
                b.cm(-1)


@requires_libstp
class TestTurnBuilderValidation:
    def test_until_wrong_type(self):
        from raccoon.step.motion.turn_dsl import TurnLeftBuilder
        b = TurnLeftBuilder()
        with pytest.raises(TypeError, match="StopCondition"):
            b.until("stop")

    def test_degrees_negative(self):
        from raccoon.step.motion.turn_dsl import TurnLeftBuilder
        b = TurnLeftBuilder()
        with pytest.raises(ValueError, match="degrees"):
            b.degrees(-90)

    def test_degrees_zero(self):
        from raccoon.step.motion.turn_dsl import TurnLeftBuilder
        b = TurnLeftBuilder()
        with pytest.raises(ValueError, match="degrees"):
            b.degrees(0)

    def test_degrees_string(self):
        from raccoon.step.motion.turn_dsl import TurnLeftBuilder
        b = TurnLeftBuilder()
        with pytest.raises(TypeError, match="degrees"):
            b.degrees("ninety")

    def test_speed_out_of_range(self):
        from raccoon.step.motion.turn_dsl import TurnRightBuilder
        b = TurnRightBuilder()
        with pytest.raises(ValueError, match="speed"):
            b.speed(1.5)

    def test_all_turn_builders(self):
        from raccoon.step.motion.turn_dsl import TurnLeftBuilder, TurnRightBuilder
        for cls in (TurnLeftBuilder, TurnRightBuilder):
            b = cls()
            with pytest.raises(TypeError, match="StopCondition"):
                b.until(42)
            with pytest.raises(ValueError, match="speed"):
                b.speed(0.0)
            with pytest.raises(ValueError, match="degrees"):
                b.degrees(-1)


# ===========================================================================
# DSL factory function validation (passes through to builder/constructor)
# ===========================================================================


@requires_libstp
class TestDslFactoryValidation:
    def test_drive_forward_builder_until_checked(self):
        """drive_forward().until('bad') raises at the builder setter."""
        from raccoon.step.motion.drive_dsl import drive_forward
        b = drive_forward(cm=25)
        with pytest.raises(TypeError, match="StopCondition"):
            b.until("stop")

    def test_turn_left_builder_speed_checked(self):
        from raccoon.step.motion.turn_dsl import turn_left
        b = turn_left(degrees=90)
        with pytest.raises(ValueError, match="speed"):
            b.speed(3.0)
