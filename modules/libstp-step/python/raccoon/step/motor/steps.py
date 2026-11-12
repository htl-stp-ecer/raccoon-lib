from __future__ import annotations

import asyncio
import math
from enum import Enum
from typing import TYPE_CHECKING

from raccoon.hal import IMotor
from raccoon.step import Step
from raccoon.step.annotation import dsl, dsl_step

# BEMF sampling rate in Hz (5ms interval on firmware) — must match motor_adapter.cpp
_BEMF_SAMPLE_RATE = 200.0

if TYPE_CHECKING:
    from raccoon.robot.api import GenericRobot


class StopMode(Enum):
    """Enumeration of motor stopping behaviours.

    OFF -- Remove power and let the motor coast (free-spin).
    PASSIVE_BRAKE -- Command zero power; H-bridge shorts the leads for
        electrical braking, but no active position hold.
    ACTIVE_BRAKE -- Engage the firmware stop latch to actively resist
        external forces and hold the current shaft position.
    """

    OFF = "off"
    PASSIVE_BRAKE = "passive_brake"
    ACTIVE_BRAKE = "active_brake"


# ---------------------------------------------------------------------------
# Step classes
# ---------------------------------------------------------------------------


@dsl_step(tags=["motor", "actuator"])
class SetMotorPower(Step):
    """Set a motor to open-loop percent power.

    Commands the motor at a raw duty-cycle percentage without any feedback
    control. The command is applied immediately and the step completes
    without waiting -- the motor keeps spinning until another command
    (or a stop step) is issued.

    Args:
        motor: The motor to control, obtained from the robot hardware map
            (e.g. ``robot.motor(0)``).
        percent: Duty-cycle power from -100 (full reverse) to 100 (full
            forward). Values outside this range raise ``ValueError``.

    Example::

        from raccoon.step.motor import motor_power, motor_off

        # Spin the claw motor forward at half power for 2 seconds
        sequence(
            motor_power(robot.motor(1), 50),
            wait(2.0),
            motor_off(robot.motor(1)),
        )
    """

    def __init__(self, motor: IMotor, percent: int) -> None:
        super().__init__()
        if not hasattr(motor, "port"):
            msg = f"motor must be a Motor (has .port), got {type(motor).__name__}"
            raise TypeError(msg)
        self._motor = motor
        self._percent = int(percent)
        if not (-100 <= self._percent <= 100):
            msg = f"percent must be -100..100, got {self._percent}"
            raise ValueError(msg)

    def required_resources(self) -> frozenset[str]:
        return frozenset({f"motor:{self._motor.port}"})

    def _generate_signature(self) -> str:
        return f"SetMotorPower(port={self._motor.port},percent={self._percent})"

    async def _execute_step(self, robot: GenericRobot) -> None:
        self._motor.set_speed(self._percent)


@dsl_step(tags=["motor", "actuator"])
class SetMotorVelocity(Step):
    """Set a motor to closed-loop velocity in firmware BEMF units.

    Commands the motor's firmware PID controller to maintain a target
    velocity expressed in raw BEMF tick units. The step completes
    immediately; the motor continues at the requested speed until changed.

    For a human-friendly velocity interface, prefer ``motor_dps`` which
    accepts degrees per second.

    Args:
        motor: The motor to control, obtained from the robot hardware map
            (e.g. ``robot.motor(0)``).
        velocity: Target velocity in firmware BEMF units (ticks per BEMF
            sample period). Positive values drive forward; negative values
            drive in reverse.

    Example::

        from raccoon.step.motor import motor_velocity, motor_brake

        # Drive the left wheel at 800 BEMF units, then brake
        sequence(
            motor_velocity(robot.motor(0), 800),
            wait(3.0),
            motor_brake(robot.motor(0)),
        )
    """

    def __init__(self, motor: IMotor, velocity: int) -> None:
        super().__init__()
        if not hasattr(motor, "port"):
            msg = f"motor must be a Motor (has .port), got {type(motor).__name__}"
            raise TypeError(msg)
        self._motor = motor
        self._velocity = int(velocity)

    def required_resources(self) -> frozenset[str]:
        return frozenset({f"motor:{self._motor.port}"})

    def _generate_signature(self) -> str:
        return f"SetMotorVelocity(port={self._motor.port},velocity={self._velocity})"

    async def _execute_step(self, robot: GenericRobot) -> None:
        self._motor.set_velocity(self._velocity)


@dsl_step(tags=["motor", "actuator"])
class SetMotorDps(Step):
    """Set a motor to closed-loop velocity in degrees per second.

    Converts the requested angular velocity from degrees per second into
    firmware BEMF units using the motor's calibration data, then engages
    the firmware's closed-loop velocity controller. The step completes
    immediately; the motor continues at the requested speed until changed.

    Prerequisites:
        The motor must have valid calibration data (``ticks_to_rad``) so
        the deg/s to BEMF conversion is accurate. Run the motor
        calibration step first if calibration has not been performed.

    Args:
        motor: The motor to control, obtained from the robot hardware map
            (e.g. ``robot.motor(0)``).
        dps: Target angular velocity in degrees per second. Positive
            values drive forward; negative values drive in reverse.

    Example::

        from raccoon.step.motor import motor_dps, motor_brake

        # Spin the left wheel at 180 deg/s for 2 seconds
        sequence(
            motor_dps(robot.motor(0), 180.0),
            wait(2.0),
            motor_brake(robot.motor(0)),
        )
    """

    def __init__(self, motor: IMotor, dps: float) -> None:
        super().__init__()
        if not hasattr(motor, "port"):
            msg = f"motor must be a Motor (has .port), got {type(motor).__name__}"
            raise TypeError(msg)
        self._motor = motor
        self._dps = float(dps)

    def required_resources(self) -> frozenset[str]:
        return frozenset({f"motor:{self._motor.port}"})

    def _generate_signature(self) -> str:
        return f"SetMotorDps(port={self._motor.port},dps={self._dps})"

    async def _execute_step(self, robot: GenericRobot) -> None:
        ticks_to_rad = self._motor.get_calibration().ticks_to_rad
        self.info(f"Motor {self._motor.port} dps: {self._dps} -> {ticks_to_rad} rad/s")
        rad_per_sec = self._dps * math.pi / 180.0
        self.info(f"Motor {self._motor.port} rad/s: {rad_per_sec}")
        bemf_target = int(round(rad_per_sec / (ticks_to_rad * _BEMF_SAMPLE_RATE)))
        self.info(f"Motor {self._motor.port} bemf: {bemf_target}")
        self._motor.set_velocity(bemf_target)


_DEFAULT_POSITION_TOLERANCE = 3  # ticks; matches the firmware hold deadband


async def _wait_until_reached(
    step: Step,
    motor: IMotor,
    goal: int,
    tolerance: int,
    timeout: float | None,
    label: str,
) -> None:
    """Poll the motor position until it is within ``tolerance`` ticks of ``goal``."""
    loop = asyncio.get_running_loop()
    start = loop.time()
    while abs(motor.get_position() - goal) > tolerance:
        if timeout is not None and (loop.time() - start) >= timeout:
            step.warn(
                f"{label} timed out after {timeout:.1f}s "
                f"(target={goal}, current={motor.get_position()})"
            )
            break
        await asyncio.sleep(0.01)


@dsl_step(tags=["motor", "actuator"])
class MoveMotorTo(Step):
    """Move a motor to an absolute encoder position and wait for completion.

    Commands the motor's firmware position controller to drive to the given
    absolute encoder tick count. The step blocks (yielding to the async
    loop) until the measured position is within ``tolerance`` ticks of the
    target, or until the optional timeout expires.

    The motor keeps holding the target position after the step finishes. It
    stays in position mode until another motor step (e.g. ``motor_off``,
    ``motor_brake`` or a velocity/speed step) switches it.

    Args:
        motor: The motor to control, obtained from the robot hardware map
            (e.g. ``robot.motor(2)``).
        position: Target position in absolute encoder ticks.
        velocity: Movement speed in firmware ticks/s. Must be positive.
            Defaults to 1000.
        tolerance: Allowed distance to the target in ticks for the move to
            count as reached. Must be >= 0. Defaults to 3.
        timeout: Maximum seconds to wait for the move to finish. ``None``
            (the default) means wait indefinitely. If the timeout fires
            a warning is logged and the step returns without stopping the
            motor.

    Example::

        from raccoon.step.motor import motor_move_to

        # Raise the arm to encoder position 500 at speed 800, with a 3s safety timeout
        motor_move_to(robot.motor(2), position=500, velocity=800, timeout=3.0)
    """

    def __init__(
        self,
        motor: IMotor,
        position: int,
        velocity: int = 1000,
        timeout: float | None = None,
        tolerance: int = _DEFAULT_POSITION_TOLERANCE,
    ) -> None:
        super().__init__()
        if not hasattr(motor, "port"):
            msg = f"motor must be a Motor (has .port), got {type(motor).__name__}"
            raise TypeError(msg)
        self._motor = motor
        self._position = int(position)
        self._velocity = int(velocity)
        self._timeout = timeout
        self._tolerance = int(tolerance)
        if self._tolerance < 0:
            msg = f"tolerance must be >= 0, got {self._tolerance}"
            raise ValueError(msg)
        if self._velocity <= 0:
            msg = f"velocity must be > 0, got {self._velocity}"
            raise ValueError(msg)
        if timeout is not None and (not isinstance(timeout, int | float) or timeout <= 0):
            msg = f"timeout must be > 0, got {timeout}"
            raise ValueError(msg)

    def required_resources(self) -> frozenset[str]:
        return frozenset({f"motor:{self._motor.port}"})

    def _generate_signature(self) -> str:
        return f"MoveMotorTo(port={self._motor.port}," f"pos={self._position},vel={self._velocity})"

    async def _execute_step(self, robot: GenericRobot) -> None:
        self._motor.move_to_position(self._velocity, self._position)
        await _wait_until_reached(
            self, self._motor, self._position, self._tolerance, self._timeout, "MoveMotorTo"
        )


@dsl_step(tags=["motor", "actuator"])
class MoveMotorRelative(Step):
    """Move a motor by a relative encoder delta and wait for completion.

    Commands the motor's firmware position controller to move by the given
    number of encoder ticks relative to its current position (resolved to an
    absolute target when the step starts). The step blocks until the measured
    position is within ``tolerance`` ticks of that target, or until the
    optional timeout expires.

    The motor keeps holding the target position after the step finishes. It
    stays in position mode until another motor step switches it.

    Args:
        motor: The motor to control, obtained from the robot hardware map
            (e.g. ``robot.motor(2)``).
        delta: Number of encoder ticks to move. Positive values move
            forward; negative values move in reverse.
        velocity: Movement speed in firmware ticks/s. Must be positive.
            Defaults to 1000.
        tolerance: Allowed distance to the target in ticks for the move to
            count as reached. Must be >= 0. Defaults to 3.
        timeout: Maximum seconds to wait for the move to finish. ``None``
            (the default) means wait indefinitely. If the timeout fires
            a warning is logged and the step returns without stopping the
            motor.

    Example::

        from raccoon.step.motor import motor_move_relative, motor_brake

        # Rotate the arm motor 200 ticks forward, then brake
        sequence(
            motor_move_relative(robot.motor(2), delta=200, velocity=600),
            motor_brake(robot.motor(2)),
        )
    """

    def __init__(
        self,
        motor: IMotor,
        delta: int,
        velocity: int = 1000,
        timeout: float | None = None,
        tolerance: int = _DEFAULT_POSITION_TOLERANCE,
    ) -> None:
        super().__init__()
        if not hasattr(motor, "port"):
            msg = f"motor must be a Motor (has .port), got {type(motor).__name__}"
            raise TypeError(msg)
        self._motor = motor
        self._delta = int(delta)
        self._velocity = int(velocity)
        self._timeout = timeout
        self._tolerance = int(tolerance)
        if self._tolerance < 0:
            msg = f"tolerance must be >= 0, got {self._tolerance}"
            raise ValueError(msg)
        if self._velocity <= 0:
            msg = f"velocity must be > 0, got {self._velocity}"
            raise ValueError(msg)
        if timeout is not None and (not isinstance(timeout, int | float) or timeout <= 0):
            msg = f"timeout must be > 0, got {timeout}"
            raise ValueError(msg)

    def required_resources(self) -> frozenset[str]:
        return frozenset({f"motor:{self._motor.port}"})

    def _generate_signature(self) -> str:
        return (
            f"MoveMotorRelative(port={self._motor.port},"
            f"delta={self._delta},vel={self._velocity})"
        )

    async def _execute_step(self, robot: GenericRobot) -> None:
        goal = self._motor.get_position() + self._delta
        self._motor.move_to_position(self._velocity, goal)
        await _wait_until_reached(
            self, self._motor, goal, self._tolerance, self._timeout, "MoveMotorRelative"
        )


def _validate_degrees(degrees: float) -> None:
    """Single place that validates a degree-based move (the angle).

    The calibration (finite, non-zero ``ticks_to_rad``) is already guaranteed by
    the C++ ``MotorCalibration::validate()`` when a motor is created or recalibrated.
    """
    if not math.isfinite(degrees):
        msg = f"degrees must be finite, got {degrees}"
        raise ValueError(msg)


def _degrees_to_ticks(motor: IMotor, degrees: float) -> int:
    """Pure conversion with the motor calibration; call ``_validate_degrees`` first."""
    return round(math.radians(degrees) / motor.get_calibration().ticks_to_rad)


class _MoveMotorDegreesBase(Step):
    """Shared validation and execution for the degree-based move steps.

    Identical to ``MoveMotorTo`` / ``MoveMotorRelative`` (same ``velocity``,
    ``timeout`` and ``tolerance``) except that the target is an angle.
    """

    _LABEL = "MoveMotorDegrees"

    def __init__(
        self,
        motor: IMotor,
        velocity: int,
        timeout: float | None,
        tolerance: int,
    ) -> None:
        super().__init__()
        if not hasattr(motor, "port"):
            msg = f"motor must be a Motor (has .port), got {type(motor).__name__}"
            raise TypeError(msg)
        self._motor = motor
        self._velocity = int(velocity)
        self._timeout = timeout
        self._tolerance = int(tolerance)
        if self._velocity <= 0:
            msg = f"velocity must be > 0, got {self._velocity}"
            raise ValueError(msg)
        if timeout is not None and (not isinstance(timeout, int | float) or timeout <= 0):
            msg = f"timeout must be > 0, got {timeout}"
            raise ValueError(msg)
        if self._tolerance < 0:
            msg = f"tolerance must be >= 0, got {self._tolerance}"
            raise ValueError(msg)

    def required_resources(self) -> frozenset[str]:
        return frozenset({f"motor:{self._motor.port}"})

    def _goal_ticks(self) -> int:
        raise NotImplementedError

    def _angle_degrees(self) -> float:
        raise NotImplementedError

    async def _execute_step(self, robot: GenericRobot) -> None:
        _validate_degrees(self._angle_degrees())
        goal = self._goal_ticks()
        self._motor.move_to_position(self._velocity, goal)
        await _wait_until_reached(
            self, self._motor, goal, self._tolerance, self._timeout, self._LABEL
        )


@dsl_step(tags=["motor", "actuator"])
class MoveMotorToDegrees(_MoveMotorDegreesBase):
    """Move a motor to an absolute angle in degrees and wait for completion.

    Same as ``move_motor_to`` but the target is an angle in degrees instead of
    encoder ticks. Degrees are converted with the motor's ``ticks_to_rad``
    calibration; 0 degrees is the position-counter zero (see
    ``reset_position_counter``). The step blocks until the measured position
    is within ``tolerance`` ticks of the target, or until the optional timeout
    expires.

    The motor keeps holding the target position after the step finishes. It
    stays in position mode until another motor step (e.g. ``motor_off``,
    ``motor_brake`` or a velocity/speed step) switches it.

    Args:
        motor: The motor to control, obtained from the robot hardware map
            (e.g. ``robot.motor(2)``). Needs a valid ``ticks_to_rad``.
        degrees: Target angle in degrees (signed).
        velocity: Movement speed in firmware ticks/s. Must be positive.
            Defaults to 1000.
        timeout: Maximum seconds to wait for the move to finish. ``None``
            (the default) means wait indefinitely. If the timeout fires
            a warning is logged and the step returns without stopping the
            motor.
        tolerance: Allowed distance to the target in ticks for the move to
            count as reached. Must be >= 0. Defaults to 3.

    Example::

        from raccoon.step.motor import move_motor_to_degrees

        # Swing the arm to 90 degrees at speed 800, with a 3s safety timeout
        move_motor_to_degrees(robot.motor(2), degrees=90, velocity=800, timeout=3.0)
    """

    _LABEL = "MoveMotorToDegrees"

    def __init__(
        self,
        motor: IMotor,
        degrees: float,
        velocity: int = 1000,
        timeout: float | None = None,
        tolerance: int = _DEFAULT_POSITION_TOLERANCE,
    ) -> None:
        super().__init__(motor, velocity, timeout, tolerance)
        self._degrees = float(degrees)

    def _generate_signature(self) -> str:
        return (
            f"MoveMotorToDegrees(port={self._motor.port},"
            f"deg={self._degrees},vel={self._velocity})"
        )

    def _angle_degrees(self) -> float:
        return self._degrees

    def _goal_ticks(self) -> int:
        return _degrees_to_ticks(self._motor, self._degrees)


@dsl_step(tags=["motor", "actuator"])
class MoveMotorRelativeDegrees(_MoveMotorDegreesBase):
    """Move a motor by a relative angle in degrees and wait for completion.

    Same as ``move_motor_relative`` but the delta is an angle in degrees
    instead of encoder ticks. The target is resolved to an absolute position
    (current position + delta) when the step starts. The step blocks until the
    measured position is within ``tolerance`` ticks of that target, or until
    the optional timeout expires.

    The motor keeps holding the target position after the step finishes. It
    stays in position mode until another motor step switches it.

    Args:
        motor: The motor to control, obtained from the robot hardware map
            (e.g. ``robot.motor(2)``). Needs a valid ``ticks_to_rad``.
        delta_degrees: Angle to move in degrees. Positive values move
            forward; negative values move in reverse.
        velocity: Movement speed in firmware ticks/s. Must be positive.
            Defaults to 1000.
        timeout: Maximum seconds to wait for the move to finish. ``None``
            (the default) means wait indefinitely. If the timeout fires
            a warning is logged and the step returns without stopping the
            motor.
        tolerance: Allowed distance to the target in ticks for the move to
            count as reached. Must be >= 0. Defaults to 3.

    Example::

        from raccoon.step.motor import move_motor_relative_degrees

        # Turn the arm motor a quarter turn forward
        move_motor_relative_degrees(robot.motor(2), delta_degrees=90, velocity=600)
    """

    _LABEL = "MoveMotorRelativeDegrees"

    def __init__(
        self,
        motor: IMotor,
        delta_degrees: float,
        velocity: int = 1000,
        timeout: float | None = None,
        tolerance: int = _DEFAULT_POSITION_TOLERANCE,
    ) -> None:
        super().__init__(motor, velocity, timeout, tolerance)
        self._delta_degrees = float(delta_degrees)

    def _generate_signature(self) -> str:
        return (
            f"MoveMotorRelativeDegrees(port={self._motor.port},"
            f"delta_deg={self._delta_degrees},vel={self._velocity})"
        )

    def _angle_degrees(self) -> float:
        return self._delta_degrees

    def _goal_ticks(self) -> int:
        return self._motor.get_position() + _degrees_to_ticks(self._motor, self._delta_degrees)


@dsl(hidden=True)
class StopMotor(Step):
    """Stop a motor using the specified mode."""

    def __init__(self, motor: IMotor, mode: StopMode = StopMode.ACTIVE_BRAKE) -> None:
        super().__init__()
        if not hasattr(motor, "port"):
            msg = f"motor must be a Motor (has .port), got {type(motor).__name__}"
            raise TypeError(msg)
        self._motor = motor
        self._mode = mode

    def required_resources(self) -> frozenset[str]:
        return frozenset({f"motor:{self._motor.port}"})

    def _generate_signature(self) -> str:
        return f"StopMotor(port={self._motor.port},mode={self._mode.value})"

    async def _execute_step(self, robot: GenericRobot) -> None:
        if self._mode == StopMode.OFF:
            self._motor.off()
        elif self._mode == StopMode.PASSIVE_BRAKE:
            self._motor.set_speed(0)
        elif self._mode == StopMode.ACTIVE_BRAKE:
            self._motor.brake()


@dsl_step(tags=["motor", "actuator"])
class MotorOff(StopMotor):
    """Turn a motor off, allowing it to coast freely.

    Removes all power from the motor so it spins down under friction
    alone. The motor shaft is not held in place and can be back-driven.
    Use this when you want the mechanism to move freely (e.g. letting an
    arm fall under gravity).

    Args:
        motor: The motor to turn off, obtained from the robot hardware map
            (e.g. ``robot.motor(0)``).

    Example::

        from raccoon.step.motor import motor_power, motor_off

        # Run a motor briefly, then let it coast to a stop
        sequence(
            motor_power(robot.motor(3), 80),
            wait(1.5),
            motor_off(robot.motor(3)),
        )
    """

    def __init__(self, motor: IMotor) -> None:
        super().__init__(motor, StopMode.OFF)


@dsl_step(tags=["motor", "actuator"])
class MotorPassiveBrake(StopMotor):
    """Passively brake a motor by commanding zero power.

    Sets the motor power to zero, which causes the H-bridge to short the
    motor leads. This provides electrical braking that decelerates the
    motor faster than coasting (``motor_off``), but does not actively hold
    position once the motor stops.

    Args:
        motor: The motor to brake, obtained from the robot hardware map
            (e.g. ``robot.motor(0)``).

    Example::

        from raccoon.step.motor import motor_velocity, motor_passive_brake

        # Drive forward then passively brake
        sequence(
            motor_velocity(robot.motor(0), 600),
            wait(2.0),
            motor_passive_brake(robot.motor(0)),
        )
    """

    def __init__(self, motor: IMotor) -> None:
        super().__init__(motor, StopMode.PASSIVE_BRAKE)


@dsl_step(tags=["motor", "actuator"])
class MotorBrake(StopMotor):
    """Actively brake a motor and hold its current position.

    Engages the firmware's active brake (stop latch), which commands the
    motor controller to resist any external force and maintain the
    current shaft position. This is the strongest stop mode and is
    appropriate when the mechanism must not move (e.g. holding an arm
    up against gravity).

    Args:
        motor: The motor to brake, obtained from the robot hardware map
            (e.g. ``robot.motor(2)``).

    Example::

        from raccoon.step.motor import motor_move_to, motor_brake

        # Move the arm to a target, then lock it in place
        sequence(
            motor_move_to(robot.motor(2), position=400),
            motor_brake(robot.motor(2)),
        )
    """

    def __init__(self, motor: IMotor) -> None:
        super().__init__(motor, StopMode.ACTIVE_BRAKE)


__all__ = [
    "StopMode",
    "SetMotorPower",
    "SetMotorVelocity",
    "SetMotorDps",
    "MoveMotorTo",
    "MoveMotorRelative",
    "MoveMotorToDegrees",
    "MoveMotorRelativeDegrees",
    "StopMotor",
    "MotorOff",
    "MotorPassiveBrake",
    "MotorBrake",
]
