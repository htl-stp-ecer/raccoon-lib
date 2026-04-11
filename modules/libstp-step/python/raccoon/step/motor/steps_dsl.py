"""Auto-generated step builders and DSL functions — DO NOT EDIT.

Source: steps.py
"""

from __future__ import annotations

_UNSET = object()

from raccoon.step.step_builder import StepBuilder
from raccoon.step.condition import StopCondition
from raccoon.step.annotation import dsl
from .steps import (
    SetMotorPower,
    SetMotorVelocity,
    SetMotorDps,
    MoveMotorTo,
    MoveMotorRelative,
    MotorOff,
    MotorPassiveBrake,
    MotorBrake,
)

from raccoon.hal import IMotor


class SetMotorPowerBuilder(StepBuilder):
    """Builder for SetMotorPower. Auto-generated — do not edit."""

    def __init__(self):
        super().__init__()
        self._motor = _UNSET
        self._percent = _UNSET

    def motor(self, value: IMotor):
        self._motor = value
        return self

    def percent(self, value: int):
        self._percent = value
        return self

    def _build(self):
        kwargs = {}
        if self._motor is not _UNSET:
            kwargs["motor"] = self._motor
        if self._percent is not _UNSET:
            kwargs["percent"] = self._percent
        return SetMotorPower(**kwargs)


@dsl(tags=["motor", "actuator"])
def set_motor_power(motor: IMotor = _UNSET, percent: int = _UNSET):
    """
    Set a motor to open-loop percent power.

    Commands the motor at a raw duty-cycle percentage without any feedback
    control. The command is applied immediately and the step completes
    without waiting -- the motor keeps spinning until another command
    (or a stop step) is issued.

    Args:
        motor: The motor to control, obtained from the robot hardware map (e.g. ``robot.motor(0)``).
        percent: Duty-cycle power from -100 (full reverse) to 100 (full forward). Values outside this range raise ``ValueError``.

    Returns:
        A SetMotorPowerBuilder (chainable via ``.motor()``, ``.percent()``, ``.on_anomaly()``, ``.skip_timing()``).

    Example::

        from raccoon.step.motor import motor_power, motor_off

        # Spin the claw motor forward at half power for 2 seconds
        sequence(
            motor_power(robot.motor(1), 50),
            wait(2.0),
            motor_off(robot.motor(1)),
        )
    """
    b = SetMotorPowerBuilder()
    if motor is not _UNSET:
        b._motor = motor
    if percent is not _UNSET:
        b._percent = percent
    return b


class SetMotorVelocityBuilder(StepBuilder):
    """Builder for SetMotorVelocity. Auto-generated — do not edit."""

    def __init__(self):
        super().__init__()
        self._motor = _UNSET
        self._velocity = _UNSET

    def motor(self, value: IMotor):
        self._motor = value
        return self

    def velocity(self, value: int):
        self._velocity = value
        return self

    def _build(self):
        kwargs = {}
        if self._motor is not _UNSET:
            kwargs["motor"] = self._motor
        if self._velocity is not _UNSET:
            kwargs["velocity"] = self._velocity
        return SetMotorVelocity(**kwargs)


@dsl(tags=["motor", "actuator"])
def set_motor_velocity(motor: IMotor = _UNSET, velocity: int = _UNSET):
    """
    Set a motor to closed-loop velocity in firmware BEMF units.

    Commands the motor's firmware PID controller to maintain a target
    velocity expressed in raw BEMF tick units. The step completes
    immediately; the motor continues at the requested speed until changed.

    For a human-friendly velocity interface, prefer ``motor_dps`` which
    accepts degrees per second.

    Args:
        motor: The motor to control, obtained from the robot hardware map (e.g. ``robot.motor(0)``).
        velocity: Target velocity in firmware BEMF units (ticks per BEMF sample period). Positive values drive forward; negative values drive in reverse.

    Returns:
        A SetMotorVelocityBuilder (chainable via ``.motor()``, ``.velocity()``, ``.on_anomaly()``, ``.skip_timing()``).

    Example::

        from raccoon.step.motor import motor_velocity, motor_brake

        # Drive the left wheel at 800 BEMF units, then brake
        sequence(
            motor_velocity(robot.motor(0), 800),
            wait(3.0),
            motor_brake(robot.motor(0)),
        )
    """
    b = SetMotorVelocityBuilder()
    if motor is not _UNSET:
        b._motor = motor
    if velocity is not _UNSET:
        b._velocity = velocity
    return b


class SetMotorDpsBuilder(StepBuilder):
    """Builder for SetMotorDps. Auto-generated — do not edit."""

    def __init__(self):
        super().__init__()
        self._motor = _UNSET
        self._dps = _UNSET

    def motor(self, value: IMotor):
        self._motor = value
        return self

    def dps(self, value: float):
        self._dps = value
        return self

    def _build(self):
        kwargs = {}
        if self._motor is not _UNSET:
            kwargs["motor"] = self._motor
        if self._dps is not _UNSET:
            kwargs["dps"] = self._dps
        return SetMotorDps(**kwargs)


@dsl(tags=["motor", "actuator"])
def set_motor_dps(motor: IMotor = _UNSET, dps: float = _UNSET):
    """
    Set a motor to closed-loop velocity in degrees per second.

    Converts the requested angular velocity from degrees per second into
    firmware BEMF units using the motor's calibration data, then engages
    the firmware's closed-loop velocity controller. The step completes
    immediately; the motor continues at the requested speed until changed.

    Prerequisites:
        The motor must have valid calibration data (``ticks_to_rad``) so
        the deg/s to BEMF conversion is accurate. Run the motor
        calibration step first if calibration has not been performed.

    Args:
        motor: The motor to control, obtained from the robot hardware map (e.g. ``robot.motor(0)``).
        dps: Target angular velocity in degrees per second. Positive values drive forward; negative values drive in reverse.

    Returns:
        A SetMotorDpsBuilder (chainable via ``.motor()``, ``.dps()``, ``.on_anomaly()``, ``.skip_timing()``).

    Example::

        from raccoon.step.motor import motor_dps, motor_brake

        # Spin the left wheel at 180 deg/s for 2 seconds
        sequence(
            motor_dps(robot.motor(0), 180.0),
            wait(2.0),
            motor_brake(robot.motor(0)),
        )
    """
    b = SetMotorDpsBuilder()
    if motor is not _UNSET:
        b._motor = motor
    if dps is not _UNSET:
        b._dps = dps
    return b


class MoveMotorToBuilder(StepBuilder):
    """Builder for MoveMotorTo. Auto-generated — do not edit."""

    def __init__(self):
        super().__init__()
        self._motor = _UNSET
        self._position = _UNSET
        self._velocity = 1000
        self._timeout = None

    def motor(self, value: IMotor):
        self._motor = value
        return self

    def position(self, value: int):
        self._position = value
        return self

    def velocity(self, value: int):
        self._velocity = value
        return self

    def timeout(self, value: Optional[float]):
        self._timeout = value
        return self

    def _build(self):
        kwargs = {}
        if self._motor is not _UNSET:
            kwargs["motor"] = self._motor
        if self._position is not _UNSET:
            kwargs["position"] = self._position
        kwargs["velocity"] = self._velocity
        kwargs["timeout"] = self._timeout
        return MoveMotorTo(**kwargs)


@dsl(tags=["motor", "actuator"])
def move_motor_to(
    motor: IMotor = _UNSET,
    position: int = _UNSET,
    velocity: int = 1000,
    timeout: Optional[float] = None,
):
    """
    Move a motor to an absolute encoder position and wait for completion.

    Commands the motor's firmware position controller to drive to the given
    absolute encoder tick count. The step blocks (yielding to the async
    loop) until the firmware reports the move is complete, or until the
    optional timeout expires.

    Args:
        motor: The motor to control, obtained from the robot hardware map (e.g. ``robot.motor(2)``).
        position: Target position in absolute encoder ticks.
        velocity: Movement speed in firmware ticks/s. Must be positive. Defaults to 1000.
        timeout: Maximum seconds to wait for the move to finish. ``None`` (the default) means wait indefinitely. If the timeout fires a warning is logged and the step returns without stopping the motor.

    Returns:
        A MoveMotorToBuilder (chainable via ``.motor()``, ``.position()``, ``.velocity()``, ``.timeout()``, ``.on_anomaly()``, ``.skip_timing()``).

    Example::

        from raccoon.step.motor import motor_move_to

        # Raise the arm to encoder position 500 at speed 800, with a 3s safety timeout
        motor_move_to(robot.motor(2), position=500, velocity=800, timeout=3.0)
    """
    b = MoveMotorToBuilder()
    if motor is not _UNSET:
        b._motor = motor
    if position is not _UNSET:
        b._position = position
    b._velocity = velocity
    b._timeout = timeout
    return b


class MoveMotorRelativeBuilder(StepBuilder):
    """Builder for MoveMotorRelative. Auto-generated — do not edit."""

    def __init__(self):
        super().__init__()
        self._motor = _UNSET
        self._delta = _UNSET
        self._velocity = 1000
        self._timeout = None

    def motor(self, value: IMotor):
        self._motor = value
        return self

    def delta(self, value: int):
        self._delta = value
        return self

    def velocity(self, value: int):
        self._velocity = value
        return self

    def timeout(self, value: Optional[float]):
        self._timeout = value
        return self

    def _build(self):
        kwargs = {}
        if self._motor is not _UNSET:
            kwargs["motor"] = self._motor
        if self._delta is not _UNSET:
            kwargs["delta"] = self._delta
        kwargs["velocity"] = self._velocity
        kwargs["timeout"] = self._timeout
        return MoveMotorRelative(**kwargs)


@dsl(tags=["motor", "actuator"])
def move_motor_relative(
    motor: IMotor = _UNSET,
    delta: int = _UNSET,
    velocity: int = 1000,
    timeout: Optional[float] = None,
):
    """
    Move a motor by a relative encoder delta and wait for completion.

    Commands the motor's firmware position controller to move by the given
    number of encoder ticks relative to its current position. The step
    blocks until the firmware reports the move is complete, or until the
    optional timeout expires.

    Args:
        motor: The motor to control, obtained from the robot hardware map (e.g. ``robot.motor(2)``).
        delta: Number of encoder ticks to move. Positive values move forward; negative values move in reverse.
        velocity: Movement speed in firmware ticks/s. Must be positive. Defaults to 1000.
        timeout: Maximum seconds to wait for the move to finish. ``None`` (the default) means wait indefinitely. If the timeout fires a warning is logged and the step returns without stopping the motor.

    Returns:
        A MoveMotorRelativeBuilder (chainable via ``.motor()``, ``.delta()``, ``.velocity()``, ``.timeout()``, ``.on_anomaly()``, ``.skip_timing()``).

    Example::

        from raccoon.step.motor import motor_move_relative, motor_brake

        # Rotate the arm motor 200 ticks forward, then brake
        sequence(
            motor_move_relative(robot.motor(2), delta=200, velocity=600),
            motor_brake(robot.motor(2)),
        )
    """
    b = MoveMotorRelativeBuilder()
    if motor is not _UNSET:
        b._motor = motor
    if delta is not _UNSET:
        b._delta = delta
    b._velocity = velocity
    b._timeout = timeout
    return b


class MotorOffBuilder(StepBuilder):
    """Builder for MotorOff. Auto-generated — do not edit."""

    def __init__(self):
        super().__init__()
        self._motor = _UNSET

    def motor(self, value: IMotor):
        self._motor = value
        return self

    def _build(self):
        kwargs = {}
        if self._motor is not _UNSET:
            kwargs["motor"] = self._motor
        return MotorOff(**kwargs)


@dsl(tags=["motor", "actuator"])
def motor_off(motor: IMotor = _UNSET):
    """
    Turn a motor off, allowing it to coast freely.

    Removes all power from the motor so it spins down under friction
    alone. The motor shaft is not held in place and can be back-driven.
    Use this when you want the mechanism to move freely (e.g. letting an
    arm fall under gravity).

    Args:
        motor: The motor to turn off, obtained from the robot hardware map (e.g. ``robot.motor(0)``).

    Returns:
        A MotorOffBuilder (chainable via ``.motor()``, ``.on_anomaly()``, ``.skip_timing()``).

    Example::

        from raccoon.step.motor import motor_power, motor_off

        # Run a motor briefly, then let it coast to a stop
        sequence(
            motor_power(robot.motor(3), 80),
            wait(1.5),
            motor_off(robot.motor(3)),
        )
    """
    b = MotorOffBuilder()
    if motor is not _UNSET:
        b._motor = motor
    return b


class MotorPassiveBrakeBuilder(StepBuilder):
    """Builder for MotorPassiveBrake. Auto-generated — do not edit."""

    def __init__(self):
        super().__init__()
        self._motor = _UNSET

    def motor(self, value: IMotor):
        self._motor = value
        return self

    def _build(self):
        kwargs = {}
        if self._motor is not _UNSET:
            kwargs["motor"] = self._motor
        return MotorPassiveBrake(**kwargs)


@dsl(tags=["motor", "actuator"])
def motor_passive_brake(motor: IMotor = _UNSET):
    """
    Passively brake a motor by commanding zero power.

    Sets the motor power to zero, which causes the H-bridge to short the
    motor leads. This provides electrical braking that decelerates the
    motor faster than coasting (``motor_off``), but does not actively hold
    position once the motor stops.

    Args:
        motor: The motor to brake, obtained from the robot hardware map (e.g. ``robot.motor(0)``).

    Returns:
        A MotorPassiveBrakeBuilder (chainable via ``.motor()``, ``.on_anomaly()``, ``.skip_timing()``).

    Example::

        from raccoon.step.motor import motor_velocity, motor_passive_brake

        # Drive forward then passively brake
        sequence(
            motor_velocity(robot.motor(0), 600),
            wait(2.0),
            motor_passive_brake(robot.motor(0)),
        )
    """
    b = MotorPassiveBrakeBuilder()
    if motor is not _UNSET:
        b._motor = motor
    return b


class MotorBrakeBuilder(StepBuilder):
    """Builder for MotorBrake. Auto-generated — do not edit."""

    def __init__(self):
        super().__init__()
        self._motor = _UNSET

    def motor(self, value: IMotor):
        self._motor = value
        return self

    def _build(self):
        kwargs = {}
        if self._motor is not _UNSET:
            kwargs["motor"] = self._motor
        return MotorBrake(**kwargs)


@dsl(tags=["motor", "actuator"])
def motor_brake(motor: IMotor = _UNSET):
    """
    Actively brake a motor and hold its current position.

    Engages the firmware's active brake (stop latch), which commands the
    motor controller to resist any external force and maintain the
    current shaft position. This is the strongest stop mode and is
    appropriate when the mechanism must not move (e.g. holding an arm
    up against gravity).

    Args:
        motor: The motor to brake, obtained from the robot hardware map (e.g. ``robot.motor(2)``).

    Returns:
        A MotorBrakeBuilder (chainable via ``.motor()``, ``.on_anomaly()``, ``.skip_timing()``).

    Example::

        from raccoon.step.motor import motor_move_to, motor_brake

        # Move the arm to a target, then lock it in place
        sequence(
            motor_move_to(robot.motor(2), position=400),
            motor_brake(robot.motor(2)),
        )
    """
    b = MotorBrakeBuilder()
    if motor is not _UNSET:
        b._motor = motor
    return b


__all__ = [
    "SetMotorPowerBuilder",
    "set_motor_power",
    "SetMotorVelocityBuilder",
    "set_motor_velocity",
    "SetMotorDpsBuilder",
    "set_motor_dps",
    "MoveMotorToBuilder",
    "move_motor_to",
    "MoveMotorRelativeBuilder",
    "move_motor_relative",
    "MotorOffBuilder",
    "motor_off",
    "MotorPassiveBrakeBuilder",
    "motor_passive_brake",
    "MotorBrakeBuilder",
    "motor_brake",
]
