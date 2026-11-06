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
    MoveMotorToDegrees,
    MoveMotorRelativeDegrees,
    MotorOff,
    MotorPassiveBrake,
    MotorBrake,
    _DEFAULT_POSITION_TOLERANCE,
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
        self._tolerance = _DEFAULT_POSITION_TOLERANCE

    def motor(self, value: IMotor):
        self._motor = value
        return self

    def position(self, value: int):
        self._position = value
        return self

    def velocity(self, value: int):
        self._velocity = value
        return self

    def timeout(self, value: float | None):
        self._timeout = value
        return self

    def tolerance(self, value: int):
        self._tolerance = value
        return self

    def _build(self):
        kwargs = {}
        if self._motor is not _UNSET:
            kwargs["motor"] = self._motor
        if self._position is not _UNSET:
            kwargs["position"] = self._position
        kwargs["velocity"] = self._velocity
        kwargs["timeout"] = self._timeout
        kwargs["tolerance"] = self._tolerance
        return MoveMotorTo(**kwargs)


@dsl(tags=["motor", "actuator"])
def move_motor_to(
    motor: IMotor = _UNSET,
    position: int = _UNSET,
    velocity: int = 1000,
    timeout: float | None = None,
    tolerance: int = _DEFAULT_POSITION_TOLERANCE,
):
    """
    Move a motor to an absolute encoder position and wait for completion.

    Commands the motor's firmware position controller to drive to the given
    absolute encoder tick count. The step blocks (yielding to the async
    loop) until the measured position is within ``tolerance`` ticks of the
    target, or until the optional timeout expires.

    The motor keeps holding the target position after the step finishes. It
    stays in position mode until another motor step (e.g. ``motor_off``,
    ``motor_brake`` or a velocity/speed step) switches it.

    Args:
        motor: The motor to control, obtained from the robot hardware map (e.g. ``robot.motor(2)``).
        position: Target position in absolute encoder ticks.
        velocity: Movement speed in firmware ticks/s. Must be positive. Defaults to 1000.
        timeout: Maximum seconds to wait for the move to finish. ``None`` (the default) means wait indefinitely. If the timeout fires a warning is logged and the step returns without stopping the motor.
        tolerance: Allowed distance to the target in ticks for the move to count as reached. Must be >= 0. Defaults to 3.

    Returns:
        A MoveMotorToBuilder (chainable via ``.motor()``, ``.position()``, ``.velocity()``, ``.timeout()``, ``.tolerance()``, ``.on_anomaly()``, ``.skip_timing()``).

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
    b._tolerance = tolerance
    return b


class MoveMotorRelativeBuilder(StepBuilder):
    """Builder for MoveMotorRelative. Auto-generated — do not edit."""

    def __init__(self):
        super().__init__()
        self._motor = _UNSET
        self._delta = _UNSET
        self._velocity = 1000
        self._timeout = None
        self._tolerance = _DEFAULT_POSITION_TOLERANCE

    def motor(self, value: IMotor):
        self._motor = value
        return self

    def delta(self, value: int):
        self._delta = value
        return self

    def velocity(self, value: int):
        self._velocity = value
        return self

    def timeout(self, value: float | None):
        self._timeout = value
        return self

    def tolerance(self, value: int):
        self._tolerance = value
        return self

    def _build(self):
        kwargs = {}
        if self._motor is not _UNSET:
            kwargs["motor"] = self._motor
        if self._delta is not _UNSET:
            kwargs["delta"] = self._delta
        kwargs["velocity"] = self._velocity
        kwargs["timeout"] = self._timeout
        kwargs["tolerance"] = self._tolerance
        return MoveMotorRelative(**kwargs)


@dsl(tags=["motor", "actuator"])
def move_motor_relative(
    motor: IMotor = _UNSET,
    delta: int = _UNSET,
    velocity: int = 1000,
    timeout: float | None = None,
    tolerance: int = _DEFAULT_POSITION_TOLERANCE,
):
    """
    Move a motor by a relative encoder delta and wait for completion.

    Commands the motor's firmware position controller to move by the given
    number of encoder ticks relative to its current position (resolved to an
    absolute target when the step starts). The step blocks until the measured
    position is within ``tolerance`` ticks of that target, or until the
    optional timeout expires.

    The motor keeps holding the target position after the step finishes. It
    stays in position mode until another motor step switches it.

    Args:
        motor: The motor to control, obtained from the robot hardware map (e.g. ``robot.motor(2)``).
        delta: Number of encoder ticks to move. Positive values move forward; negative values move in reverse.
        velocity: Movement speed in firmware ticks/s. Must be positive. Defaults to 1000.
        timeout: Maximum seconds to wait for the move to finish. ``None`` (the default) means wait indefinitely. If the timeout fires a warning is logged and the step returns without stopping the motor.
        tolerance: Allowed distance to the target in ticks for the move to count as reached. Must be >= 0. Defaults to 3.

    Returns:
        A MoveMotorRelativeBuilder (chainable via ``.motor()``, ``.delta()``, ``.velocity()``, ``.timeout()``, ``.tolerance()``, ``.on_anomaly()``, ``.skip_timing()``).

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
    b._tolerance = tolerance
    return b


class MoveMotorToDegreesBuilder(StepBuilder):
    """Builder for MoveMotorToDegrees. Auto-generated — do not edit."""

    def __init__(self):
        super().__init__()
        self._motor = _UNSET
        self._degrees = _UNSET
        self._velocity_dps = 360.0
        self._timeout = None
        self._tolerance_degrees = None

    def motor(self, value: IMotor):
        self._motor = value
        return self

    def degrees(self, value: float):
        self._degrees = value
        return self

    def velocity_dps(self, value: float):
        self._velocity_dps = value
        return self

    def timeout(self, value: float | None):
        self._timeout = value
        return self

    def tolerance_degrees(self, value: float | None):
        self._tolerance_degrees = value
        return self

    def _build(self):
        kwargs = {}
        if self._motor is not _UNSET:
            kwargs["motor"] = self._motor
        if self._degrees is not _UNSET:
            kwargs["degrees"] = self._degrees
        kwargs["velocity_dps"] = self._velocity_dps
        kwargs["timeout"] = self._timeout
        kwargs["tolerance_degrees"] = self._tolerance_degrees
        return MoveMotorToDegrees(**kwargs)


@dsl(tags=["motor", "actuator"])
def move_motor_to_degrees(
    motor: IMotor = _UNSET,
    degrees: float = _UNSET,
    velocity_dps: float = 360.0,
    timeout: float | None = None,
    tolerance_degrees: float | None = None,
):
    """
    Move a motor to an absolute angle in degrees and wait until it arrives.

    Like ``move_motor_to`` but the target is given in degrees instead of
    encoder ticks. Degrees are converted with the motor's ``ticks_to_rad``
    calibration; 0 degrees is the motor's current position-counter zero (see
    ``reset_position_counter``). The step blocks until the measured position
    is within ``tolerance_degrees`` of the target (default: 3 ticks), or the
    optional timeout expires.

    The motor keeps holding the target after the step finishes. It stays in
    position mode until another motor step (e.g. ``motor_off`` or
    ``motor_brake``) switches it.

    Args:
        motor: The motor to control (e.g. ``robot.motor(2)``). Needs a valid ``ticks_to_rad`` calibration.
        degrees: Target angle in degrees (signed).
        velocity_dps: Movement speed in degrees per second. Must be positive. Defaults to 360.
        timeout: Maximum seconds to wait. ``None`` waits indefinitely. On timeout a warning is logged and the motor is not stopped.
        tolerance_degrees: Allowed distance to the target in degrees. ``None`` (default) uses 3 ticks.

    Returns:
        A MoveMotorToDegreesBuilder (chainable via ``.motor()``, ``.degrees()``, ``.velocity_dps()``, ``.timeout()``, ``.tolerance_degrees()``, ``.on_anomaly()``, ``.skip_timing()``).

    Example::

        from raccoon.step.motor import move_motor_to_degrees

        # Swing the arm motor to 90 degrees at 180 deg/s
        move_motor_to_degrees(robot.motor(2), degrees=90, velocity_dps=180)
    """
    b = MoveMotorToDegreesBuilder()
    if motor is not _UNSET:
        b._motor = motor
    if degrees is not _UNSET:
        b._degrees = degrees
    b._velocity_dps = velocity_dps
    b._timeout = timeout
    b._tolerance_degrees = tolerance_degrees
    return b


class MoveMotorRelativeDegreesBuilder(StepBuilder):
    """Builder for MoveMotorRelativeDegrees. Auto-generated — do not edit."""

    def __init__(self):
        super().__init__()
        self._motor = _UNSET
        self._delta_degrees = _UNSET
        self._velocity_dps = 360.0
        self._timeout = None
        self._tolerance_degrees = None

    def motor(self, value: IMotor):
        self._motor = value
        return self

    def delta_degrees(self, value: float):
        self._delta_degrees = value
        return self

    def velocity_dps(self, value: float):
        self._velocity_dps = value
        return self

    def timeout(self, value: float | None):
        self._timeout = value
        return self

    def tolerance_degrees(self, value: float | None):
        self._tolerance_degrees = value
        return self

    def _build(self):
        kwargs = {}
        if self._motor is not _UNSET:
            kwargs["motor"] = self._motor
        if self._delta_degrees is not _UNSET:
            kwargs["delta_degrees"] = self._delta_degrees
        kwargs["velocity_dps"] = self._velocity_dps
        kwargs["timeout"] = self._timeout
        kwargs["tolerance_degrees"] = self._tolerance_degrees
        return MoveMotorRelativeDegrees(**kwargs)


@dsl(tags=["motor", "actuator"])
def move_motor_relative_degrees(
    motor: IMotor = _UNSET,
    delta_degrees: float = _UNSET,
    velocity_dps: float = 360.0,
    timeout: float | None = None,
    tolerance_degrees: float | None = None,
):
    """
    Move a motor by a relative angle in degrees and wait until it arrives.

    Like ``move_motor_relative`` but the delta is given in degrees. The target
    is resolved to an absolute position (current position + delta) when the
    step starts. Degrees are converted with the motor's ``ticks_to_rad``
    calibration. The step blocks until the position is within
    ``tolerance_degrees`` of the target (default: 3 ticks), or the optional
    timeout expires.

    The motor keeps holding the target after the step finishes. It stays in
    position mode until another motor step switches it.

    Args:
        motor: The motor to control (e.g. ``robot.motor(2)``). Needs a valid ``ticks_to_rad`` calibration.
        delta_degrees: Angle to move in degrees. Positive moves forward, negative in reverse.
        velocity_dps: Movement speed in degrees per second. Must be positive. Defaults to 360.
        timeout: Maximum seconds to wait. ``None`` waits indefinitely. On timeout a warning is logged and the motor is not stopped.
        tolerance_degrees: Allowed distance to the target in degrees. ``None`` (default) uses 3 ticks.

    Returns:
        A MoveMotorRelativeDegreesBuilder (chainable via ``.motor()``, ``.delta_degrees()``, ``.velocity_dps()``, ``.timeout()``, ``.tolerance_degrees()``, ``.on_anomaly()``, ``.skip_timing()``).

    Example::

        from raccoon.step.motor import move_motor_relative_degrees

        # Turn the arm motor a quarter turn forward
        move_motor_relative_degrees(robot.motor(2), delta_degrees=90)
    """
    b = MoveMotorRelativeDegreesBuilder()
    if motor is not _UNSET:
        b._motor = motor
    if delta_degrees is not _UNSET:
        b._delta_degrees = delta_degrees
    b._velocity_dps = velocity_dps
    b._timeout = timeout
    b._tolerance_degrees = tolerance_degrees
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
    "MoveMotorToDegreesBuilder",
    "move_motor_to_degrees",
    "MoveMotorRelativeDegreesBuilder",
    "move_motor_relative_degrees",
    "MotorOffBuilder",
    "motor_off",
    "MotorPassiveBrakeBuilder",
    "motor_passive_brake",
    "MotorBrakeBuilder",
    "motor_brake",
]
