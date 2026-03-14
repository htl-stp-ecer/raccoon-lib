"""Auto-generated step builders and DSL functions — DO NOT EDIT.

Source: line_follow.py
"""

from __future__ import annotations

_UNSET = object()

from libstp.step.step_builder import StepBuilder
from libstp.step.condition import StopCondition
from libstp.step.annotation import dsl
from .line_follow import FollowLine, FollowLineSingle, DirectionalFollowLine, StrafeFollowLine, StrafeFollowLineSingle, DirectionalFollowLineSingle, LineSide


class FollowLineBuilder(StepBuilder):
    """Builder for FollowLine. Auto-generated — do not edit."""

    def __init__(self):
        super().__init__()
        self._left_sensor = _UNSET
        self._right_sensor = _UNSET
        self._distance_cm = None
        self._speed = 0.5
        self._kp = 0.75
        self._ki = 0.0
        self._kd = 0.5
        self._until = None

    def left_sensor(self, value: IRSensor):
        self._left_sensor = value
        return self

    def right_sensor(self, value: IRSensor):
        self._right_sensor = value
        return self

    def distance_cm(self, value: float | None):
        self._distance_cm = value
        return self

    def speed(self, value: float):
        self._speed = value
        return self

    def kp(self, value: float):
        self._kp = value
        return self

    def ki(self, value: float):
        self._ki = value
        return self

    def kd(self, value: float):
        self._kd = value
        return self

    def until(self, value: StopCondition | None):
        self._until = value
        return self

    def _build(self):
        kwargs = {}
        if self._left_sensor is not _UNSET:
            kwargs['left_sensor'] = self._left_sensor
        if self._right_sensor is not _UNSET:
            kwargs['right_sensor'] = self._right_sensor
        kwargs['distance_cm'] = self._distance_cm
        kwargs['speed'] = self._speed
        kwargs['kp'] = self._kp
        kwargs['ki'] = self._ki
        kwargs['kd'] = self._kd
        kwargs['until'] = self._until
        return FollowLine(**kwargs)


@dsl(tags=['motion', 'line-follow'])
def follow_line(left_sensor: IRSensor = _UNSET, right_sensor: IRSensor = _UNSET, distance_cm: float | None = None, speed: float = 0.5, kp: float = 0.75, ki: float = 0.0, kd: float = 0.5, until: StopCondition | None = None):
    """
    Follow a line using two IR sensors for steering.

    Drives forward while a PID controller steers the robot to keep it centered
    on a line. The error signal is the difference between the left and right
    sensors' ``probabilityOfBlack()`` readings. A positive error (left sees
    more black) steers the robot back toward center. The underlying
    ``LinearMotion`` handles profiled velocity control and odometry-based
    distance tracking, while the PID output overrides the heading command
    as an angular velocity (omega).

    Supports distance-based termination, composable ``StopCondition`` via
    ``.until()``, or both (whichever triggers first). At least one of
    ``distance_cm`` or ``until`` must be provided.

    Both sensors must be calibrated (white/black thresholds set) before use.

    Args:
        left_sensor: Left IR sensor instance, positioned to the left of the line.
        right_sensor: Right IR sensor instance, positioned to the right of the line.
        distance_cm: Distance to follow in centimeters. The step finishes when this distance has been traveled according to odometry. Optional if ``until`` is provided.
        speed: Fraction of max velocity (0.0--1.0). Lower speeds give the PID more time to correct but are slower overall. Default 0.5.
        kp: Proportional gain for steering PID. Higher values produce sharper corrections. Default 0.75.
        ki: Integral gain for steering PID. Typically left at 0.0 unless there is a persistent drift. Default 0.0.
        kd: Derivative gain for steering PID. Damps oscillation around the line. Default 0.5.
        until: Composable stop condition (e.g., ``on_black(left) & on_black(right)``). Can also be chained via the ``.until()`` builder method.

    Returns:
        A FollowLineBuilder (chainable via ``.left_sensor()``, ``.right_sensor()``, ``.distance_cm()``, ``.speed()``, ``.kp()``, ``.ki()``, ``.kd()``, ``.until()``, ``.on_anomaly()``, ``.skip_timing()``).

    Example::

        from libstp.step.motion import FollowLine
        from libstp.step.condition import on_black

        # Follow a line for 80 cm at half speed
        follow_line(left_sensor=left, right_sensor=right, distance_cm=80, speed=0.5)

        # Follow until both sensors see black (intersection)
        follow_line(left, right, speed=0.5).until(on_black(left) & on_black(right))

        # Distance + early termination
        follow_line(left, right, distance_cm=100, speed=0.5).until(on_black(third))
    """
    b = FollowLineBuilder()
    if left_sensor is not _UNSET:
        b._left_sensor = left_sensor
    if right_sensor is not _UNSET:
        b._right_sensor = right_sensor
    b._distance_cm = distance_cm
    b._speed = speed
    b._kp = kp
    b._ki = ki
    b._kd = kd
    b._until = until
    return b


class FollowLineSingleBuilder(StepBuilder):
    """Builder for FollowLineSingle. Auto-generated — do not edit."""

    def __init__(self):
        super().__init__()
        self._sensor = _UNSET
        self._distance_cm = None
        self._speed = 0.5
        self._side = LineSide.LEFT
        self._kp = 1.0
        self._ki = 0.0
        self._kd = 0.3
        self._until = None

    def sensor(self, value: IRSensor):
        self._sensor = value
        return self

    def distance_cm(self, value: float | None):
        self._distance_cm = value
        return self

    def speed(self, value: float):
        self._speed = value
        return self

    def side(self, value: LineSide):
        self._side = value
        return self

    def kp(self, value: float):
        self._kp = value
        return self

    def ki(self, value: float):
        self._ki = value
        return self

    def kd(self, value: float):
        self._kd = value
        return self

    def until(self, value: StopCondition | None):
        self._until = value
        return self

    def _build(self):
        kwargs = {}
        if self._sensor is not _UNSET:
            kwargs['sensor'] = self._sensor
        kwargs['distance_cm'] = self._distance_cm
        kwargs['speed'] = self._speed
        kwargs['side'] = self._side
        kwargs['kp'] = self._kp
        kwargs['ki'] = self._ki
        kwargs['kd'] = self._kd
        kwargs['until'] = self._until
        return FollowLineSingle(**kwargs)


@dsl(tags=['motion', 'line-follow'])
def follow_line_single(sensor: IRSensor = _UNSET, distance_cm: float | None = None, speed: float = 0.5, side: LineSide = LineSide.LEFT, kp: float = 1.0, ki: float = 0.0, kd: float = 0.3, until: StopCondition | None = None):
    """
    Follow a line edge using a single IR sensor.

    The sensor tracks the boundary between the line and the background, where
    ``probabilityOfBlack()`` is approximately 0.5. The PID controller drives
    the error ``(reading - 0.5)`` toward zero, keeping the sensor positioned
    right on the edge. The ``side`` parameter controls which edge: ``LEFT``
    means the sensor is to the left of the line (steers right when it sees
    black), and ``RIGHT`` is the opposite.

    This variant is useful when only one sensor is available, or when the line
    is too narrow for two sensors. The underlying ``LinearMotion`` handles
    profiled velocity and odometry-based distance tracking.

    Supports distance-based termination, composable ``StopCondition`` via
    ``.until()``, or both (whichever triggers first). At least one of
    ``distance_cm`` or ``until`` must be provided.

    The sensor must be calibrated (white/black thresholds set) before use.

    Args:
        sensor: The IR sensor instance used for edge tracking.
        distance_cm: Distance to follow in centimeters. The step finishes when this distance has been traveled. Optional if ``until`` is provided.
        speed: Fraction of max velocity (0.0--1.0). Default 0.5.
        side: Which edge of the line to track. ``LineSide.LEFT`` (default) or ``LineSide.RIGHT``.
        kp: Proportional gain for steering PID. Default 1.0.
        ki: Integral gain for steering PID. Default 0.0.
        kd: Derivative gain for steering PID. Default 0.3.
        until: Composable stop condition (e.g., ``on_black(stop_sensor)``). Can also be chained via the ``.until()`` builder method.

    Returns:
        A FollowLineSingleBuilder (chainable via ``.sensor()``, ``.distance_cm()``, ``.speed()``, ``.side()``, ``.kp()``, ``.ki()``, ``.kd()``, ``.until()``, ``.on_anomaly()``, ``.skip_timing()``).

    Example::

        from libstp.step.motion import FollowLineSingle, LineSide
        from libstp.step.condition import on_black

        # Follow left edge for 60 cm
        follow_line_single(sensor=front_ir, distance_cm=60, speed=0.4)

        # Follow until stop sensor sees black
        follow_line_single(front_ir, speed=0.4, side=LineSide.LEFT).until(on_black(stop))

        # Distance + early termination with timeout
        follow_line_single(front_ir, distance_cm=100).until(on_black(stop) | after_seconds(10))
    """
    b = FollowLineSingleBuilder()
    if sensor is not _UNSET:
        b._sensor = sensor
    b._distance_cm = distance_cm
    b._speed = speed
    b._side = side
    b._kp = kp
    b._ki = ki
    b._kd = kd
    b._until = until
    return b


class DirectionalFollowLineBuilder(StepBuilder):
    """Builder for DirectionalFollowLine. Auto-generated — do not edit."""

    def __init__(self):
        super().__init__()
        self._left_sensor = _UNSET
        self._right_sensor = _UNSET
        self._distance_cm = None
        self._heading_speed = 0.0
        self._strafe_speed = 0.0
        self._kp = 0.75
        self._ki = 0.0
        self._kd = 0.5
        self._until = None

    def left_sensor(self, value: IRSensor):
        self._left_sensor = value
        return self

    def right_sensor(self, value: IRSensor):
        self._right_sensor = value
        return self

    def distance_cm(self, value: float | None):
        self._distance_cm = value
        return self

    def heading_speed(self, value: float):
        self._heading_speed = value
        return self

    def strafe_speed(self, value: float):
        self._strafe_speed = value
        return self

    def kp(self, value: float):
        self._kp = value
        return self

    def ki(self, value: float):
        self._ki = value
        return self

    def kd(self, value: float):
        self._kd = value
        return self

    def until(self, value: StopCondition | None):
        self._until = value
        return self

    def _build(self):
        kwargs = {}
        if self._left_sensor is not _UNSET:
            kwargs['left_sensor'] = self._left_sensor
        if self._right_sensor is not _UNSET:
            kwargs['right_sensor'] = self._right_sensor
        kwargs['distance_cm'] = self._distance_cm
        kwargs['heading_speed'] = self._heading_speed
        kwargs['strafe_speed'] = self._strafe_speed
        kwargs['kp'] = self._kp
        kwargs['ki'] = self._ki
        kwargs['kd'] = self._kd
        kwargs['until'] = self._until
        return DirectionalFollowLine(**kwargs)


@dsl(tags=['motion', 'line-follow'])
def directional_follow_line(left_sensor: IRSensor = _UNSET, right_sensor: IRSensor = _UNSET, distance_cm: float | None = None, heading_speed: float = 0.0, strafe_speed: float = 0.0, kp: float = 0.75, ki: float = 0.0, kd: float = 0.5, until: StopCondition | None = None):
    """
    Follow a line with independent heading and strafe speeds.

    Drive along a line using any combination of forward and lateral velocity
    while a PID controller steers the robot via angular velocity.  The error
    signal is the difference between the left and right sensors'
    ``probabilityOfBlack()`` readings.  Distance is tracked via odometry as
    the euclidean distance from the start position.

    Unlike ``FollowLine`` which only drives forward, this step accepts both
    ``heading_speed`` (forward/backward) and ``strafe_speed`` (left/right)
    as independent fractions of max velocity, enabling line following while
    strafing or driving diagonally.

    Supports distance-based termination, composable ``StopCondition`` via
    ``.until()``, or both (whichever triggers first). At least one of
    ``distance_cm`` or ``until`` must be provided.

    Both sensors must be calibrated (white/black thresholds set) before use.
    Requires a mecanum or omni-wheel drivetrain if ``strafe_speed`` is
    nonzero.

    Args:
        left_sensor: Left IR sensor instance, positioned to the left of the line.
        right_sensor: Right IR sensor instance, positioned to the right of the line.
        distance_cm: Distance to follow in centimeters.  The step finishes when this euclidean distance has been traveled. Optional if ``until`` is provided.
        heading_speed: Forward/backward speed as a fraction of max velocity (-1.0 to 1.0).  Positive = forward, negative = backward. Default 0.0.
        strafe_speed: Lateral speed as a fraction of max velocity (-1.0 to 1.0).  Positive = right, negative = left.  Default 0.0.
        kp: Proportional gain for steering PID.  Default 0.75.
        ki: Integral gain for steering PID.  Default 0.0.
        kd: Derivative gain for steering PID.  Default 0.5.
        until: Composable stop condition. Can also be chained via the ``.until()`` builder method.

    Returns:
        A DirectionalFollowLineBuilder (chainable via ``.left_sensor()``, ``.right_sensor()``, ``.distance_cm()``, ``.heading_speed()``, ``.strafe_speed()``, ``.kp()``, ``.ki()``, ``.kd()``, ``.until()``, ``.on_anomaly()``, ``.skip_timing()``).

    Example::

        from libstp.step.motion import DirectionalFollowLine
        from libstp.step.condition import on_black

        # Strafe right while following a line for 50 cm
        directional_follow_line(left, right, distance_cm=50, strafe_speed=0.5)

        # Follow until both sensors see black
        directional_follow_line(left, right, strafe_speed=0.4).until(
            on_black(left) & on_black(right)
        )
    """
    b = DirectionalFollowLineBuilder()
    if left_sensor is not _UNSET:
        b._left_sensor = left_sensor
    if right_sensor is not _UNSET:
        b._right_sensor = right_sensor
    b._distance_cm = distance_cm
    b._heading_speed = heading_speed
    b._strafe_speed = strafe_speed
    b._kp = kp
    b._ki = ki
    b._kd = kd
    b._until = until
    return b


class StrafeFollowLineBuilder(StepBuilder):
    """Builder for StrafeFollowLine. Auto-generated — do not edit."""

    def __init__(self):
        super().__init__()
        self._left_sensor = _UNSET
        self._right_sensor = _UNSET
        self._distance_cm = None
        self._speed = 0.5
        self._kp = 0.75
        self._ki = 0.0
        self._kd = 0.5
        self._until = None

    def left_sensor(self, value: IRSensor):
        self._left_sensor = value
        return self

    def right_sensor(self, value: IRSensor):
        self._right_sensor = value
        return self

    def distance_cm(self, value: float | None):
        self._distance_cm = value
        return self

    def speed(self, value: float):
        self._speed = value
        return self

    def kp(self, value: float):
        self._kp = value
        return self

    def ki(self, value: float):
        self._ki = value
        return self

    def kd(self, value: float):
        self._kd = value
        return self

    def until(self, value: StopCondition | None):
        self._until = value
        return self

    def _build(self):
        kwargs = {}
        if self._left_sensor is not _UNSET:
            kwargs['left_sensor'] = self._left_sensor
        if self._right_sensor is not _UNSET:
            kwargs['right_sensor'] = self._right_sensor
        kwargs['distance_cm'] = self._distance_cm
        kwargs['speed'] = self._speed
        kwargs['kp'] = self._kp
        kwargs['ki'] = self._ki
        kwargs['kd'] = self._kd
        kwargs['until'] = self._until
        return StrafeFollowLine(**kwargs)


@dsl(tags=['motion', 'line-follow'])
def strafe_follow_line(left_sensor: IRSensor = _UNSET, right_sensor: IRSensor = _UNSET, distance_cm: float | None = None, speed: float = 0.5, kp: float = 0.75, ki: float = 0.0, kd: float = 0.5, until: StopCondition | None = None):
    """
    Follow a line forward, correcting position by strafing left/right.

    The robot drives forward at the given speed while a PID controller
    corrects lateral position using two sensors.  Unlike ``FollowLine``
    which steers by rotating, this step keeps the robot's heading constant
    and corrects by strafing, which is useful when the robot must maintain
    a fixed orientation (e.g. to keep a side-mounted mechanism aligned).

    Supports distance-based termination, composable ``StopCondition`` via
    ``.until()``, or both (whichever triggers first). At least one of
    ``distance_cm`` or ``until`` must be provided.

    Both sensors must be calibrated.  Requires a mecanum or omni-wheel
    drivetrain.

    Args:
        left_sensor: Left IR sensor instance.
        right_sensor: Right IR sensor instance.
        distance_cm: Distance to follow in centimeters. Optional if ``until`` is provided.
        speed: Forward speed as fraction of max velocity (0.0 to 1.0). Default 0.5.  Use negative values to drive backward.
        kp: Proportional gain for lateral PID.  Default 0.75.
        ki: Integral gain for lateral PID.  Default 0.0.
        kd: Derivative gain for lateral PID.  Default 0.5.
        until: Composable stop condition. Can also be chained via the ``.until()`` builder method.

    Returns:
        A StrafeFollowLineBuilder (chainable via ``.left_sensor()``, ``.right_sensor()``, ``.distance_cm()``, ``.speed()``, ``.kp()``, ``.ki()``, ``.kd()``, ``.until()``, ``.on_anomaly()``, ``.skip_timing()``).

    Example::

        from libstp.step.motion import StrafeFollowLine
        from libstp.step.condition import on_black

        # Follow a line for 40 cm, correcting via strafe
        strafe_follow_line(left, right, distance_cm=40, speed=0.4)

        # Follow until both sensors see black
        strafe_follow_line(left, right, speed=0.4).until(
            on_black(left) & on_black(right)
        )
    """
    b = StrafeFollowLineBuilder()
    if left_sensor is not _UNSET:
        b._left_sensor = left_sensor
    if right_sensor is not _UNSET:
        b._right_sensor = right_sensor
    b._distance_cm = distance_cm
    b._speed = speed
    b._kp = kp
    b._ki = ki
    b._kd = kd
    b._until = until
    return b


class StrafeFollowLineSingleBuilder(StepBuilder):
    """Builder for StrafeFollowLineSingle. Auto-generated — do not edit."""

    def __init__(self):
        super().__init__()
        self._sensor = _UNSET
        self._distance_cm = None
        self._speed = 0.5
        self._side = LineSide.LEFT
        self._kp = 1.0
        self._ki = 0.0
        self._kd = 0.3
        self._until = None

    def sensor(self, value: IRSensor):
        self._sensor = value
        return self

    def distance_cm(self, value: float | None):
        self._distance_cm = value
        return self

    def speed(self, value: float):
        self._speed = value
        return self

    def side(self, value: LineSide):
        self._side = value
        return self

    def kp(self, value: float):
        self._kp = value
        return self

    def ki(self, value: float):
        self._ki = value
        return self

    def kd(self, value: float):
        self._kd = value
        return self

    def until(self, value: StopCondition | None):
        self._until = value
        return self

    def _build(self):
        kwargs = {}
        if self._sensor is not _UNSET:
            kwargs['sensor'] = self._sensor
        kwargs['distance_cm'] = self._distance_cm
        kwargs['speed'] = self._speed
        kwargs['side'] = self._side
        kwargs['kp'] = self._kp
        kwargs['ki'] = self._ki
        kwargs['kd'] = self._kd
        kwargs['until'] = self._until
        return StrafeFollowLineSingle(**kwargs)


@dsl(tags=['motion', 'line-follow'])
def strafe_follow_line_single(sensor: IRSensor = _UNSET, distance_cm: float | None = None, speed: float = 0.5, side: LineSide = LineSide.LEFT, kp: float = 1.0, ki: float = 0.0, kd: float = 0.3, until: StopCondition | None = None):
    """
    Follow a line edge forward, correcting position by strafing.

    The robot drives forward at the given speed while a PID controller
    corrects lateral position using a single sensor tracking the line edge.
    Unlike ``FollowLineSingle`` which steers by rotating, this step keeps
    the robot's heading constant and corrects by strafing.

    Supports distance-based termination, composable ``StopCondition`` via
    ``.until()``, or both (whichever triggers first). At least one of
    ``distance_cm`` or ``until`` must be provided.

    The sensor must be calibrated.  Requires a mecanum or omni-wheel
    drivetrain.

    Args:
        sensor: IR sensor for edge tracking.
        distance_cm: Distance to follow in centimeters. Optional if ``until`` is provided.
        speed: Forward speed as fraction of max velocity (0.0 to 1.0). Default 0.5.  Use negative values to drive backward.
        side: Which edge of the line to track.  Default ``LineSide.LEFT``.
        kp: Proportional gain for lateral PID.  Default 1.0.
        ki: Integral gain for lateral PID.  Default 0.0.
        kd: Derivative gain for lateral PID.  Default 0.3.
        until: Composable stop condition. Can also be chained via the ``.until()`` builder method.

    Returns:
        A StrafeFollowLineSingleBuilder (chainable via ``.sensor()``, ``.distance_cm()``, ``.speed()``, ``.side()``, ``.kp()``, ``.ki()``, ``.kd()``, ``.until()``, ``.on_anomaly()``, ``.skip_timing()``).

    Example::

        from libstp.step.motion import StrafeFollowLineSingle, LineSide
        from libstp.step.condition import on_black

        # Follow a line edge for 40 cm, correcting via strafe
        strafe_follow_line_single(front_ir, distance_cm=40, speed=0.4)

        # Follow until stop sensor sees black
        strafe_follow_line_single(front_ir, speed=0.4).until(on_black(stop))
    """
    b = StrafeFollowLineSingleBuilder()
    if sensor is not _UNSET:
        b._sensor = sensor
    b._distance_cm = distance_cm
    b._speed = speed
    b._side = side
    b._kp = kp
    b._ki = ki
    b._kd = kd
    b._until = until
    return b


class DirectionalFollowLineSingleBuilder(StepBuilder):
    """Builder for DirectionalFollowLineSingle. Auto-generated — do not edit."""

    def __init__(self):
        super().__init__()
        self._sensor = _UNSET
        self._distance_cm = None
        self._heading_speed = 0.0
        self._strafe_speed = 0.0
        self._side = LineSide.LEFT
        self._kp = 1.0
        self._ki = 0.0
        self._kd = 0.3
        self._until = None

    def sensor(self, value: IRSensor):
        self._sensor = value
        return self

    def distance_cm(self, value: float | None):
        self._distance_cm = value
        return self

    def heading_speed(self, value: float):
        self._heading_speed = value
        return self

    def strafe_speed(self, value: float):
        self._strafe_speed = value
        return self

    def side(self, value: LineSide):
        self._side = value
        return self

    def kp(self, value: float):
        self._kp = value
        return self

    def ki(self, value: float):
        self._ki = value
        return self

    def kd(self, value: float):
        self._kd = value
        return self

    def until(self, value: StopCondition | None):
        self._until = value
        return self

    def _build(self):
        kwargs = {}
        if self._sensor is not _UNSET:
            kwargs['sensor'] = self._sensor
        kwargs['distance_cm'] = self._distance_cm
        kwargs['heading_speed'] = self._heading_speed
        kwargs['strafe_speed'] = self._strafe_speed
        kwargs['side'] = self._side
        kwargs['kp'] = self._kp
        kwargs['ki'] = self._ki
        kwargs['kd'] = self._kd
        kwargs['until'] = self._until
        return DirectionalFollowLineSingle(**kwargs)


@dsl(tags=['motion', 'line-follow'])
def directional_follow_line_single(sensor: IRSensor = _UNSET, distance_cm: float | None = None, heading_speed: float = 0.0, strafe_speed: float = 0.0, side: LineSide = LineSide.LEFT, kp: float = 1.0, ki: float = 0.0, kd: float = 0.3, until: StopCondition | None = None):
    """
    Follow a line edge with a single sensor and independent heading/strafe speeds.

    The sensor tracks the boundary between the line and the background, where
    ``probabilityOfBlack()`` is approximately 0.5.  The ``side`` parameter
    selects which edge to track.  The PID output controls angular velocity
    while heading and strafe velocities are set independently.

    Supports distance-based termination, composable ``StopCondition`` via
    ``.until()``, or both (whichever triggers first). At least one of
    ``distance_cm`` or ``until`` must be provided.

    The sensor must be calibrated (white/black thresholds set) before use.
    Requires a mecanum or omni-wheel drivetrain if ``strafe_speed`` is
    nonzero.

    Args:
        sensor: IR sensor for edge tracking.
        distance_cm: Distance to follow in centimeters. Optional if ``until`` is provided.
        heading_speed: Forward/backward speed fraction (-1.0 to 1.0). Default 0.0.
        strafe_speed: Lateral speed fraction (-1.0 to 1.0).  Default 0.0.
        side: Which edge of the line to track.  Default ``LineSide.LEFT``.
        kp: Proportional gain for steering PID.  Default 1.0.
        ki: Integral gain for steering PID.  Default 0.0.
        kd: Derivative gain for steering PID.  Default 0.3.
        until: Composable stop condition. Can also be chained via the ``.until()`` builder method.

    Returns:
        A DirectionalFollowLineSingleBuilder (chainable via ``.sensor()``, ``.distance_cm()``, ``.heading_speed()``, ``.strafe_speed()``, ``.side()``, ``.kp()``, ``.ki()``, ``.kd()``, ``.until()``, ``.on_anomaly()``, ``.skip_timing()``).

    Example::

        from libstp.step.motion import DirectionalFollowLineSingle, LineSide
        from libstp.step.condition import on_black

        # Strafe right while tracking the left edge for 50 cm
        directional_follow_line_single(front_ir, distance_cm=50, strafe_speed=0.4)

        # Follow until stop sensor sees black
        directional_follow_line_single(front_ir, strafe_speed=0.4).until(on_black(stop))
    """
    b = DirectionalFollowLineSingleBuilder()
    if sensor is not _UNSET:
        b._sensor = sensor
    b._distance_cm = distance_cm
    b._heading_speed = heading_speed
    b._strafe_speed = strafe_speed
    b._side = side
    b._kp = kp
    b._ki = ki
    b._kd = kd
    b._until = until
    return b


__all__ = ['FollowLineBuilder', 'follow_line', 'FollowLineSingleBuilder', 'follow_line_single', 'DirectionalFollowLineBuilder', 'directional_follow_line', 'StrafeFollowLineBuilder', 'strafe_follow_line', 'StrafeFollowLineSingleBuilder', 'strafe_follow_line_single', 'DirectionalFollowLineSingleBuilder', 'directional_follow_line_single']
