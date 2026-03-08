"""
Line following using IR sensors.

This module provides steps for following lines using one or two IR sensors
with PID-based steering control.

Two families of steps are available:

1. **Profiled line follow** (``FollowLine``, ``FollowLineSingle``, etc.) —
   built on ``LinearMotion`` for trapezoidal-profiled distance control along
   a single axis.

2. **Directional line follow** (``DirectionalFollowLine``,
   ``StrafeFollowLine``, etc.) — uses direct ``ChassisVelocity`` control
   with independent heading and strafe speed inputs, allowing line following
   while strafing, driving diagonally, or any combination.
"""
import math
from dataclasses import dataclass
from enum import Enum
from typing import TYPE_CHECKING

from libstp.foundation import ChassisVelocity, PidConfig, PidController
from libstp.motion import LinearMotion, LinearMotionConfig, LinearAxis
from libstp.sensor_ir import IRSensor

from .. import SimulationStep, SimulationStepDelta, dsl
from ..annotation import dsl_step
from .motion_step import MotionStep

if TYPE_CHECKING:
    from libstp.robot.api import GenericRobot


@dataclass
class LineFollowConfig:
    """Configuration for LineFollow step with two sensors."""
    left_sensor: IRSensor
    right_sensor: IRSensor
    speed_scale: float  # 0-1 fraction of max velocity
    distance_cm: float | None = None  # None = run until both black
    kp: float = 0.75
    ki: float = 0.0
    kd: float = 0.5
    both_black_threshold: float = 0.7  # threshold for "both black" stop condition


class LineSide(Enum):
    """Which edge of the line to track with a single sensor."""
    LEFT = "left"
    RIGHT = "right"


@dataclass
class SingleLineFollowConfig:
    """Configuration for single-sensor line following.

    The sensor tracks the edge of a line using PID control.
    ``side`` selects which edge: LEFT means the sensor approaches
    from the left (steers right when it sees black), RIGHT is the
    opposite.
    """
    sensor: IRSensor
    speed_scale: float  # 0-1 fraction of max velocity
    distance_cm: float  # distance to follow
    side: LineSide = LineSide.LEFT
    kp: float = 1.0
    ki: float = 0.0
    kd: float = 0.3


@dataclass
class SingleLineFollowUntilBlackConfig:
    """Configuration for single-sensor line following that stops when a second sensor sees black.

    The ``sensor`` tracks the line edge using PID control, while
    ``stop_sensor`` is monitored each cycle. When the stop sensor's
    probabilityOfBlack exceeds ``stop_threshold``, the step finishes.
    """
    sensor: IRSensor
    stop_sensor: IRSensor
    speed_scale: float  # 0-1 fraction of max velocity
    side: LineSide = LineSide.LEFT
    stop_threshold: float = 0.7
    kp: float = 1.0
    ki: float = 0.0
    kd: float = 0.3


@dsl(hidden=True)
class LineFollow(MotionStep):
    """Follow a line using two IR sensors with PID steering.

    Computes a steering error as the difference between the left and right
    sensors' ``probabilityOfBlack()`` readings and feeds it through a PID
    controller. The PID output is applied as an angular velocity (omega)
    override on the underlying ``LinearMotion``, which handles profiled
    distance control and odometry integration.

    Supports two modes: fixed-distance (stop after traveling a set distance)
    and until-both-black (stop when both sensors see black simultaneously,
    indicating an intersection).
    """

    def __init__(self, config: LineFollowConfig):
        super().__init__()
        self.config = config
        self._motion: LinearMotion | None = None
        self._pid: PidController | None = None

    def _generate_signature(self) -> str:
        mode = f"{self.config.distance_cm:.1f}cm" if self.config.distance_cm else "until_both_black"
        return (
            f"LineFollow(mode={mode}, speed={self.config.speed_scale:.2f})"
        )

    def to_simulation_step(self) -> SimulationStep:
        base = super().to_simulation_step()
        distance_m = (self.config.distance_cm / 100.0) if self.config.distance_cm else 0.3
        base.delta = SimulationStepDelta(
            forward=distance_m,
            strafe=0.0,
            angular=0.0,
        )
        return base

    def on_start(self, robot: "GenericRobot") -> None:
        cfg = self.config

        motion_config = LinearMotionConfig()
        motion_config.axis = LinearAxis.Forward
        if cfg.distance_cm is not None:
            motion_config.distance_m = cfg.distance_cm / 100.0
        else:
            # Large distance so LinearMotion never finishes by distance;
            # the both_black check in on_update stops early.
            motion_config.distance_m = 100.0
        motion_config.speed_scale = cfg.speed_scale

        self._motion = LinearMotion(
            robot.drive, robot.odometry, robot.motion_pid_config, motion_config,
        )
        self._motion.start()

        self._pid = PidController(PidConfig(
            kp=cfg.kp, ki=cfg.ki, kd=cfg.kd,
            integral_max=1.0, output_min=-1.0, output_max=1.0,
        ))

        mode = f"{cfg.distance_cm:.1f}cm" if cfg.distance_cm else "until_both_black"
        self.debug(
            f"on_start: mode={mode}, speed_scale={cfg.speed_scale:.2f}, "
            f"PID({cfg.kp}, {cfg.ki}, {cfg.kd})"
        )

    def on_update(self, robot: "GenericRobot", dt: float) -> bool:
        cfg = self.config

        # Check both_black stop condition (only for until_both_black mode)
        if cfg.distance_cm is None:
            left_black = cfg.left_sensor.probabilityOfBlack()
            right_black = cfg.right_sensor.probabilityOfBlack()
            if (left_black >= cfg.both_black_threshold and
                    right_black >= cfg.both_black_threshold):
                self.debug(
                    f"stop: both black (L={left_black:.2f}, R={right_black:.2f}, "
                    f"thresh={cfg.both_black_threshold:.2f})"
                )
                return True

        # Sensor error: left - right
        left_conf = cfg.left_sensor.probabilityOfBlack()
        right_conf = cfg.right_sensor.probabilityOfBlack()
        error = left_conf - right_conf

        # PID steering -> omega override on LinearMotion
        wz = self._pid.update(error, dt)
        self._motion.set_omega_override(wz)

        self.debug(
            f"L={left_conf:.2f} R={right_conf:.2f} err={error:.2f} wz={wz:.3f} dt={dt:.4f}"
        )

        # LinearMotion handles odometry, profiled velocity, drive commands
        self._motion.update(dt)
        return self._motion.is_finished()


@dsl(hidden=True)
class SingleSensorLineFollow(MotionStep):
    """Follow a line edge using a single IR sensor with PID edge-tracking.

    Targets ``probabilityOfBlack() = 0.5`` (the line edge) as the setpoint.
    The ``side`` configuration flips the error sign to select left vs. right
    edge tracking. The PID output overrides the angular velocity on the
    underlying ``LinearMotion``, which handles profiled distance control
    and odometry integration. Terminates when the configured distance has
    been traveled.
    """

    def __init__(self, config: SingleLineFollowConfig):
        super().__init__()
        self.config = config
        self._motion: LinearMotion | None = None
        self._pid: PidController | None = None

    def _generate_signature(self) -> str:
        return (
            f"SingleSensorLineFollow(distance={self.config.distance_cm:.1f}cm, "
            f"side={self.config.side.value}, speed={self.config.speed_scale:.2f})"
        )

    def to_simulation_step(self) -> SimulationStep:
        base = super().to_simulation_step()
        base.delta = SimulationStepDelta(
            forward=self.config.distance_cm / 100.0,
            strafe=0.0,
            angular=0.0,
        )
        return base

    def on_start(self, robot: "GenericRobot") -> None:
        cfg = self.config

        motion_config = LinearMotionConfig()
        motion_config.axis = LinearAxis.Forward
        motion_config.distance_m = cfg.distance_cm / 100.0
        motion_config.speed_scale = cfg.speed_scale

        self._motion = LinearMotion(
            robot.drive, robot.odometry, robot.motion_pid_config, motion_config,
        )
        self._motion.start()

        self._pid = PidController(PidConfig(
            kp=cfg.kp, ki=cfg.ki, kd=cfg.kd,
            integral_max=1.0, output_min=-1.0, output_max=1.0,
        ))

        self.debug(
            f"on_start: distance={cfg.distance_cm:.1f}cm, side={cfg.side.value}, "
            f"speed_scale={cfg.speed_scale:.2f}, PID({cfg.kp}, {cfg.ki}, {cfg.kd})"
        )

    def on_update(self, robot: "GenericRobot", dt: float) -> bool:
        cfg = self.config

        # Edge-tracking error: 0.5 = edge of line
        reading = cfg.sensor.probabilityOfBlack()
        error = reading - 0.5
        if cfg.side == LineSide.RIGHT:
            error = -error

        # PID steering -> omega override on LinearMotion
        wz = self._pid.update(error, dt)
        self._motion.set_omega_override(wz)

        self.debug(
            f"raw={cfg.sensor.read():.0f} black={reading:.2f} err={error:.2f} wz={wz:.3f} dt={dt:.4f}"
        )

        self._motion.update(dt)
        return self._motion.is_finished()


@dsl(hidden=True)
class SingleSensorLineFollowUntilBlack(MotionStep):
    """Follow a line edge using one sensor, stopping when a second sensor sees black.

    Combines single-sensor edge tracking (targeting ``probabilityOfBlack() =
    0.5``) with an event-based stop condition. The tracking sensor feeds a
    PID controller whose output overrides angular velocity on the underlying
    ``LinearMotion``. Each cycle, the stop sensor is checked; when its
    ``probabilityOfBlack()`` exceeds the configured threshold the step
    finishes immediately.
    """

    def __init__(self, config: SingleLineFollowUntilBlackConfig):
        super().__init__()
        self.config = config
        self._motion: LinearMotion | None = None
        self._pid: PidController | None = None

    def _generate_signature(self) -> str:
        return (
            f"SingleSensorLineFollowUntilBlack("
            f"side={self.config.side.value}, speed={self.config.speed_scale:.2f}, "
            f"stop_thresh={self.config.stop_threshold:.2f})"
        )

    def to_simulation_step(self) -> SimulationStep:
        base = super().to_simulation_step()
        base.delta = SimulationStepDelta(
            forward=0.3,
            strafe=0.0,
            angular=0.0,
        )
        return base

    def on_start(self, robot: "GenericRobot") -> None:
        cfg = self.config
        self._odometry = robot.odometry
        self._start_heading = robot.odometry.get_heading()
        self._elapsed = 0.0

        motion_config = LinearMotionConfig()
        motion_config.axis = LinearAxis.Forward
        # Large distance so LinearMotion never finishes by distance;
        # the stop_sensor check in on_update stops early.
        motion_config.distance_m = 100.0
        motion_config.speed_scale = cfg.speed_scale

        self._motion = LinearMotion(
            robot.drive, robot.odometry, robot.motion_pid_config, motion_config,
        )
        self._motion.start()

        self._pid = PidController(PidConfig(
            kp=cfg.kp, ki=cfg.ki, kd=cfg.kd,
            integral_max=1.0, output_min=-1.0, output_max=1.0,
        ))

        self.debug(
            f"on_start: side={cfg.side.value}, speed_scale={cfg.speed_scale:.2f}, "
            f"stop_thresh={cfg.stop_threshold:.2f}, PID({cfg.kp}, {cfg.ki}, {cfg.kd}), "
            f"track_port={cfg.sensor.port} (white={cfg.sensor.whiteThreshold:.0f}, black={cfg.sensor.blackThreshold:.0f}), "
            f"stop_port={cfg.stop_sensor.port} (white={cfg.stop_sensor.whiteThreshold:.0f}, black={cfg.stop_sensor.blackThreshold:.0f}), "
            f"heading={self._start_heading:.2f}rad"
        )

    def on_update(self, robot: "GenericRobot", dt: float) -> bool:
        cfg = self.config
        self._elapsed += dt

        # Check stop sensor
        stop_reading = cfg.stop_sensor.probabilityOfBlack()
        if stop_reading >= cfg.stop_threshold:
            heading = self._odometry.get_heading()
            self.debug(
                f"stop: stop_raw={cfg.stop_sensor.read():.0f} stop_black={stop_reading:.2f} >= {cfg.stop_threshold:.2f}"
                f" (port={cfg.stop_sensor.port}, white={cfg.stop_sensor.whiteThreshold:.0f}, black={cfg.stop_sensor.blackThreshold:.0f})"
                f" elapsed={self._elapsed:.2f}s heading={heading:.2f}rad (delta={heading - self._start_heading:.2f})"
            )
            return True

        # Edge-tracking error: 0.5 = edge of line
        reading = cfg.sensor.probabilityOfBlack()
        error = reading - 0.5
        if cfg.side == LineSide.RIGHT:
            error = -error

        # PID steering -> omega override on LinearMotion
        wz = self._pid.update(error, dt)
        self._motion.set_omega_override(wz)

        heading = self._odometry.get_heading()
        self.debug(
            f"raw={cfg.sensor.read():.0f} black={reading:.2f} stop_raw={cfg.stop_sensor.read():.0f} stop={stop_reading:.2f} "
            f"err={error:.2f} wz={wz:.3f} I={self._pid.integral:.3f} "
            f"hdg={heading:.2f} dhdg={heading - self._start_heading:.2f} t={self._elapsed:.2f}s"
        )

        self._motion.update(dt)
        return False


# ---------------------------------------------------------------------------
# Profiled line follow — @dsl_step public classes
# ---------------------------------------------------------------------------


@dsl_step(tags=["motion", "line-follow"])
class FollowLine(LineFollow):
    """Follow a line for a specified distance using two IR sensors for steering.

    Drives forward while a PID controller steers the robot to keep it centered
    on a line. The error signal is the difference between the left and right
    sensors' ``probabilityOfBlack()`` readings. A positive error (left sees
    more black) steers the robot back toward center. The underlying
    ``LinearMotion`` handles profiled velocity control and odometry-based
    distance tracking, while the PID output overrides the heading command
    as an angular velocity (omega).

    Both sensors must be calibrated (white/black thresholds set) before use.

    Args:
        left_sensor: Left IR sensor instance, positioned to the left of the
            line.
        right_sensor: Right IR sensor instance, positioned to the right of
            the line.
        distance_cm: Distance to follow in centimeters. The step finishes
            when this distance has been traveled according to odometry.
        speed: Fraction of max velocity (0.0--1.0). Lower speeds give the
            PID more time to correct but are slower overall. Default 0.5.
        kp: Proportional gain for steering PID. Higher values produce
            sharper corrections. Default 0.75.
        ki: Integral gain for steering PID. Typically left at 0.0 unless
            there is a persistent drift. Default 0.0.
        kd: Derivative gain for steering PID. Damps oscillation around the
            line. Default 0.5.

    Returns:
        A ``FollowLine`` step configured for distance-based line following.

    Example::

        from libstp.step.motion import FollowLine

        # Follow a line for 80 cm at half speed
        FollowLine(
            left_sensor=robot.left_ir,
            right_sensor=robot.right_ir,
            distance_cm=80.0,
            speed=0.5,
        )

        # Tighter tracking with higher kp
        FollowLine(
            left_sensor=robot.left_ir,
            right_sensor=robot.right_ir,
            distance_cm=120.0,
            speed=0.3,
            kp=1.2,
            kd=0.8,
        )
    """

    def __init__(
        self,
        left_sensor: IRSensor,
        right_sensor: IRSensor,
        distance_cm: float,
        speed: float = 0.5,
        kp: float = 0.75,
        ki: float = 0.0,
        kd: float = 0.5,
    ) -> None:
        self._left_sensor = left_sensor
        self._right_sensor = right_sensor
        self._distance_cm = distance_cm
        self._speed = speed
        self._kp = kp
        self._ki = ki
        self._kd = kd
        config = LineFollowConfig(
            left_sensor=left_sensor,
            right_sensor=right_sensor,
            speed_scale=speed,
            distance_cm=distance_cm,
            kp=kp, ki=ki, kd=kd,
        )
        super().__init__(config)

    def _generate_signature(self) -> str:
        return (
            f"FollowLine(distance={self._distance_cm:.1f}cm, "
            f"speed={self._speed:.2f})"
        )


@dsl_step(tags=["motion", "line-follow"])
class FollowLineUntilBothBlack(LineFollow):
    """Follow a line until both sensors detect black, indicating an intersection.

    Drives forward with PID-based steering (same as ``FollowLine``) but instead
    of stopping after a fixed distance, the step monitors both sensors each
    cycle. When *both* ``probabilityOfBlack()`` readings exceed
    ``both_black_threshold`` simultaneously, the robot has reached a
    perpendicular line or intersection and the step finishes.

    Internally the distance target is set very large so ``LinearMotion`` never
    finishes on its own -- the both-black condition is the sole termination
    criterion.

    Both sensors must be calibrated (white/black thresholds set) before use.

    Args:
        left_sensor: Left IR sensor instance, positioned to the left of the
            line.
        right_sensor: Right IR sensor instance, positioned to the right of
            the line.
        speed: Fraction of max velocity (0.0--1.0). Default 0.5.
        kp: Proportional gain for steering PID. Default 0.75.
        ki: Integral gain for steering PID. Default 0.0.
        kd: Derivative gain for steering PID. Default 0.5.
        both_black_threshold: The ``probabilityOfBlack()`` value that both
            sensors must exceed to trigger the stop. Default 0.7.

    Returns:
        A ``FollowLineUntilBothBlack`` step that stops when an intersection
        is detected.

    Example::

        from libstp.step.motion import FollowLineUntilBothBlack

        # Follow a line until hitting a cross-line
        FollowLineUntilBothBlack(
            left_sensor=robot.left_ir,
            right_sensor=robot.right_ir,
            speed=0.4,
        )

        # More sensitive intersection detection
        FollowLineUntilBothBlack(
            left_sensor=robot.left_ir,
            right_sensor=robot.right_ir,
            speed=0.3,
            both_black_threshold=0.6,
        )
    """

    def __init__(
        self,
        left_sensor: IRSensor,
        right_sensor: IRSensor,
        speed: float = 0.5,
        kp: float = 0.75,
        ki: float = 0.0,
        kd: float = 0.5,
        both_black_threshold: float = 0.7,
    ) -> None:
        self._left_sensor = left_sensor
        self._right_sensor = right_sensor
        self._speed = speed
        self._kp = kp
        self._ki = ki
        self._kd = kd
        self._both_black_threshold = both_black_threshold
        config = LineFollowConfig(
            left_sensor=left_sensor,
            right_sensor=right_sensor,
            speed_scale=speed,
            distance_cm=None,
            kp=kp, ki=ki, kd=kd,
            both_black_threshold=both_black_threshold,
        )
        super().__init__(config)

    def _generate_signature(self) -> str:
        return (
            f"FollowLineUntilBothBlack(speed={self._speed:.2f}, "
            f"threshold={self._both_black_threshold:.2f})"
        )


@dsl_step(tags=["motion", "line-follow"])
class FollowLineSingle(SingleSensorLineFollow):
    """Follow a line edge using a single IR sensor for a specified distance.

    The sensor tracks the boundary between the line and the background, where
    ``probabilityOfBlack()`` is approximately 0.5. The PID controller drives
    the error ``(reading - 0.5)`` toward zero, keeping the sensor positioned
    right on the edge. The ``side`` parameter controls which edge: ``LEFT``
    means the sensor is to the left of the line (steers right when it sees
    black), and ``RIGHT`` is the opposite.

    This variant is useful when only one sensor is available, or when the line
    is too narrow for two sensors. The underlying ``LinearMotion`` handles
    profiled velocity and odometry-based distance tracking.

    The sensor must be calibrated (white/black thresholds set) before use.

    Args:
        sensor: The IR sensor instance used for edge tracking.
        distance_cm: Distance to follow in centimeters. The step finishes
            when this distance has been traveled.
        speed: Fraction of max velocity (0.0--1.0). Default 0.5.
        side: Which edge of the line to track. ``LineSide.LEFT`` (default)
            or ``LineSide.RIGHT``.
        kp: Proportional gain for steering PID. Default 1.0.
        ki: Integral gain for steering PID. Default 0.0.
        kd: Derivative gain for steering PID. Default 0.3.

    Returns:
        A ``FollowLineSingle`` step.

    Example::

        from libstp.step.motion import FollowLineSingle, LineSide

        # Follow the left edge of a line for 60 cm
        FollowLineSingle(
            sensor=robot.front_ir,
            distance_cm=60.0,
            speed=0.4,
            side=LineSide.LEFT,
        )

        # Follow the right edge with custom PID gains
        FollowLineSingle(
            sensor=robot.front_ir,
            distance_cm=100.0,
            speed=0.5,
            side=LineSide.RIGHT,
            kp=1.5,
            kd=0.5,
        )
    """

    def __init__(
        self,
        sensor: IRSensor,
        distance_cm: float,
        speed: float = 0.5,
        side: LineSide = LineSide.LEFT,
        kp: float = 1.0,
        ki: float = 0.0,
        kd: float = 0.3,
    ) -> None:
        self._sensor = sensor
        self._distance_cm = distance_cm
        self._speed = speed
        self._side = side
        self._kp = kp
        self._ki = ki
        self._kd = kd
        config = SingleLineFollowConfig(
            sensor=sensor,
            speed_scale=speed,
            side=side,
            distance_cm=distance_cm,
            kp=kp, ki=ki, kd=kd,
        )
        super().__init__(config)

    def _generate_signature(self) -> str:
        return (
            f"FollowLineSingle(distance={self._distance_cm:.1f}cm, "
            f"side={self._side.value}, speed={self._speed:.2f})"
        )


@dsl_step(tags=["motion", "line-follow"])
class FollowLineSingleUntilBlack(SingleSensorLineFollowUntilBlack):
    """Follow a line edge using one sensor, stopping when a second sensor sees black.

    Combines single-sensor edge tracking with an event-based stop condition.
    The ``sensor`` tracks the line edge (targeting ``probabilityOfBlack() ~
    0.5``) using PID control, while the ``stop_sensor`` is polled each cycle.
    When the stop sensor's ``probabilityOfBlack()`` exceeds
    ``stop_threshold``, the step finishes immediately.

    This is useful for following a line until the robot reaches a perpendicular
    marker or a specific position detected by a second sensor (e.g., a side-
    mounted sensor that crosses a branch line).

    Both sensors must be calibrated (white/black thresholds set) before use.

    Args:
        sensor: The IR sensor used for edge-tracking along the line.
        stop_sensor: A second IR sensor monitored for the stop condition.
            The step finishes when this sensor's ``probabilityOfBlack()``
            exceeds ``stop_threshold``.
        speed: Fraction of max velocity (0.0--1.0). Default 0.5.
        side: Which edge of the line to track. ``LineSide.LEFT`` (default)
            or ``LineSide.RIGHT``.
        stop_threshold: The ``probabilityOfBlack()`` value the stop sensor
            must exceed to trigger the stop. Default 0.7.
        kp: Proportional gain for steering PID. Default 1.0.
        ki: Integral gain for steering PID. Default 0.0.
        kd: Derivative gain for steering PID. Default 0.3.

    Returns:
        A ``FollowLineSingleUntilBlack`` step.

    Example::

        from libstp.step.motion import FollowLineSingleUntilBlack, LineSide

        # Follow left edge until the right sensor hits a cross-line
        FollowLineSingleUntilBlack(
            sensor=robot.left_ir,
            stop_sensor=robot.right_ir,
            speed=0.4,
            side=LineSide.LEFT,
        )

        # Lower stop threshold for earlier detection
        FollowLineSingleUntilBlack(
            sensor=robot.front_ir,
            stop_sensor=robot.side_ir,
            speed=0.3,
            stop_threshold=0.5,
            kp=1.2,
        )
    """

    def __init__(
        self,
        sensor: IRSensor,
        stop_sensor: IRSensor,
        speed: float = 0.5,
        side: LineSide = LineSide.LEFT,
        stop_threshold: float = 0.7,
        kp: float = 1.0,
        ki: float = 0.0,
        kd: float = 0.3,
    ) -> None:
        self._sensor = sensor
        self._stop_sensor = stop_sensor
        self._speed = speed
        self._side = side
        self._stop_threshold = stop_threshold
        self._kp = kp
        self._ki = ki
        self._kd = kd
        config = SingleLineFollowUntilBlackConfig(
            sensor=sensor,
            stop_sensor=stop_sensor,
            speed_scale=speed,
            side=side,
            stop_threshold=stop_threshold,
            kp=kp, ki=ki, kd=kd,
        )
        super().__init__(config)

    def _generate_signature(self) -> str:
        return (
            f"FollowLineSingleUntilBlack(side={self._side.value}, "
            f"speed={self._speed:.2f}, stop_thresh={self._stop_threshold:.2f})"
        )


# ---------------------------------------------------------------------------
# Directional line follow — generic heading + strafe with PID steering
# ---------------------------------------------------------------------------


@dataclass
class DirectionalLineFollowConfig:
    """Configuration for directional line following with two sensors.

    Allows independent heading (forward/backward) and strafe (left/right)
    speed components.  The PID controller steers via angular velocity based
    on the difference between left and right sensor readings.
    """
    left_sensor: IRSensor
    right_sensor: IRSensor
    heading_speed: float    # -1..1 fraction of max forward velocity
    strafe_speed: float     # -1..1 fraction of max lateral velocity (positive = right)
    distance_cm: float | None = None  # None = run until both black
    kp: float = 0.75
    ki: float = 0.0
    kd: float = 0.5
    both_black_threshold: float = 0.7


@dsl(hidden=True)
class DirectionalLineFollow(MotionStep):
    """Follow a line with independent heading and strafe velocity components.

    Uses direct ``ChassisVelocity`` control instead of ``LinearMotion``,
    enabling line following while strafing, driving diagonally, or any
    combination.  A PID controller computes angular velocity from the
    difference between the left and right sensors' ``probabilityOfBlack()``
    readings.

    Distance is tracked via odometry as euclidean distance from the start
    position.  Supports two stop modes: fixed distance and until-both-black.
    """

    def __init__(self, config: DirectionalLineFollowConfig):
        super().__init__()
        self.config = config
        self._pid: PidController | None = None
        self._vx: float = 0.0
        self._vy: float = 0.0
        self._target_distance_m: float | None = None

    def _generate_signature(self) -> str:
        mode = f"{self.config.distance_cm:.1f}cm" if self.config.distance_cm else "until_both_black"
        return (
            f"DirectionalLineFollow(mode={mode}, "
            f"heading={self.config.heading_speed:.2f}, "
            f"strafe={self.config.strafe_speed:.2f})"
        )

    def to_simulation_step(self) -> SimulationStep:
        base = super().to_simulation_step()
        distance_m = (self.config.distance_cm / 100.0) if self.config.distance_cm else 0.3
        # Approximate direction from speed components
        speed_mag = math.hypot(self.config.heading_speed, self.config.strafe_speed)
        if speed_mag > 0:
            fwd_frac = self.config.heading_speed / speed_mag
            str_frac = self.config.strafe_speed / speed_mag
        else:
            fwd_frac, str_frac = 1.0, 0.0
        base.delta = SimulationStepDelta(
            forward=distance_m * fwd_frac,
            strafe=distance_m * str_frac,
            angular=0.0,
        )
        return base

    def on_start(self, robot: "GenericRobot") -> None:
        cfg = self.config

        # Convert speed fractions to m/s
        pid_cfg = robot.motion_pid_config
        self._vx = cfg.heading_speed * pid_cfg.linear.max_velocity
        self._vy = cfg.strafe_speed * pid_cfg.lateral.max_velocity

        if cfg.distance_cm is not None:
            self._target_distance_m = cfg.distance_cm / 100.0

        robot.odometry.reset()

        self._pid = PidController(PidConfig(
            kp=cfg.kp, ki=cfg.ki, kd=cfg.kd,
            integral_max=1.0, output_min=-1.0, output_max=1.0,
        ))

        mode = f"{cfg.distance_cm:.1f}cm" if cfg.distance_cm else "until_both_black"
        self.debug(
            f"on_start: mode={mode}, vx={self._vx:.3f}m/s, vy={self._vy:.3f}m/s, "
            f"PID({cfg.kp}, {cfg.ki}, {cfg.kd})"
        )

    def on_update(self, robot: "GenericRobot", dt: float) -> bool:
        cfg = self.config

        left_conf = cfg.left_sensor.probabilityOfBlack()
        right_conf = cfg.right_sensor.probabilityOfBlack()

        # Check both_black stop condition (only for until_both_black mode)
        if cfg.distance_cm is None:
            if (left_conf >= cfg.both_black_threshold and
                    right_conf >= cfg.both_black_threshold):
                self.debug(
                    f"stop: both black (L={left_conf:.2f}, R={right_conf:.2f}, "
                    f"thresh={cfg.both_black_threshold:.2f})"
                )
                return True

        # Check distance stop condition
        if self._target_distance_m is not None:
            dist = robot.odometry.get_distance_from_origin()
            if dist.straight_line >= self._target_distance_m:
                self.debug(
                    f"stop: distance reached ({dist.straight_line:.3f}m >= "
                    f"{self._target_distance_m:.3f}m)"
                )
                return True

        # PID steering: sensor error -> omega
        error = left_conf - right_conf
        wz = self._pid.update(error, dt)

        robot.drive.set_velocity(ChassisVelocity(self._vx, self._vy, wz))
        robot.drive.update(dt)

        self.debug(
            f"L={left_conf:.2f} R={right_conf:.2f} err={error:.2f} wz={wz:.3f} dt={dt:.4f}"
        )

        return False


@dataclass
class DirectionalSingleLineFollowConfig:
    """Configuration for directional single-sensor line following.

    The sensor tracks the edge of a line using PID control while the robot
    moves with the given heading and strafe velocity components.
    """
    sensor: IRSensor
    heading_speed: float    # -1..1 fraction of max forward velocity
    strafe_speed: float     # -1..1 fraction of max lateral velocity (positive = right)
    distance_cm: float | None = None  # None requires stop_sensor
    side: LineSide = LineSide.LEFT
    stop_sensor: IRSensor | None = None
    stop_threshold: float = 0.7
    kp: float = 1.0
    ki: float = 0.0
    kd: float = 0.3


@dsl(hidden=True)
class DirectionalSingleLineFollow(MotionStep):
    """Follow a line edge with independent heading and strafe velocity.

    Targets ``probabilityOfBlack() = 0.5`` (the line edge) as the setpoint.
    The ``side`` configuration flips the error sign to select left vs. right
    edge tracking.  The PID output controls angular velocity while heading
    and strafe velocities are set directly via ``ChassisVelocity``.

    Supports distance-based and stop-sensor-based termination.
    """

    def __init__(self, config: DirectionalSingleLineFollowConfig):
        super().__init__()
        self.config = config
        self._pid: PidController | None = None
        self._vx: float = 0.0
        self._vy: float = 0.0
        self._target_distance_m: float | None = None

    def _generate_signature(self) -> str:
        if self.config.distance_cm is not None:
            mode = f"{self.config.distance_cm:.1f}cm"
        elif self.config.stop_sensor is not None:
            mode = "until_stop_sensor"
        else:
            mode = "indefinite"
        return (
            f"DirectionalSingleLineFollow(mode={mode}, "
            f"side={self.config.side.value}, "
            f"heading={self.config.heading_speed:.2f}, "
            f"strafe={self.config.strafe_speed:.2f})"
        )

    def to_simulation_step(self) -> SimulationStep:
        base = super().to_simulation_step()
        distance_m = (self.config.distance_cm / 100.0) if self.config.distance_cm else 0.3
        speed_mag = math.hypot(self.config.heading_speed, self.config.strafe_speed)
        if speed_mag > 0:
            fwd_frac = self.config.heading_speed / speed_mag
            str_frac = self.config.strafe_speed / speed_mag
        else:
            fwd_frac, str_frac = 1.0, 0.0
        base.delta = SimulationStepDelta(
            forward=distance_m * fwd_frac,
            strafe=distance_m * str_frac,
            angular=0.0,
        )
        return base

    def on_start(self, robot: "GenericRobot") -> None:
        cfg = self.config

        pid_cfg = robot.motion_pid_config
        self._vx = cfg.heading_speed * pid_cfg.linear.max_velocity
        self._vy = cfg.strafe_speed * pid_cfg.lateral.max_velocity

        if cfg.distance_cm is not None:
            self._target_distance_m = cfg.distance_cm / 100.0

        robot.odometry.reset()

        self._pid = PidController(PidConfig(
            kp=cfg.kp, ki=cfg.ki, kd=cfg.kd,
            integral_max=1.0, output_min=-1.0, output_max=1.0,
        ))

        self.debug(
            f"on_start: side={cfg.side.value}, vx={self._vx:.3f}m/s, vy={self._vy:.3f}m/s, "
            f"PID({cfg.kp}, {cfg.ki}, {cfg.kd})"
        )

    def on_update(self, robot: "GenericRobot", dt: float) -> bool:
        cfg = self.config

        # Check stop sensor
        if cfg.stop_sensor is not None:
            stop_reading = cfg.stop_sensor.probabilityOfBlack()
            if stop_reading >= cfg.stop_threshold:
                self.debug(
                    f"stop: stop_sensor black={stop_reading:.2f} >= {cfg.stop_threshold:.2f}"
                )
                return True

        # Check distance
        if self._target_distance_m is not None:
            dist = robot.odometry.get_distance_from_origin()
            if dist.straight_line >= self._target_distance_m:
                self.debug(
                    f"stop: distance reached ({dist.straight_line:.3f}m >= "
                    f"{self._target_distance_m:.3f}m)"
                )
                return True

        # Edge-tracking error: 0.5 = edge of line
        reading = cfg.sensor.probabilityOfBlack()
        error = reading - 0.5
        if cfg.side == LineSide.RIGHT:
            error = -error

        wz = self._pid.update(error, dt)

        robot.drive.set_velocity(ChassisVelocity(self._vx, self._vy, wz))
        robot.drive.update(dt)

        self.debug(
            f"black={reading:.2f} err={error:.2f} wz={wz:.3f} dt={dt:.4f}"
        )

        return False


# ---------------------------------------------------------------------------
# Directional line follow — @dsl_step public classes
# ---------------------------------------------------------------------------


@dsl_step(tags=["motion", "line-follow"])
class DirectionalFollowLine(DirectionalLineFollow):
    """Follow a line for a distance with independent heading and strafe speeds.

    Drive along a line using any combination of forward and lateral velocity
    while a PID controller steers the robot via angular velocity.  The error
    signal is the difference between the left and right sensors'
    ``probabilityOfBlack()`` readings.  Distance is tracked via odometry as
    the euclidean distance from the start position.

    Unlike ``FollowLine`` which only drives forward, this step accepts both
    ``heading_speed`` (forward/backward) and ``strafe_speed`` (left/right)
    as independent fractions of max velocity, enabling line following while
    strafing or driving diagonally.

    Both sensors must be calibrated (white/black thresholds set) before use.
    Requires a mecanum or omni-wheel drivetrain if ``strafe_speed`` is
    nonzero.

    Args:
        left_sensor: Left IR sensor instance, positioned to the left of the
            line.
        right_sensor: Right IR sensor instance, positioned to the right of
            the line.
        distance_cm: Distance to follow in centimeters.  The step finishes
            when this euclidean distance has been traveled.
        heading_speed: Forward/backward speed as a fraction of max velocity
            (-1.0 to 1.0).  Positive = forward, negative = backward.
            Default 0.0.
        strafe_speed: Lateral speed as a fraction of max velocity (-1.0 to
            1.0).  Positive = right, negative = left.  Default 0.0.
        kp: Proportional gain for steering PID.  Default 0.75.
        ki: Integral gain for steering PID.  Default 0.0.
        kd: Derivative gain for steering PID.  Default 0.5.

    Returns:
        A ``DirectionalFollowLine`` step.

    Example::

        from libstp.step.motion import DirectionalFollowLine

        # Strafe right while following a line for 50 cm
        DirectionalFollowLine(
            left_sensor=robot.left_ir,
            right_sensor=robot.right_ir,
            distance_cm=50.0,
            strafe_speed=0.5,
        )

        # Drive diagonally forward-right along a line
        DirectionalFollowLine(
            left_sensor=robot.left_ir,
            right_sensor=robot.right_ir,
            distance_cm=80.0,
            heading_speed=0.3,
            strafe_speed=0.4,
        )
    """

    def __init__(
        self,
        left_sensor: IRSensor,
        right_sensor: IRSensor,
        distance_cm: float,
        heading_speed: float = 0.0,
        strafe_speed: float = 0.0,
        kp: float = 0.75,
        ki: float = 0.0,
        kd: float = 0.5,
    ) -> None:
        self._left_sensor = left_sensor
        self._right_sensor = right_sensor
        self._distance_cm = distance_cm
        self._heading_speed = heading_speed
        self._strafe_speed = strafe_speed
        self._kp = kp
        self._ki = ki
        self._kd = kd
        super().__init__(DirectionalLineFollowConfig(
            left_sensor=left_sensor,
            right_sensor=right_sensor,
            heading_speed=heading_speed,
            strafe_speed=strafe_speed,
            distance_cm=distance_cm,
            kp=kp, ki=ki, kd=kd,
        ))

    def _generate_signature(self) -> str:
        return (
            f"DirectionalFollowLine(distance={self._distance_cm:.1f}cm, "
            f"heading={self._heading_speed:.2f}, strafe={self._strafe_speed:.2f})"
        )


@dsl_step(tags=["motion", "line-follow"])
class DirectionalFollowLineUntilBothBlack(DirectionalLineFollow):
    """Follow a line with heading and strafe until both sensors detect black.

    Same as ``DirectionalFollowLine`` but instead of stopping after a
    fixed distance, the step monitors both sensors each cycle and finishes
    when both ``probabilityOfBlack()`` readings exceed
    ``both_black_threshold`` simultaneously, indicating an intersection.

    Both sensors must be calibrated (white/black thresholds set) before use.
    Requires a mecanum or omni-wheel drivetrain if ``strafe_speed`` is
    nonzero.

    Args:
        left_sensor: Left IR sensor instance.
        right_sensor: Right IR sensor instance.
        heading_speed: Forward/backward speed fraction (-1.0 to 1.0).
            Default 0.0.
        strafe_speed: Lateral speed fraction (-1.0 to 1.0).  Default 0.0.
        kp: Proportional gain for steering PID.  Default 0.75.
        ki: Integral gain for steering PID.  Default 0.0.
        kd: Derivative gain for steering PID.  Default 0.5.
        both_black_threshold: ``probabilityOfBlack()`` value that both
            sensors must exceed to trigger the stop.  Default 0.7.

    Returns:
        A ``DirectionalFollowLineUntilBothBlack`` step that stops at an
        intersection.

    Example::

        from libstp.step.motion import DirectionalFollowLineUntilBothBlack

        # Strafe right along a line until hitting a cross-line
        DirectionalFollowLineUntilBothBlack(
            left_sensor=robot.left_ir,
            right_sensor=robot.right_ir,
            strafe_speed=0.4,
        )
    """

    def __init__(
        self,
        left_sensor: IRSensor,
        right_sensor: IRSensor,
        heading_speed: float = 0.0,
        strafe_speed: float = 0.0,
        kp: float = 0.75,
        ki: float = 0.0,
        kd: float = 0.5,
        both_black_threshold: float = 0.7,
    ) -> None:
        self._left_sensor = left_sensor
        self._right_sensor = right_sensor
        self._heading_speed = heading_speed
        self._strafe_speed = strafe_speed
        self._kp = kp
        self._ki = ki
        self._kd = kd
        self._both_black_threshold = both_black_threshold
        super().__init__(DirectionalLineFollowConfig(
            left_sensor=left_sensor,
            right_sensor=right_sensor,
            heading_speed=heading_speed,
            strafe_speed=strafe_speed,
            distance_cm=None,
            kp=kp, ki=ki, kd=kd,
            both_black_threshold=both_black_threshold,
        ))

    def _generate_signature(self) -> str:
        return (
            f"DirectionalFollowLineUntilBothBlack("
            f"heading={self._heading_speed:.2f}, strafe={self._strafe_speed:.2f}, "
            f"threshold={self._both_black_threshold:.2f})"
        )


@dsl_step(tags=["motion", "line-follow"])
class StrafeFollowLine(DirectionalLineFollow):
    """Follow a line by strafing right for a specified distance.

    Convenience wrapper around ``DirectionalFollowLine`` for pure lateral
    line following.  The robot strafes right at the given speed while PID
    steering keeps it centered on the line using two sensors.

    Both sensors must be calibrated.  Requires a mecanum or omni-wheel
    drivetrain.

    Args:
        left_sensor: Left IR sensor instance.
        right_sensor: Right IR sensor instance.
        distance_cm: Distance to strafe in centimeters.
        speed: Strafe speed as fraction of max lateral velocity (0.0 to
            1.0).  Default 0.5.  Use negative values to strafe left.
        kp: Proportional gain for steering PID.  Default 0.75.
        ki: Integral gain for steering PID.  Default 0.0.
        kd: Derivative gain for steering PID.  Default 0.5.

    Returns:
        A ``StrafeFollowLine`` step configured for lateral motion.

    Example::

        from libstp.step.motion import StrafeFollowLine

        # Strafe right along a line for 40 cm
        StrafeFollowLine(
            left_sensor=robot.left_ir,
            right_sensor=robot.right_ir,
            distance_cm=40.0,
            speed=0.4,
        )

        # Strafe left along a line for 30 cm
        StrafeFollowLine(
            left_sensor=robot.left_ir,
            right_sensor=robot.right_ir,
            distance_cm=30.0,
            speed=-0.4,
        )
    """

    def __init__(
        self,
        left_sensor: IRSensor,
        right_sensor: IRSensor,
        distance_cm: float,
        speed: float = 0.5,
        kp: float = 0.75,
        ki: float = 0.0,
        kd: float = 0.5,
    ) -> None:
        self._left_sensor = left_sensor
        self._right_sensor = right_sensor
        self._distance_cm = distance_cm
        self._speed = speed
        self._kp = kp
        self._ki = ki
        self._kd = kd
        super().__init__(DirectionalLineFollowConfig(
            left_sensor=left_sensor,
            right_sensor=right_sensor,
            heading_speed=0.0,
            strafe_speed=speed,
            distance_cm=distance_cm,
            kp=kp, ki=ki, kd=kd,
        ))

    def _generate_signature(self) -> str:
        return (
            f"StrafeFollowLine(distance={self._distance_cm:.1f}cm, "
            f"speed={self._speed:.2f})"
        )


@dsl_step(tags=["motion", "line-follow"])
class StrafeFollowLineUntilBothBlack(DirectionalLineFollow):
    """Follow a line by strafing right until both sensors detect black.

    Convenience wrapper around ``DirectionalFollowLineUntilBothBlack``
    for pure lateral line following until an intersection is reached.

    Both sensors must be calibrated.  Requires a mecanum or omni-wheel
    drivetrain.

    Args:
        left_sensor: Left IR sensor instance.
        right_sensor: Right IR sensor instance.
        speed: Strafe speed as fraction of max lateral velocity (0.0 to
            1.0).  Default 0.5.  Use negative values to strafe left.
        kp: Proportional gain for steering PID.  Default 0.75.
        ki: Integral gain for steering PID.  Default 0.0.
        kd: Derivative gain for steering PID.  Default 0.5.
        both_black_threshold: ``probabilityOfBlack()`` value that both
            sensors must exceed to trigger the stop.  Default 0.7.

    Returns:
        A ``StrafeFollowLineUntilBothBlack`` step that stops at an
        intersection.

    Example::

        from libstp.step.motion import StrafeFollowLineUntilBothBlack

        # Strafe right along a line until a cross-line
        StrafeFollowLineUntilBothBlack(
            left_sensor=robot.left_ir,
            right_sensor=robot.right_ir,
            speed=0.4,
        )
    """

    def __init__(
        self,
        left_sensor: IRSensor,
        right_sensor: IRSensor,
        speed: float = 0.5,
        kp: float = 0.75,
        ki: float = 0.0,
        kd: float = 0.5,
        both_black_threshold: float = 0.7,
    ) -> None:
        self._left_sensor = left_sensor
        self._right_sensor = right_sensor
        self._speed = speed
        self._kp = kp
        self._ki = ki
        self._kd = kd
        self._both_black_threshold = both_black_threshold
        super().__init__(DirectionalLineFollowConfig(
            left_sensor=left_sensor,
            right_sensor=right_sensor,
            heading_speed=0.0,
            strafe_speed=speed,
            distance_cm=None,
            kp=kp, ki=ki, kd=kd,
            both_black_threshold=both_black_threshold,
        ))

    def _generate_signature(self) -> str:
        return (
            f"StrafeFollowLineUntilBothBlack(speed={self._speed:.2f}, "
            f"threshold={self._both_black_threshold:.2f})"
        )


@dsl_step(tags=["motion", "line-follow"])
class StrafeFollowLineSingle(DirectionalSingleLineFollow):
    """Follow a line edge by strafing right using a single sensor.

    Convenience wrapper around ``DirectionalFollowLineSingle`` for pure
    lateral single-sensor line following.  The robot strafes at the given
    speed while PID edge-tracking keeps the sensor on the line boundary.

    The sensor must be calibrated.  Requires a mecanum or omni-wheel
    drivetrain.

    Args:
        sensor: IR sensor for edge tracking.
        distance_cm: Distance to strafe in centimeters.
        speed: Strafe speed as fraction of max lateral velocity (0.0 to
            1.0).  Default 0.5.  Use negative values to strafe left.
        side: Which edge of the line to track.  Default ``LineSide.LEFT``.
        kp: Proportional gain for steering PID.  Default 1.0.
        ki: Integral gain for steering PID.  Default 0.0.
        kd: Derivative gain for steering PID.  Default 0.3.

    Returns:
        A ``StrafeFollowLineSingle`` step configured for lateral motion.

    Example::

        from libstp.step.motion import StrafeFollowLineSingle, LineSide

        # Strafe right along a line edge for 40 cm
        StrafeFollowLineSingle(
            sensor=robot.front_ir,
            distance_cm=40.0,
            speed=0.4,
            side=LineSide.LEFT,
        )

        # Strafe left along a line edge for 30 cm
        StrafeFollowLineSingle(
            sensor=robot.front_ir,
            distance_cm=30.0,
            speed=-0.4,
            side=LineSide.RIGHT,
        )
    """

    def __init__(
        self,
        sensor: IRSensor,
        distance_cm: float,
        speed: float = 0.5,
        side: LineSide = LineSide.LEFT,
        kp: float = 1.0,
        ki: float = 0.0,
        kd: float = 0.3,
    ) -> None:
        self._sensor = sensor
        self._distance_cm = distance_cm
        self._speed = speed
        self._side = side
        self._kp = kp
        self._ki = ki
        self._kd = kd
        super().__init__(DirectionalSingleLineFollowConfig(
            sensor=sensor,
            heading_speed=0.0,
            strafe_speed=speed,
            distance_cm=distance_cm,
            side=side,
            kp=kp, ki=ki, kd=kd,
        ))

    def _generate_signature(self) -> str:
        return (
            f"StrafeFollowLineSingle(distance={self._distance_cm:.1f}cm, "
            f"side={self._side.value}, speed={self._speed:.2f})"
        )


@dsl_step(tags=["motion", "line-follow"])
class StrafeFollowLineSingleUntilBlack(DirectionalSingleLineFollow):
    """Follow a line edge by strafing, stopping when a second sensor sees black.

    Convenience wrapper around ``DirectionalFollowLineSingleUntilBlack``
    for pure lateral single-sensor line following with a stop-sensor trigger.

    Both sensors must be calibrated.  Requires a mecanum or omni-wheel
    drivetrain.

    Args:
        sensor: IR sensor for edge tracking.
        stop_sensor: Second IR sensor for the stop condition.
        speed: Strafe speed as fraction of max lateral velocity (0.0 to
            1.0).  Default 0.5.  Use negative values to strafe left.
        side: Which edge of the line to track.  Default ``LineSide.LEFT``.
        stop_threshold: ``probabilityOfBlack()`` the stop sensor must
            exceed.  Default 0.7.
        kp: Proportional gain for steering PID.  Default 1.0.
        ki: Integral gain for steering PID.  Default 0.0.
        kd: Derivative gain for steering PID.  Default 0.3.

    Returns:
        A ``StrafeFollowLineSingleUntilBlack`` step that stops on sensor
        trigger.

    Example::

        from libstp.step.motion import StrafeFollowLineSingleUntilBlack, LineSide

        # Strafe right along a line edge until the stop sensor hits a cross-line
        StrafeFollowLineSingleUntilBlack(
            sensor=robot.left_ir,
            stop_sensor=robot.right_ir,
            speed=0.4,
            side=LineSide.LEFT,
        )
    """

    def __init__(
        self,
        sensor: IRSensor,
        stop_sensor: IRSensor,
        speed: float = 0.5,
        side: LineSide = LineSide.LEFT,
        stop_threshold: float = 0.7,
        kp: float = 1.0,
        ki: float = 0.0,
        kd: float = 0.3,
    ) -> None:
        self._sensor = sensor
        self._stop_sensor = stop_sensor
        self._speed = speed
        self._side = side
        self._stop_threshold = stop_threshold
        self._kp = kp
        self._ki = ki
        self._kd = kd
        super().__init__(DirectionalSingleLineFollowConfig(
            sensor=sensor,
            heading_speed=0.0,
            strafe_speed=speed,
            side=side,
            stop_sensor=stop_sensor,
            stop_threshold=stop_threshold,
            kp=kp, ki=ki, kd=kd,
        ))

    def _generate_signature(self) -> str:
        return (
            f"StrafeFollowLineSingleUntilBlack(side={self._side.value}, "
            f"speed={self._speed:.2f}, stop_thresh={self._stop_threshold:.2f})"
        )


@dsl_step(tags=["motion", "line-follow"])
class DirectionalFollowLineSingle(DirectionalSingleLineFollow):
    """Follow a line edge with a single sensor and independent heading/strafe speeds.

    The sensor tracks the boundary between the line and the background, where
    ``probabilityOfBlack()`` is approximately 0.5.  The ``side`` parameter
    selects which edge to track.  The PID output controls angular velocity
    while heading and strafe velocities are set independently.

    The sensor must be calibrated (white/black thresholds set) before use.
    Requires a mecanum or omni-wheel drivetrain if ``strafe_speed`` is
    nonzero.

    Args:
        sensor: IR sensor for edge tracking.
        distance_cm: Distance to follow in centimeters.
        heading_speed: Forward/backward speed fraction (-1.0 to 1.0).
            Default 0.0.
        strafe_speed: Lateral speed fraction (-1.0 to 1.0).  Default 0.0.
        side: Which edge of the line to track.  Default ``LineSide.LEFT``.
        kp: Proportional gain for steering PID.  Default 1.0.
        ki: Integral gain for steering PID.  Default 0.0.
        kd: Derivative gain for steering PID.  Default 0.3.

    Returns:
        A ``DirectionalFollowLineSingle`` step.

    Example::

        from libstp.step.motion import DirectionalFollowLineSingle, LineSide

        # Strafe right while tracking the left edge of a line
        DirectionalFollowLineSingle(
            sensor=robot.front_ir,
            distance_cm=50.0,
            strafe_speed=0.4,
            side=LineSide.LEFT,
        )
    """

    def __init__(
        self,
        sensor: IRSensor,
        distance_cm: float,
        heading_speed: float = 0.0,
        strafe_speed: float = 0.0,
        side: LineSide = LineSide.LEFT,
        kp: float = 1.0,
        ki: float = 0.0,
        kd: float = 0.3,
    ) -> None:
        self._sensor = sensor
        self._distance_cm = distance_cm
        self._heading_speed = heading_speed
        self._strafe_speed = strafe_speed
        self._side = side
        self._kp = kp
        self._ki = ki
        self._kd = kd
        super().__init__(DirectionalSingleLineFollowConfig(
            sensor=sensor,
            heading_speed=heading_speed,
            strafe_speed=strafe_speed,
            distance_cm=distance_cm,
            side=side,
            kp=kp, ki=ki, kd=kd,
        ))

    def _generate_signature(self) -> str:
        return (
            f"DirectionalFollowLineSingle(distance={self._distance_cm:.1f}cm, "
            f"side={self._side.value}, heading={self._heading_speed:.2f}, "
            f"strafe={self._strafe_speed:.2f})"
        )


@dsl_step(tags=["motion", "line-follow"])
class DirectionalFollowLineSingleUntilBlack(DirectionalSingleLineFollow):
    """Follow a line edge with heading/strafe, stopping when a second sensor sees black.

    Combines single-sensor edge tracking with an event-based stop condition.
    The ``sensor`` tracks the line edge using PID control while the
    ``stop_sensor`` is polled each cycle.

    Both sensors must be calibrated before use.  Requires a mecanum or
    omni-wheel drivetrain if ``strafe_speed`` is nonzero.

    Args:
        sensor: IR sensor for edge tracking.
        stop_sensor: Second IR sensor for the stop condition.
        heading_speed: Forward/backward speed fraction (-1.0 to 1.0).
            Default 0.0.
        strafe_speed: Lateral speed fraction (-1.0 to 1.0).  Default 0.0.
        side: Which edge of the line to track.  Default ``LineSide.LEFT``.
        stop_threshold: ``probabilityOfBlack()`` the stop sensor must
            exceed.  Default 0.7.
        kp: Proportional gain for steering PID.  Default 1.0.
        ki: Integral gain for steering PID.  Default 0.0.
        kd: Derivative gain for steering PID.  Default 0.3.

    Returns:
        A ``DirectionalFollowLineSingleUntilBlack`` step.

    Example::

        from libstp.step.motion import DirectionalFollowLineSingleUntilBlack, LineSide

        # Strafe right along a line edge until the stop sensor hits a cross-line
        DirectionalFollowLineSingleUntilBlack(
            sensor=robot.left_ir,
            stop_sensor=robot.right_ir,
            strafe_speed=0.4,
            side=LineSide.LEFT,
        )
    """

    def __init__(
        self,
        sensor: IRSensor,
        stop_sensor: IRSensor,
        heading_speed: float = 0.0,
        strafe_speed: float = 0.0,
        side: LineSide = LineSide.LEFT,
        stop_threshold: float = 0.7,
        kp: float = 1.0,
        ki: float = 0.0,
        kd: float = 0.3,
    ) -> None:
        self._sensor = sensor
        self._stop_sensor = stop_sensor
        self._heading_speed = heading_speed
        self._strafe_speed = strafe_speed
        self._side = side
        self._stop_threshold = stop_threshold
        self._kp = kp
        self._ki = ki
        self._kd = kd
        super().__init__(DirectionalSingleLineFollowConfig(
            sensor=sensor,
            heading_speed=heading_speed,
            strafe_speed=strafe_speed,
            side=side,
            stop_sensor=stop_sensor,
            stop_threshold=stop_threshold,
            kp=kp, ki=ki, kd=kd,
        ))

    def _generate_signature(self) -> str:
        return (
            f"DirectionalFollowLineSingleUntilBlack(side={self._side.value}, "
            f"heading={self._heading_speed:.2f}, strafe={self._strafe_speed:.2f}, "
            f"stop_thresh={self._stop_threshold:.2f})"
        )
