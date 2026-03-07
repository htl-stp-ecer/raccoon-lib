"""
Line following using IR sensors.

This module provides steps for following lines using one or two IR sensors
with PID-based steering control, built on top of LinearMotion for proper
profiled distance control and odometry integration.
"""
from dataclasses import dataclass
from enum import Enum
from typing import TYPE_CHECKING

from libstp.foundation import PidConfig, PidController
from libstp.motion import LinearMotion, LinearMotionConfig, LinearAxis
from libstp.sensor_ir import IRSensor

from .. import SimulationStep, SimulationStepDelta, dsl
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


@dsl(tags=["motion", "line-follow"])
def follow_line(
    left_sensor: IRSensor,
    right_sensor: IRSensor,
    distance_cm: float,
    speed: float = 0.5,
    kp: float = 0.75,
    ki: float = 0.0,
    kd: float = 0.5,
) -> LineFollow:
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
        A ``LineFollow`` step configured for distance-based line following.

    Example::

        from libstp.step.motion import follow_line

        # Follow a line for 80 cm at half speed
        step = follow_line(
            left_sensor=robot.left_ir,
            right_sensor=robot.right_ir,
            distance_cm=80.0,
            speed=0.5,
        )

        # Tighter tracking with higher kp
        step = follow_line(
            left_sensor=robot.left_ir,
            right_sensor=robot.right_ir,
            distance_cm=120.0,
            speed=0.3,
            kp=1.2,
            kd=0.8,
        )
    """
    config = LineFollowConfig(
        left_sensor=left_sensor,
        right_sensor=right_sensor,
        speed_scale=speed,
        distance_cm=distance_cm,
        kp=kp, ki=ki, kd=kd,
    )
    return LineFollow(config)


@dsl(tags=["motion", "line-follow"])
def follow_line_until_both_black(
    left_sensor: IRSensor,
    right_sensor: IRSensor,
    speed: float = 0.5,
    kp: float = 0.75,
    ki: float = 0.0,
    kd: float = 0.5,
    both_black_threshold: float = 0.7,
) -> LineFollow:
    """Follow a line until both sensors detect black, indicating an intersection.

    Drives forward with PID-based steering (same as ``follow_line``) but instead
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
        A ``LineFollow`` step that stops when an intersection is detected.

    Example::

        from libstp.step.motion import follow_line_until_both_black

        # Follow a line until hitting a cross-line
        step = follow_line_until_both_black(
            left_sensor=robot.left_ir,
            right_sensor=robot.right_ir,
            speed=0.4,
        )

        # More sensitive intersection detection
        step = follow_line_until_both_black(
            left_sensor=robot.left_ir,
            right_sensor=robot.right_ir,
            speed=0.3,
            both_black_threshold=0.6,
        )
    """
    config = LineFollowConfig(
        left_sensor=left_sensor,
        right_sensor=right_sensor,
        speed_scale=speed,
        distance_cm=None,
        kp=kp, ki=ki, kd=kd,
        both_black_threshold=both_black_threshold,
    )
    return LineFollow(config)


@dsl(tags=["motion", "line-follow"])
def follow_line_single(
    sensor: IRSensor,
    distance_cm: float,
    speed: float = 0.5,
    side: LineSide = LineSide.LEFT,
    kp: float = 1.0,
    ki: float = 0.0,
    kd: float = 0.3,
) -> SingleSensorLineFollow:
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
        A ``SingleSensorLineFollow`` step.

    Example::

        from libstp.step.motion import follow_line_single, LineSide

        # Follow the left edge of a line for 60 cm
        step = follow_line_single(
            sensor=robot.front_ir,
            distance_cm=60.0,
            speed=0.4,
            side=LineSide.LEFT,
        )

        # Follow the right edge with custom PID gains
        step = follow_line_single(
            sensor=robot.front_ir,
            distance_cm=100.0,
            speed=0.5,
            side=LineSide.RIGHT,
            kp=1.5,
            kd=0.5,
        )
    """
    config = SingleLineFollowConfig(
        sensor=sensor,
        speed_scale=speed,
        side=side,
        distance_cm=distance_cm,
        kp=kp, ki=ki, kd=kd,
    )
    return SingleSensorLineFollow(config)


@dsl(tags=["motion", "line-follow"])
def follow_line_single_until_black(
    sensor: IRSensor,
    stop_sensor: IRSensor,
    speed: float = 0.5,
    side: LineSide = LineSide.LEFT,
    stop_threshold: float = 0.7,
    kp: float = 1.0,
    ki: float = 0.0,
    kd: float = 0.3,
) -> SingleSensorLineFollowUntilBlack:
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
        A ``SingleSensorLineFollowUntilBlack`` step.

    Example::

        from libstp.step.motion import follow_line_single_until_black, LineSide

        # Follow left edge until the right sensor hits a cross-line
        step = follow_line_single_until_black(
            sensor=robot.left_ir,
            stop_sensor=robot.right_ir,
            speed=0.4,
            side=LineSide.LEFT,
        )

        # Lower stop threshold for earlier detection
        step = follow_line_single_until_black(
            sensor=robot.front_ir,
            stop_sensor=robot.side_ir,
            speed=0.3,
            stop_threshold=0.5,
            kp=1.2,
        )
    """
    config = SingleLineFollowUntilBlackConfig(
        sensor=sensor,
        stop_sensor=stop_sensor,
        speed_scale=speed,
        side=side,
        stop_threshold=stop_threshold,
        kp=kp, ki=ki, kd=kd,
    )
    return SingleSensorLineFollowUntilBlack(config)
