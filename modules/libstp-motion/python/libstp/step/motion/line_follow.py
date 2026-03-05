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
    """
    Follow a line using two IR sensors with PID steering.

    Uses LinearMotion for profiled distance control and odometry,
    with sensor-based PID steering overriding the heading controller.
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

        self._pid = PidController(PidConfig(kp=cfg.kp, ki=cfg.ki, kd=cfg.kd))

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
    """
    Follow a line using a single IR sensor with PID edge-tracking.

    Uses LinearMotion for profiled distance control and odometry,
    with sensor-based PID steering overriding the heading controller.
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

        self._pid = PidController(PidConfig(kp=cfg.kp, ki=cfg.ki, kd=cfg.kd))

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
    """
    Follow a line using a single IR sensor, stopping when a second sensor detects black.

    Uses LinearMotion for velocity control and odometry, with sensor-based
    PID steering overriding the heading controller. The stop sensor is
    checked each cycle; when it exceeds the threshold the step finishes.
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

        self._pid = PidController(PidConfig(kp=cfg.kp, ki=cfg.ki, kd=cfg.kd))

        self.debug(
            f"on_start: side={cfg.side.value}, speed_scale={cfg.speed_scale:.2f}, "
            f"stop_thresh={cfg.stop_threshold:.2f}, PID({cfg.kp}, {cfg.ki}, {cfg.kd})"
        )

    def on_update(self, robot: "GenericRobot", dt: float) -> bool:
        cfg = self.config

        # Check stop sensor
        stop_reading = cfg.stop_sensor.probabilityOfBlack()
        if stop_reading >= cfg.stop_threshold:
            self.debug(
                f"stop: stop_raw={cfg.stop_sensor.read():.0f} stop_black={stop_reading:.2f} >= {cfg.stop_threshold:.2f}"
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

        self.debug(
            f"raw={cfg.sensor.read():.0f} black={reading:.2f} stop_raw={cfg.stop_sensor.read():.0f} stop={stop_reading:.2f} err={error:.2f} wz={wz:.3f} dt={dt:.4f}"
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
    """
    Follow a line for a specified distance using two sensors.

    Args:
        left_sensor: Left IR sensor instance
        right_sensor: Right IR sensor instance
        distance_cm: Distance to follow in centimeters
        speed: Fraction of max speed, 0-1 (default 0.5)
        kp, ki, kd: PID gains for steering

    Returns:
        LineFollow step configured for distance-based following
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
    """
    Follow a line until both sensors detect black (intersection).

    Args:
        left_sensor: Left IR sensor instance
        right_sensor: Right IR sensor instance
        speed: Fraction of max speed, 0-1 (default 0.5)
        kp, ki, kd: PID gains for steering
        both_black_threshold: Both sensors must exceed this to stop

    Returns:
        LineFollow step that stops at intersections
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
    """
    Follow a line using a single sensor for a specified distance.

    The sensor tracks the edge of the line (where probabilityOfBlack ~ 0.5).
    ``side`` selects which edge to follow.

    Args:
        sensor: The IR sensor instance
        distance_cm: Distance to follow in centimeters
        speed: Fraction of max speed, 0-1 (default 0.5)
        side: Which edge to track (LEFT or RIGHT)
        kp, ki, kd: PID gains for steering

    Returns:
        SingleSensorLineFollow step
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
    """
    Follow a line using a single sensor, stopping when a second sensor detects black.

    The ``sensor`` tracks the edge of the line (where probabilityOfBlack ~ 0.5).
    ``side`` selects which edge to follow. The step finishes when
    ``stop_sensor.probabilityOfBlack()`` exceeds ``stop_threshold``.

    Args:
        sensor: The IR sensor used for edge-tracking
        stop_sensor: A second IR sensor; step stops when this reads black
        speed: Fraction of max speed, 0-1 (default 0.5)
        side: Which edge to track (LEFT or RIGHT)
        stop_threshold: probabilityOfBlack threshold for stop sensor (default 0.7)
        kp, ki, kd: PID gains for steering

    Returns:
        SingleSensorLineFollowUntilBlack step
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
