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

All steps support composable ``StopCondition`` via the ``.until()`` builder
method, enabling patterns like::

    follow_line(left, right, speed=0.5).until(on_black(left) & on_black(right))
    follow_line_single(sensor, speed=0.4).until(on_black(stop_sensor))
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import TYPE_CHECKING

from raccoon.foundation import PidConfig, PidController
from raccoon.motion import (
    DirectionalLineFollowMotion,
    DirectionalLineFollowMotionConfig,
    LinearAxis,
    LinearMotion,
    LinearMotionConfig,
    LineFollowCorrectionMode,
)
from raccoon.sensor_ir import IRSensor

from .. import dsl
from ..annotation import dsl_step
from ..condition import StopCondition
from ._heading_utils import get_world_heading_rad
from .motion_step import MotionStep

if TYPE_CHECKING:
    from raccoon.robot.api import GenericRobot


@dataclass
class LineFollowConfig:
    """Configuration for LineFollow step with two sensors."""

    left_sensor: IRSensor
    right_sensor: IRSensor
    speed_scale: float  # 0-1 fraction of max velocity
    distance_cm: float | None = None  # None = run until condition stops
    kp: float = 0.4
    ki: float = 0.0
    kd: float = 0.1


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
    distance_cm: float | None = None  # None = run until condition stops
    side: LineSide = LineSide.LEFT
    kp: float = 0.4
    ki: float = 0.0
    kd: float = 0.1


_SENTINEL_DISTANCE_M = 100.0  # Large distance; condition stops early


@dsl(hidden=True)
class LineFollow(MotionStep):
    """Follow a line using two IR sensors with PID steering.

    Computes a steering error as the difference between the left and right
    sensors' ``probabilityOfBlack()`` readings and feeds it through a PID
    controller. The PID output is applied as an angular velocity (omega)
    override on the underlying ``LinearMotion``, which handles profiled
    distance control and odometry integration.

    Supports distance-based termination, composable ``StopCondition`` via
    ``.until()``, or both (whichever triggers first).
    """

    def __init__(self, config: LineFollowConfig, until: StopCondition | None = None):
        super().__init__()
        self.config = config
        self._until = until
        self._motion: LinearMotion | None = None
        self._pid: PidController | None = None

    def _generate_signature(self) -> str:
        parts = []
        if self.config.distance_cm is not None:
            parts.append(f"{self.config.distance_cm:.1f}cm")
        if self._until is not None:
            parts.append("until")
        mode = "+".join(parts) if parts else "indefinite"
        return f"LineFollow(mode={mode}, speed={self.config.speed_scale:.2f})"

    def lower_to_segments(self) -> "list":
        from raccoon.motion import LinearAxis

        from .path.ir import Segment

        cfg = self.config
        return [
            Segment(
                kind="follow_line",
                axis=LinearAxis.Forward,
                sign=1.0,
                distance_m=cfg.distance_cm / 100.0 if cfg.distance_cm is not None else None,
                speed_scale=cfg.speed_scale,
                condition=None,
                has_known_endpoint=cfg.distance_cm is not None,
                opaque_step=self,
            )
        ]

    def on_start(self, robot: "GenericRobot") -> None:
        cfg = self.config

        motion_config = LinearMotionConfig()
        motion_config.axis = LinearAxis.Forward
        if cfg.distance_cm is not None:
            motion_config.distance_m = cfg.distance_cm / 100.0
        else:
            motion_config.distance_m = _SENTINEL_DISTANCE_M
        # Without an explicit distance the motion runs until an external
        # condition fires (sensor seeing the stop line, until=...). Tell
        # the C++ controller so it skips the SpeedMode distance check.
        motion_config.has_distance_target = cfg.distance_cm is not None
        motion_config.speed_scale = cfg.speed_scale
        # PID overrides omega each tick, but the underlying LinearMotion still
        # needs a valid absolute heading for its profile reference.
        motion_config.target_heading_rad = get_world_heading_rad(robot)

        self._motion = LinearMotion(
            robot.drive,
            robot.odometry,
            robot.motion_pid_config,
            motion_config,
        )
        self._motion.start()

        self._pid = PidController(
            PidConfig(
                kp=cfg.kp,
                ki=cfg.ki,
                kd=cfg.kd,
                integral_max=1.0,
                output_min=-1.0,
                output_max=1.0,
            )
        )

        if self._until is not None:
            self._until.start(robot)

        parts = []
        if cfg.distance_cm is not None:
            parts.append(f"{cfg.distance_cm:.1f}cm")
        if self._until is not None:
            parts.append("until")
        mode = "+".join(parts) if parts else "indefinite"
        self.debug(
            f"on_start: mode={mode}, speed_scale={cfg.speed_scale:.2f}, "
            f"PID({cfg.kp}, {cfg.ki}, {cfg.kd})"
        )

    def on_update(self, robot: "GenericRobot", dt: float) -> bool:
        cfg = self.config

        # Check composable stop condition
        if self._until is not None and self._until.check(robot):
            return True

        # Sensor error: left - right
        left_conf = cfg.left_sensor.probabilityOfBlack()
        right_conf = cfg.right_sensor.probabilityOfBlack()
        error = left_conf - right_conf

        # PID steering -> omega override on LinearMotion
        wz = self._pid.update(error, dt)
        self._motion.set_omega_override(wz)

        self.debug(f"L={left_conf:.2f} R={right_conf:.2f} err={error:.2f} wz={wz:.3f} dt={dt:.4f}")

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
    and odometry integration.

    Supports distance-based termination, composable ``StopCondition`` via
    ``.until()``, or both (whichever triggers first).
    """

    def __init__(self, config: SingleLineFollowConfig, until: StopCondition | None = None):
        super().__init__()
        self.config = config
        self._until = until
        self._motion: LinearMotion | None = None
        self._pid: PidController | None = None

    def _generate_signature(self) -> str:
        parts = []
        if self.config.distance_cm is not None:
            parts.append(f"{self.config.distance_cm:.1f}cm")
        if self._until is not None:
            parts.append("until")
        mode = "+".join(parts) if parts else "indefinite"
        return (
            f"SingleSensorLineFollow(mode={mode}, "
            f"side={self.config.side.value}, speed={self.config.speed_scale:.2f})"
        )

    def lower_to_segments(self) -> "list":
        from raccoon.motion import LinearAxis

        from .path.ir import Segment

        cfg = self.config
        return [
            Segment(
                kind="follow_line",
                axis=LinearAxis.Forward,
                sign=1.0,
                distance_m=cfg.distance_cm / 100.0 if cfg.distance_cm is not None else None,
                speed_scale=cfg.speed_scale,
                condition=None,
                has_known_endpoint=cfg.distance_cm is not None,
                opaque_step=self,
            )
        ]

    def on_start(self, robot: "GenericRobot") -> None:
        cfg = self.config

        motion_config = LinearMotionConfig()
        motion_config.axis = LinearAxis.Forward
        if cfg.distance_cm is not None:
            motion_config.distance_m = cfg.distance_cm / 100.0
        else:
            motion_config.distance_m = _SENTINEL_DISTANCE_M
        # Without an explicit distance the motion runs until an external
        # condition fires (sensor seeing the stop line, until=...). Tell
        # the C++ controller so it skips the SpeedMode distance check.
        motion_config.has_distance_target = cfg.distance_cm is not None
        motion_config.speed_scale = cfg.speed_scale
        motion_config.target_heading_rad = get_world_heading_rad(robot)

        self._motion = LinearMotion(
            robot.drive,
            robot.odometry,
            robot.motion_pid_config,
            motion_config,
        )
        self._motion.start()

        self._pid = PidController(
            PidConfig(
                kp=cfg.kp,
                ki=cfg.ki,
                kd=cfg.kd,
                integral_max=1.0,
                output_min=-1.0,
                output_max=1.0,
            )
        )

        if self._until is not None:
            self._until.start(robot)

        self.debug(
            f"on_start: distance={cfg.distance_cm}cm, side={cfg.side.value}, "
            f"speed_scale={cfg.speed_scale:.2f}, PID({cfg.kp}, {cfg.ki}, {cfg.kd})"
        )

    def on_update(self, robot: "GenericRobot", dt: float) -> bool:
        cfg = self.config

        # Check composable stop condition
        if self._until is not None and self._until.check(robot):
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
            f"raw={cfg.sensor.read():.0f} black={reading:.2f} err={error:.2f} wz={wz:.3f} dt={dt:.4f}"
        )

        self._motion.update(dt)
        return self._motion.is_finished()


# ---------------------------------------------------------------------------
# Profiled line follow — @dsl_step public classes
# ---------------------------------------------------------------------------


@dsl_step(tags=["motion", "line-follow"])
class FollowLine(LineFollow):
    """Follow a line using two IR sensors for steering.

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
        left_sensor: Left IR sensor instance, positioned to the left of the
            line.
        right_sensor: Right IR sensor instance, positioned to the right of
            the line.
        distance_cm: Distance to follow in centimeters. The step finishes
            when this distance has been traveled according to odometry.
            Optional if ``until`` is provided.
        speed: Fraction of max velocity (0.0--1.0). Lower speeds give the
            PID more time to correct but are slower overall. Default 0.5.
        kp: Proportional gain for steering PID. Higher values produce
            sharper corrections. Default 0.75.
        ki: Integral gain for steering PID. Typically left at 0.0 unless
            there is a persistent drift. Default 0.0.
        kd: Derivative gain for steering PID. Damps oscillation around the
            line. Default 0.5.
        until: Composable stop condition (e.g., ``on_black(left) &
            on_black(right)``). Can also be chained via the ``.until()``
            builder method.

    Returns:
        A ``FollowLine`` step configured for line following.

    Example::

        from raccoon.step.motion import FollowLine
        from raccoon.step.condition import on_black

        # Follow a line for 80 cm at half speed
        follow_line(left_sensor=left, right_sensor=right, distance_cm=80, speed=0.5)

        # Follow until both sensors see black (intersection)
        follow_line(left, right, speed=0.5).until(on_black(left) & on_black(right))

        # Distance + early termination
        follow_line(left, right, distance_cm=100, speed=0.5).until(on_black(third))
    """

    def __init__(
        self,
        left_sensor: IRSensor,
        right_sensor: IRSensor,
        distance_cm: float | None = None,
        speed: float = 0.5,
        kp: float = 0.4,
        ki: float = 0.0,
        kd: float = 0.1,
        until: StopCondition | None = None,
    ) -> None:
        if distance_cm is None and until is None:
            msg = "FollowLine requires either 'distance_cm' or 'until'"
            raise ValueError(msg)
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
            kp=kp,
            ki=ki,
            kd=kd,
        )
        super().__init__(config, until=until)

    def _generate_signature(self) -> str:
        parts = []
        if self._distance_cm is not None:
            parts.append(f"{self._distance_cm:.1f}cm")
        if self._until is not None:
            parts.append("until")
        mode = "+".join(parts)
        return f"FollowLine(mode={mode}, speed={self._speed:.2f})"


@dsl_step(tags=["motion", "line-follow"])
class FollowLineSingle(SingleSensorLineFollow):
    """Follow a line edge using a single IR sensor.

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
        distance_cm: Distance to follow in centimeters. The step finishes
            when this distance has been traveled. Optional if ``until`` is
            provided.
        speed: Fraction of max velocity (0.0--1.0). Default 0.5.
        side: Which edge of the line to track. ``LineSide.LEFT`` (default)
            or ``LineSide.RIGHT``.
        kp: Proportional gain for steering PID. Default 1.0.
        ki: Integral gain for steering PID. Default 0.0.
        kd: Derivative gain for steering PID. Default 0.3.
        until: Composable stop condition (e.g., ``on_black(stop_sensor)``).
            Can also be chained via the ``.until()`` builder method.

    Returns:
        A ``FollowLineSingle`` step.

    Example::

        from raccoon.step.motion import FollowLineSingle, LineSide
        from raccoon.step.condition import on_black

        # Follow left edge for 60 cm
        follow_line_single(sensor=front_ir, distance_cm=60, speed=0.4)

        # Follow until stop sensor sees black
        follow_line_single(front_ir, speed=0.4, side=LineSide.LEFT).until(on_black(stop))

        # Distance + early termination with timeout
        follow_line_single(front_ir, distance_cm=100).until(on_black(stop) | after_seconds(10))
    """

    def __init__(
        self,
        sensor: IRSensor,
        distance_cm: float | None = None,
        speed: float = 0.5,
        side: LineSide = LineSide.LEFT,
        kp: float = 0.4,
        ki: float = 0.0,
        kd: float = 0.1,
        until: StopCondition | None = None,
    ) -> None:
        if distance_cm is None and until is None:
            msg = "FollowLineSingle requires either 'distance_cm' or 'until'"
            raise ValueError(msg)
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
            kp=kp,
            ki=ki,
            kd=kd,
        )
        super().__init__(config, until=until)

    def _generate_signature(self) -> str:
        parts = []
        if self._distance_cm is not None:
            parts.append(f"{self._distance_cm:.1f}cm")
        if self._until is not None:
            parts.append("until")
        mode = "+".join(parts)
        return (
            f"FollowLineSingle(mode={mode}, " f"side={self._side.value}, speed={self._speed:.2f})"
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

    When ``lateral_correction`` is True, the PID output controls lateral
    velocity (vy) instead of angular velocity (wz), keeping the robot's
    heading constant while correcting position by strafing.  When
    ``forward_correction`` is True, the PID output controls forward velocity
    (vx), useful for following while the primary motion is lateral (vy).
    """

    left_sensor: IRSensor
    right_sensor: IRSensor
    heading_speed: float  # -1..1 fraction of max forward velocity
    strafe_speed: float  # -1..1 fraction of max lateral velocity (positive = right)
    distance_cm: float | None = None  # None = run until condition stops
    kp: float = 0.4
    ki: float = 0.0
    kd: float = 0.1
    lateral_correction: bool = False
    forward_correction: bool = False
    heading_hold: bool = True
    correction_sign: float = 1.0
    # Absolute heading (degrees from the HeadingReferenceService origin) to hold
    # while correcting. None = hold whatever heading the step starts at. Only
    # consulted for heading-hold modes (lateral/forward correction).
    heading_deg: float | None = None


def _correction_mode_name(
    cfg: DirectionalLineFollowConfig | DirectionalSingleLineFollowConfig,
) -> str:
    if cfg.forward_correction:
        return "forward"
    if cfg.lateral_correction:
        return "lateral"
    return "angular"


def _forward_correction_sign_for_lateral_follow(strafe_speed: float) -> float:
    # For +vy (right), the travel-right normal is -vx; for -vy it is +vx.
    return -1.0 if strafe_speed >= 0.0 else 1.0


def _apply_hold_heading(
    step: "MotionStep",
    motion_config: "DirectionalLineFollowMotionConfig",
    heading_deg: float | None,
    heading_hold: bool,
    correction_mode: "LineFollowCorrectionMode",
    robot: "GenericRobot",
) -> None:
    """Pin the heading-hold PID onto an absolute reference-relative heading.

    When ``heading_deg`` is given it is resolved to an absolute world heading
    via :class:`HeadingReferenceService` (same origin used by
    ``turn_to_heading_*`` and ``drive_forward(heading=…)``) and written onto
    the motion config. Absolute heading only takes effect in heading-hold modes
    (lateral/forward correction); pairing it with angular correction or
    ``heading_hold=False`` is a no-op, so we warn rather than fail.
    """
    if heading_deg is None:
        return

    if not heading_hold or correction_mode == LineFollowCorrectionMode.Angular:
        step.debug(
            f"heading_deg={heading_deg:.1f}° ignored: absolute heading only "
            f"applies to heading-hold lateral/forward correction "
            f"(correction={correction_mode}, heading_hold={heading_hold})"
        )
        return

    from raccoon.robot.heading_reference import HeadingReferenceService

    ref_svc = robot.get_service(HeadingReferenceService)
    motion_config.has_target_heading = True
    motion_config.target_heading_rad = ref_svc.target_absolute_rad(heading_deg)
    step.debug(
        f"holding absolute heading {heading_deg:.1f}° "
        f"(target_heading_rad={motion_config.target_heading_rad:.3f})"
    )


@dsl(hidden=True)
class DirectionalLineFollow(MotionStep):
    """Follow a line with independent heading and strafe velocity components.

    Uses direct ``ChassisVelocity`` control instead of ``LinearMotion``,
    enabling line following while strafing, driving diagonally, or any
    combination.  A PID controller computes angular velocity from the
    difference between the left and right sensors' ``probabilityOfBlack()``
    readings.

    Distance is tracked via odometry as euclidean distance from the start
    position.  Supports distance-based termination, composable
    ``StopCondition``, or both.
    """

    def __init__(self, config: DirectionalLineFollowConfig, until: StopCondition | None = None):
        super().__init__()
        self.config = config
        self._until = until
        self._motion: DirectionalLineFollowMotion | None = None

    def _generate_signature(self) -> str:
        parts = []
        if self.config.distance_cm is not None:
            parts.append(f"{self.config.distance_cm:.1f}cm")
        if self._until is not None:
            parts.append("until")
        mode = "+".join(parts) if parts else "indefinite"
        corr = _correction_mode_name(self.config)
        return (
            f"DirectionalLineFollow(mode={mode}, corr={corr}, "
            f"heading={self.config.heading_speed:.2f}, "
            f"strafe={self.config.strafe_speed:.2f})"
        )

    def lower_to_segments(self) -> "list":
        from raccoon.motion import LinearAxis

        from .path.ir import Segment

        cfg = self.config
        # Travel axis follows the *primary* motion. When forward_correction is
        # set the base motion is lateral (strafe on vy) and only the cross-track
        # is corrected on vx, so the robot travels on the Lateral axis. Angular
        # and lateral correction both keep forward (heading_speed) as the primary
        # travel, so they remain on the Forward axis. Arbitrary diagonal travel
        # (both heading and strafe non-zero) isn't axis-aligned; we default to
        # Forward — the warm-start heuristic is best-effort.
        if cfg.forward_correction:
            axis = LinearAxis.Lateral
            speed_scale = abs(cfg.strafe_speed)
        else:
            axis = LinearAxis.Forward
            speed_scale = abs(cfg.heading_speed)
        return [
            Segment(
                kind="follow_line",
                axis=axis,
                sign=1.0,
                distance_m=cfg.distance_cm / 100.0 if cfg.distance_cm is not None else None,
                speed_scale=speed_scale,
                condition=None,
                has_known_endpoint=cfg.distance_cm is not None,
                opaque_step=self,
            )
        ]

    def on_start(self, robot: "GenericRobot") -> None:
        cfg = self.config
        motion_config = DirectionalLineFollowMotionConfig()
        motion_config.heading_speed = cfg.heading_speed
        motion_config.strafe_speed = cfg.strafe_speed
        motion_config.distance_m = (cfg.distance_cm / 100.0) if cfg.distance_cm is not None else 0.0
        motion_config.has_distance_target = cfg.distance_cm is not None
        motion_config.kp = cfg.kp
        motion_config.ki = cfg.ki
        motion_config.kd = cfg.kd
        motion_config.heading_hold = cfg.heading_hold
        motion_config.correction_sign = cfg.correction_sign
        if cfg.forward_correction:
            motion_config.correction_mode = LineFollowCorrectionMode.Forward
        elif cfg.lateral_correction:
            motion_config.correction_mode = LineFollowCorrectionMode.Lateral
        else:
            motion_config.correction_mode = LineFollowCorrectionMode.Angular

        _apply_hold_heading(
            self,
            motion_config,
            cfg.heading_deg,
            cfg.heading_hold,
            motion_config.correction_mode,
            robot,
        )

        self._motion = DirectionalLineFollowMotion(
            robot.drive,
            robot.odometry,
            robot.motion_pid_config,
            motion_config,
        )
        self._motion.start()

        if self._until is not None:
            self._until.start(robot)

        parts = []
        if cfg.distance_cm is not None:
            parts.append(f"{cfg.distance_cm:.1f}cm")
        if self._until is not None:
            parts.append("until")
        mode = "+".join(parts) if parts else "indefinite"
        corr_str = _correction_mode_name(cfg)
        self.debug(
            f"on_start: mode={mode}, heading={cfg.heading_speed:.2f}, strafe={cfg.strafe_speed:.2f}, "
            f"correction={corr_str}, PID({cfg.kp}, {cfg.ki}, {cfg.kd})"
        )

    def on_update(self, robot: "GenericRobot", dt: float) -> bool:
        cfg = self.config

        # Check composable stop condition
        if self._until is not None and self._until.check(robot):
            return True

        left_conf = cfg.left_sensor.probabilityOfBlack()
        right_conf = cfg.right_sensor.probabilityOfBlack()

        error = left_conf - right_conf
        self._motion.set_sensor_error(error)
        self._motion.update(dt)
        telemetry = self._motion.get_telemetry()
        t = telemetry[-1] if telemetry else None
        if t is not None:
            self.debug(
                f"L={left_conf:.2f} R={right_conf:.2f} err={error:.2f} corr={t.correction:.3f} "
                f"-> vx={t.cmd_vx_mps:.3f} vy={t.cmd_vy_mps:.3f} wz={t.cmd_wz_radps:.3f} "
                f"(hdg_err={t.heading_error_rad:.3f}rad, dt={dt:.4f})"
            )
        else:
            self.debug(f"L={left_conf:.2f} R={right_conf:.2f} err={error:.2f} dt={dt:.4f}")
        return self._motion.is_finished()


@dataclass
class DirectionalSingleLineFollowConfig:
    """Configuration for directional single-sensor line following.

    The sensor tracks the edge of a line using PID control while the robot
    moves with the given heading and strafe velocity components.

    When ``lateral_correction`` is True, the PID output controls lateral
    velocity (vy) instead of angular velocity (wz), keeping the robot's
    heading constant while correcting position by strafing.  When
    ``forward_correction`` is True, the PID output controls forward velocity
    (vx), useful for following while the primary motion is lateral (vy).
    """

    sensor: IRSensor
    heading_speed: float  # -1..1 fraction of max forward velocity
    strafe_speed: float  # -1..1 fraction of max lateral velocity (positive = right)
    distance_cm: float | None = None  # None = run until condition stops
    side: LineSide = LineSide.LEFT
    kp: float = 0.4
    ki: float = 0.0
    kd: float = 0.1
    lateral_correction: bool = False
    forward_correction: bool = False
    heading_hold: bool = True
    correction_sign: float = 1.0
    # Absolute heading (degrees from the HeadingReferenceService origin) to hold
    # while correcting. None = hold whatever heading the step starts at. Only
    # consulted for heading-hold modes (lateral/forward correction).
    heading_deg: float | None = None


@dsl(hidden=True)
class DirectionalSingleLineFollow(MotionStep):
    """Follow a line edge with independent heading and strafe velocity.

    Targets ``probabilityOfBlack() = 0.5`` (the line edge) as the setpoint.
    The ``side`` configuration flips the error sign to select left vs. right
    edge tracking.  The PID output controls angular velocity while heading
    and strafe velocities are set directly via ``ChassisVelocity``.

    Supports distance-based and composable ``StopCondition``-based
    termination.
    """

    def __init__(
        self, config: DirectionalSingleLineFollowConfig, until: StopCondition | None = None
    ):
        super().__init__()
        self.config = config
        self._until = until
        self._motion: DirectionalLineFollowMotion | None = None

    def _generate_signature(self) -> str:
        parts = []
        if self.config.distance_cm is not None:
            parts.append(f"{self.config.distance_cm:.1f}cm")
        if self._until is not None:
            parts.append("until")
        mode = "+".join(parts) if parts else "indefinite"
        corr = _correction_mode_name(self.config)
        return (
            f"DirectionalSingleLineFollow(mode={mode}, corr={corr}, "
            f"side={self.config.side.value}, "
            f"heading={self.config.heading_speed:.2f}, "
            f"strafe={self.config.strafe_speed:.2f})"
        )

    def lower_to_segments(self) -> "list":
        from raccoon.motion import LinearAxis

        from .path.ir import Segment

        cfg = self.config
        # See DirectionalLineFollow.lower_to_segments for the axis rationale:
        # forward_correction => primary travel is lateral (strafe); otherwise
        # the primary travel is forward. Diagonal travel defaults to Forward.
        if cfg.forward_correction:
            axis = LinearAxis.Lateral
            speed_scale = abs(cfg.strafe_speed)
        else:
            axis = LinearAxis.Forward
            speed_scale = abs(cfg.heading_speed)
        return [
            Segment(
                kind="follow_line",
                axis=axis,
                sign=1.0,
                distance_m=cfg.distance_cm / 100.0 if cfg.distance_cm is not None else None,
                speed_scale=speed_scale,
                condition=None,
                has_known_endpoint=cfg.distance_cm is not None,
                opaque_step=self,
            )
        ]

    def on_start(self, robot: "GenericRobot") -> None:
        cfg = self.config
        motion_config = DirectionalLineFollowMotionConfig()
        motion_config.heading_speed = cfg.heading_speed
        motion_config.strafe_speed = cfg.strafe_speed
        motion_config.distance_m = (cfg.distance_cm / 100.0) if cfg.distance_cm is not None else 0.0
        motion_config.has_distance_target = cfg.distance_cm is not None
        motion_config.kp = cfg.kp
        motion_config.ki = cfg.ki
        motion_config.kd = cfg.kd
        motion_config.heading_hold = cfg.heading_hold
        motion_config.correction_sign = cfg.correction_sign
        if cfg.forward_correction:
            motion_config.correction_mode = LineFollowCorrectionMode.Forward
        elif cfg.lateral_correction:
            motion_config.correction_mode = LineFollowCorrectionMode.Lateral
        else:
            motion_config.correction_mode = LineFollowCorrectionMode.Angular

        _apply_hold_heading(
            self,
            motion_config,
            cfg.heading_deg,
            cfg.heading_hold,
            motion_config.correction_mode,
            robot,
        )

        self._motion = DirectionalLineFollowMotion(
            robot.drive,
            robot.odometry,
            robot.motion_pid_config,
            motion_config,
        )
        self._motion.start()

        if self._until is not None:
            self._until.start(robot)

        corr_str = _correction_mode_name(cfg)
        self.debug(
            f"on_start: side={cfg.side.value}, heading={cfg.heading_speed:.2f}, strafe={cfg.strafe_speed:.2f}, "
            f"correction={corr_str}, heading_hold={cfg.heading_hold}, PID({cfg.kp}, {cfg.ki}, {cfg.kd})"
        )

    def on_update(self, robot: "GenericRobot", dt: float) -> bool:
        cfg = self.config

        # Check composable stop condition
        if self._until is not None and self._until.check(robot):
            return True

        # Edge-tracking error: 0.5 = edge of line
        reading = cfg.sensor.probabilityOfBlack()
        error = reading - 0.5
        if cfg.side == LineSide.RIGHT:
            error = -error

        self._motion.set_sensor_error(error)
        self._motion.update(dt)
        telemetry = self._motion.get_telemetry()
        t = telemetry[-1] if telemetry else None
        if t is not None:
            self.debug(
                f"black={reading:.2f} err={error:.2f} corr={t.correction:.3f} "
                f"-> vx={t.cmd_vx_mps:.3f} vy={t.cmd_vy_mps:.3f} wz={t.cmd_wz_radps:.3f} "
                f"(hdg_err={t.heading_error_rad:.3f}rad, dt={dt:.4f})"
            )
        else:
            self.debug(f"black={reading:.2f} err={error:.2f} dt={dt:.4f}")
        return self._motion.is_finished()


# ---------------------------------------------------------------------------
# Directional line follow — @dsl_step public classes
# ---------------------------------------------------------------------------


@dsl_step(tags=["motion", "line-follow"])
class DirectionalFollowLine(DirectionalLineFollow):
    """Follow a line with independent heading and strafe speeds.

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
        left_sensor: Left IR sensor instance, positioned to the left of the
            line.
        right_sensor: Right IR sensor instance, positioned to the right of
            the line.
        distance_cm: Distance to follow in centimeters.  The step finishes
            when this euclidean distance has been traveled. Optional if
            ``until`` is provided.
        heading_speed: Forward/backward speed as a fraction of max velocity
            (-1.0 to 1.0).  Positive = forward, negative = backward.
            Default 0.0.
        strafe_speed: Lateral speed as a fraction of max velocity (-1.0 to
            1.0).  Positive = right, negative = left.  Default 0.0.
        kp: Proportional gain for steering PID.  Default 0.75.
        ki: Integral gain for steering PID.  Default 0.0.
        kd: Derivative gain for steering PID.  Default 0.5.
        until: Composable stop condition. Can also be chained via the
            ``.until()`` builder method.

    Returns:
        A ``DirectionalFollowLine`` step.

    Example::

        from raccoon.step.motion import DirectionalFollowLine
        from raccoon.step.condition import on_black

        # Strafe right while following a line for 50 cm
        directional_follow_line(left, right, distance_cm=50, strafe_speed=0.5)

        # Follow until both sensors see black
        directional_follow_line(left, right, strafe_speed=0.4).until(on_black(left) & on_black(right))
    """

    def __init__(
        self,
        left_sensor: IRSensor,
        right_sensor: IRSensor,
        distance_cm: float | None = None,
        heading_speed: float = 0.0,
        strafe_speed: float = 0.0,
        kp: float = 0.4,
        ki: float = 0.0,
        kd: float = 0.1,
        until: StopCondition | None = None,
    ) -> None:
        if distance_cm is None and until is None:
            msg = "DirectionalFollowLine requires either 'distance_cm' or 'until'"
            raise ValueError(msg)
        self._left_sensor = left_sensor
        self._right_sensor = right_sensor
        self._distance_cm = distance_cm
        self._heading_speed = heading_speed
        self._strafe_speed = strafe_speed
        self._kp = kp
        self._ki = ki
        self._kd = kd
        super().__init__(
            DirectionalLineFollowConfig(
                left_sensor=left_sensor,
                right_sensor=right_sensor,
                heading_speed=heading_speed,
                strafe_speed=strafe_speed,
                distance_cm=distance_cm,
                kp=kp,
                ki=ki,
                kd=kd,
            ),
            until=until,
        )

    def _generate_signature(self) -> str:
        parts = []
        if self._distance_cm is not None:
            parts.append(f"{self._distance_cm:.1f}cm")
        if self._until is not None:
            parts.append("until")
        mode = "+".join(parts)
        return (
            f"DirectionalFollowLine(mode={mode}, "
            f"heading={self._heading_speed:.2f}, strafe={self._strafe_speed:.2f})"
        )


@dsl_step(tags=["motion", "line-follow"])
class StrafeFollowLine(DirectionalLineFollow):
    """Follow a line forward, correcting position by strafing left/right.

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
        distance_cm: Distance to follow in centimeters. Optional if
            ``until`` is provided.
        speed: Forward speed as fraction of max velocity (0.0 to 1.0).
            Default 0.5.  Use negative values to drive backward.
        kp: Proportional gain for lateral PID.  Default 0.75.
        ki: Integral gain for lateral PID.  Default 0.0.
        kd: Derivative gain for lateral PID.  Default 0.5.
        heading_deg: Absolute heading to hold while correcting, in degrees
            from the :func:`mark_heading_reference` origin (CCW-positive, same
            convention as ``turn_to_heading_left`` and ``drive_forward(heading=…)``).
            ``None`` (default) holds whatever heading the robot has when the
            step starts.  Requires :func:`mark_heading_reference` earlier in the
            mission when set.
        until: Composable stop condition. Can also be chained via the
            ``.until()`` builder method.

    Returns:
        A ``StrafeFollowLine`` step configured for lateral correction.

    Example::

        from raccoon.step.motion import StrafeFollowLine
        from raccoon.step.condition import on_black

        # Follow a line for 40 cm, correcting via strafe
        strafe_follow_line(left, right, distance_cm=40, speed=0.4)

        # Follow until both sensors see black
        strafe_follow_line(left, right, speed=0.4).until(on_black(left) & on_black(right))

        # Hold the board-forward heading (0° from the marked reference)
        strafe_follow_line(left, right, distance_cm=40, speed=0.4, heading_deg=0)
    """

    def __init__(
        self,
        left_sensor: IRSensor,
        right_sensor: IRSensor,
        distance_cm: float | None = None,
        speed: float = 0.5,
        kp: float = 0.4,
        ki: float = 0.0,
        kd: float = 0.1,
        heading_deg: float | None = None,
        until: StopCondition | None = None,
    ) -> None:
        if distance_cm is None and until is None:
            msg = "StrafeFollowLine requires either 'distance_cm' or 'until'"
            raise ValueError(msg)
        self._left_sensor = left_sensor
        self._right_sensor = right_sensor
        self._distance_cm = distance_cm
        self._speed = speed
        self._kp = kp
        self._ki = ki
        self._kd = kd
        self._heading_deg = heading_deg
        super().__init__(
            DirectionalLineFollowConfig(
                left_sensor=left_sensor,
                right_sensor=right_sensor,
                heading_speed=speed,
                strafe_speed=0.0,
                distance_cm=distance_cm,
                kp=kp,
                ki=ki,
                kd=kd,
                lateral_correction=True,
                heading_deg=heading_deg,
            ),
            until=until,
        )

    def _generate_signature(self) -> str:
        parts = []
        if self._distance_cm is not None:
            parts.append(f"{self._distance_cm:.1f}cm")
        if self._until is not None:
            parts.append("until")
        mode = "+".join(parts)
        return f"StrafeFollowLine(mode={mode}, speed={self._speed:.2f})"


@dsl_step(tags=["motion", "line-follow"])
class StrafeFollowLineSingle(DirectionalSingleLineFollow):
    """Follow a line edge forward, correcting position by strafing.

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
        distance_cm: Distance to follow in centimeters. Optional if
            ``until`` is provided.
        speed: Forward speed as fraction of max velocity (0.0 to 1.0).
            Default 0.5.  Use negative values to drive backward.
        side: Which edge of the line to track.  Default ``LineSide.LEFT``.
        kp: Proportional gain for lateral PID.  Default 1.0.
        ki: Integral gain for lateral PID.  Default 0.0.
        kd: Derivative gain for lateral PID.  Default 0.3.
        heading_deg: Absolute heading to hold while correcting, in degrees
            from the :func:`mark_heading_reference` origin (CCW-positive).
            ``None`` (default) holds the heading captured at step start.
            Requires :func:`mark_heading_reference` earlier when set.
        until: Composable stop condition. Can also be chained via the
            ``.until()`` builder method.

    Returns:
        A ``StrafeFollowLineSingle`` step configured for lateral correction.

    Example::

        from raccoon.step.motion import StrafeFollowLineSingle, LineSide
        from raccoon.step.condition import on_black

        # Follow a line edge for 40 cm, correcting via strafe
        strafe_follow_line_single(front_ir, distance_cm=40, speed=0.4)

        # Follow until stop sensor sees black
        strafe_follow_line_single(front_ir, speed=0.4).until(on_black(stop))

        # Hold 0° (board forward) while correcting on the strafe axis
        strafe_follow_line_single(front_ir, distance_cm=40, speed=0.4, heading_deg=0)
    """

    def __init__(
        self,
        sensor: IRSensor,
        distance_cm: float | None = None,
        speed: float = 0.5,
        side: LineSide = LineSide.LEFT,
        kp: float = 0.4,
        ki: float = 0.0,
        kd: float = 0.1,
        heading_deg: float | None = None,
        until: StopCondition | None = None,
    ) -> None:
        if distance_cm is None and until is None:
            msg = "StrafeFollowLineSingle requires either 'distance_cm' or 'until'"
            raise ValueError(msg)
        self._sensor = sensor
        self._distance_cm = distance_cm
        self._speed = speed
        self._side = side
        self._kp = kp
        self._ki = ki
        self._kd = kd
        self._heading_deg = heading_deg
        super().__init__(
            DirectionalSingleLineFollowConfig(
                sensor=sensor,
                heading_speed=speed,
                strafe_speed=0.0,
                distance_cm=distance_cm,
                side=side,
                kp=kp,
                ki=ki,
                kd=kd,
                lateral_correction=True,
                heading_deg=heading_deg,
            ),
            until=until,
        )

    def _generate_signature(self) -> str:
        parts = []
        if self._distance_cm is not None:
            parts.append(f"{self._distance_cm:.1f}cm")
        if self._until is not None:
            parts.append("until")
        mode = "+".join(parts)
        return (
            f"StrafeFollowLineSingle(mode={mode}, "
            f"side={self._side.value}, speed={self._speed:.2f})"
        )


@dsl_step(tags=["motion", "line-follow"])
class LateralFollowLine(DirectionalLineFollow):
    """Follow a line while strafing laterally, correcting with forward/backward motion.

    The robot moves primarily along ``vy`` (right for positive ``speed``, left
    for negative ``speed``) while a PID controller corrects cross-track error
    on ``vx``. Heading is held constant with the heading PID. This is the
    lateral/omni counterpart to ``StrafeFollowLine``, whose primary motion is
    forward and whose correction is lateral.

    ``left_sensor`` and ``right_sensor`` are interpreted relative to the
    current lateral follow direction. When strafing right, the left sensor is
    the forward-side sensor and the right sensor is the rear-side sensor; when
    strafing left, that geometry is mirrored.

    Supports distance-based termination, composable ``StopCondition`` via
    ``.until()``, or both (whichever triggers first). At least one of
    ``distance_cm`` or ``until`` must be provided.

    Args:
        left_sensor: Sensor on the travel-left side of the line.
        right_sensor: Sensor on the travel-right side of the line.
        distance_cm: Lateral distance to follow in centimeters. Optional if
            ``until`` is provided.
        speed: Lateral speed as a fraction of max velocity (-1.0 to 1.0).
            Positive strafes right, negative strafes left.
        kp: Proportional gain for cross-track PID.
        ki: Integral gain for cross-track PID.
        kd: Derivative gain for cross-track PID.
        heading_deg: Absolute heading to hold while strafing, in degrees from
            the :func:`mark_heading_reference` origin (CCW-positive). ``None``
            (default) holds the heading captured at step start. Requires
            :func:`mark_heading_reference` earlier when set.
        until: Composable stop condition. Can also be chained via the
            ``.until()`` builder method.
    """

    def __init__(
        self,
        left_sensor: IRSensor,
        right_sensor: IRSensor,
        distance_cm: float | None = None,
        speed: float = 0.5,
        kp: float = 0.4,
        ki: float = 0.0,
        kd: float = 0.1,
        heading_deg: float | None = None,
        until: StopCondition | None = None,
    ) -> None:
        if distance_cm is None and until is None:
            msg = "LateralFollowLine requires either 'distance_cm' or 'until'"
            raise ValueError(msg)
        self._left_sensor = left_sensor
        self._right_sensor = right_sensor
        self._distance_cm = distance_cm
        self._speed = speed
        self._kp = kp
        self._ki = ki
        self._kd = kd
        self._heading_deg = heading_deg
        super().__init__(
            DirectionalLineFollowConfig(
                left_sensor=left_sensor,
                right_sensor=right_sensor,
                heading_speed=0.0,
                strafe_speed=speed,
                distance_cm=distance_cm,
                kp=kp,
                ki=ki,
                kd=kd,
                forward_correction=True,
                correction_sign=_forward_correction_sign_for_lateral_follow(speed),
                heading_deg=heading_deg,
            ),
            until=until,
        )

    def _generate_signature(self) -> str:
        parts = []
        if self._distance_cm is not None:
            parts.append(f"{self._distance_cm:.1f}cm")
        if self._until is not None:
            parts.append("until")
        mode = "+".join(parts)
        return f"LateralFollowLine(mode={mode}, speed={self._speed:.2f})"


@dsl_step(tags=["motion", "line-follow"])
class LateralFollowLineSingle(DirectionalSingleLineFollow):
    """Follow a line edge while strafing laterally.

    The robot moves primarily along ``vy`` (right for positive ``speed``, left
    for negative ``speed``) while a PID controller corrects the edge-tracking
    error on ``vx``. ``LineSide.LEFT`` and ``LineSide.RIGHT`` are interpreted
    relative to the lateral follow direction, not the robot's fixed front.
    That means the correction sign is mirrored automatically when ``speed`` is
    negative.

    Supports distance-based termination, composable ``StopCondition`` via
    ``.until()``, or both (whichever triggers first). At least one of
    ``distance_cm`` or ``until`` must be provided.

    Args:
        sensor: IR sensor for edge tracking.
        distance_cm: Lateral distance to follow in centimeters. Optional if
            ``until`` is provided.
        speed: Lateral speed as a fraction of max velocity (-1.0 to 1.0).
            Positive strafes right, negative strafes left.
        side: Which edge of the line to track, relative to the lateral travel
            direction.
        kp: Proportional gain for cross-track PID.
        ki: Integral gain for cross-track PID.
        kd: Derivative gain for cross-track PID.
        heading_deg: Absolute heading to hold while strafing, in degrees from
            the :func:`mark_heading_reference` origin (CCW-positive). ``None``
            (default) holds the heading captured at step start. Requires
            :func:`mark_heading_reference` earlier when set.
        until: Composable stop condition. Can also be chained via the
            ``.until()`` builder method.
    """

    def __init__(
        self,
        sensor: IRSensor,
        distance_cm: float | None = None,
        speed: float = 0.5,
        side: LineSide = LineSide.LEFT,
        kp: float = 0.4,
        ki: float = 0.0,
        kd: float = 0.1,
        heading_deg: float | None = None,
        until: StopCondition | None = None,
    ) -> None:
        if distance_cm is None and until is None:
            msg = "LateralFollowLineSingle requires either 'distance_cm' or 'until'"
            raise ValueError(msg)
        self._sensor = sensor
        self._distance_cm = distance_cm
        self._speed = speed
        self._side = side
        self._kp = kp
        self._ki = ki
        self._kd = kd
        self._heading_deg = heading_deg
        super().__init__(
            DirectionalSingleLineFollowConfig(
                sensor=sensor,
                heading_speed=0.0,
                strafe_speed=speed,
                distance_cm=distance_cm,
                side=side,
                kp=kp,
                ki=ki,
                kd=kd,
                forward_correction=True,
                correction_sign=_forward_correction_sign_for_lateral_follow(speed),
                heading_deg=heading_deg,
            ),
            until=until,
        )

    def _generate_signature(self) -> str:
        parts = []
        if self._distance_cm is not None:
            parts.append(f"{self._distance_cm:.1f}cm")
        if self._until is not None:
            parts.append("until")
        mode = "+".join(parts)
        return (
            f"LateralFollowLineSingle(mode={mode}, "
            f"side={self._side.value}, speed={self._speed:.2f})"
        )


@dsl_step(tags=["motion", "line-follow"])
class DirectionalFollowLineSingle(DirectionalSingleLineFollow):
    """Follow a line edge with a single sensor and independent heading/strafe speeds.

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
        distance_cm: Distance to follow in centimeters. Optional if
            ``until`` is provided.
        heading_speed: Forward/backward speed fraction (-1.0 to 1.0).
            Default 0.0.
        strafe_speed: Lateral speed fraction (-1.0 to 1.0).  Default 0.0.
        side: Which edge of the line to track.  Default ``LineSide.LEFT``.
        kp: Proportional gain for steering PID.  Default 1.0.
        ki: Integral gain for steering PID.  Default 0.0.
        kd: Derivative gain for steering PID.  Default 0.3.
        until: Composable stop condition. Can also be chained via the
            ``.until()`` builder method.

    Returns:
        A ``DirectionalFollowLineSingle`` step.

    Example::

        from raccoon.step.motion import DirectionalFollowLineSingle, LineSide
        from raccoon.step.condition import on_black

        # Strafe right while tracking the left edge for 50 cm
        directional_follow_line_single(front_ir, distance_cm=50, strafe_speed=0.4)

        # Follow until stop sensor sees black
        directional_follow_line_single(front_ir, strafe_speed=0.4).until(on_black(stop))
    """

    def __init__(
        self,
        sensor: IRSensor,
        distance_cm: float | None = None,
        heading_speed: float = 0.0,
        strafe_speed: float = 0.0,
        side: LineSide = LineSide.LEFT,
        kp: float = 0.4,
        ki: float = 0.0,
        kd: float = 0.1,
        until: StopCondition | None = None,
    ) -> None:
        if distance_cm is None and until is None:
            msg = "DirectionalFollowLineSingle requires either 'distance_cm' or 'until'"
            raise ValueError(msg)
        self._sensor = sensor
        self._distance_cm = distance_cm
        self._heading_speed = heading_speed
        self._strafe_speed = strafe_speed
        self._side = side
        self._kp = kp
        self._ki = ki
        self._kd = kd
        super().__init__(
            DirectionalSingleLineFollowConfig(
                sensor=sensor,
                heading_speed=heading_speed,
                strafe_speed=strafe_speed,
                distance_cm=distance_cm,
                side=side,
                kp=kp,
                ki=ki,
                kd=kd,
            ),
            until=until,
        )

    def _generate_signature(self) -> str:
        parts = []
        if self._distance_cm is not None:
            parts.append(f"{self._distance_cm:.1f}cm")
        if self._until is not None:
            parts.append("until")
        mode = "+".join(parts)
        return (
            f"DirectionalFollowLineSingle(mode={mode}, "
            f"side={self._side.value}, heading={self._heading_speed:.2f}, "
            f"strafe={self._strafe_speed:.2f})"
        )
