"""High-level Python API for the libstp simulator.

This module wraps :mod:`raccoon.sim` (the pybind11 bindings) with ergonomic
helpers so a team's pytest can do::

    from raccoon.testing.sim import use_scene

    with use_scene("empty_table.ftmap",
                   robot=my_robot_config,
                   start=(50, 50, 0)):
        await drive_forward(cm=30).run_step(robot)
        assert pose().x == pytest.approx(80.0, abs=2.0)

The context manager configures the process-wide ``MockPlatform`` singleton
to talk to a fresh ``SimWorld`` for the duration of the block, then detaches
on exit so subsequent code paths see the stock mock HAL.

Only available when the wheel is built with ``DRIVER_BUNDLE=mock`` — the
``raccoon.sim.mock`` submodule must be present. Importing this module on a
wombat-bundle wheel raises ``RuntimeError``.

This module lives under :mod:`raccoon.testing` because it is exclusively
a test-harness concern; the actual C++ sim bindings still live at
``raccoon.sim``. Historically this API was at ``raccoon.step.sim`` — that
path remains as a deprecation shim and will be removed in a future release.
"""
from __future__ import annotations

from contextlib import contextmanager
from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterator, List, Literal, Optional, Tuple, Union

try:
    from raccoon import sim as _sim
except ImportError as exc:  # pragma: no cover - pure import guard
    msg = (
        "raccoon.sim is not installed — build the library with "
        "`pip install -e . --config-settings=cmake.define.DRIVER_BUNDLE=mock`"
    )
    raise RuntimeError(msg) from exc

if not hasattr(_sim, "mock"):
    msg = (
        "raccoon.sim.mock submodule is missing. The wheel was built without "
        "DRIVER_BUNDLE=mock; rebuild with that option to use the simulator."
    )
    raise RuntimeError(msg)

_mock = _sim.mock

PoseTuple = Tuple[float, float, float]
PoseLike = Union[PoseTuple, "_sim.Pose2D"]


@dataclass
class LineSensorMount:
    analog_port: int
    forward_cm: float
    strafe_cm: float
    name: str = ""


@dataclass
class DistanceSensorMount:
    analog_port: int
    forward_cm: float
    strafe_cm: float
    mount_angle_rad: float = 0.0
    max_range_cm: float = 100.0
    name: str = ""


@dataclass
class SimRobotConfig:
    """Sim-side counterpart to the team's GenericRobot geometry/kinematics.

    Defaults match a typical Botball wombat. Override fields that differ on
    your robot — they must agree with whatever the real ``Drive`` /
    ``DifferentialKinematics`` were constructed with so the sim physics
    matches what the motion controllers expect.
    """

    width_cm: float = 18.0
    length_cm: float = 18.0
    rotation_center_forward_cm: float = 0.0
    rotation_center_strafe_cm: float = 0.0
    wheel_radius_m: float = 0.03
    track_width_m: float = 0.15
    wheelbase_m: float = 0.15

    drivetrain: Literal["diff", "mecanum"] = "diff"

    # Differential drive — used when drivetrain == "diff".
    left_motor_port: int = 0
    right_motor_port: int = 1
    left_motor_inverted: bool = False
    right_motor_inverted: bool = False

    # Mecanum drive — used when drivetrain == "mecanum".
    fl_motor_port: int = 0
    fr_motor_port: int = 1
    bl_motor_port: int = 2
    br_motor_port: int = 3
    fl_motor_inverted: bool = False
    fr_motor_inverted: bool = False
    bl_motor_inverted: bool = False
    br_motor_inverted: bool = False

    max_wheel_velocity_rad_s: float = 30.0
    motor_time_constant_sec: float = 0.05

    ticks_to_rad: float = 2.0 * 3.14159265358979 / 1440.0

    viscous_drag_coeff: float = 0.0
    coulomb_friction_rad_s2: float = 0.0
    bemf_noise_stddev: float = 0.0

    line_sensors: List[LineSensorMount] = field(default_factory=list)
    distance_sensors: List[DistanceSensorMount] = field(default_factory=list)


def _to_pose(pose: PoseLike) -> "_sim.Pose2D":
    if isinstance(pose, _sim.Pose2D):
        return pose
    if len(pose) != 3:
        msg = f"start pose must be a (x, y, theta) tuple, got {pose!r}"
        raise ValueError(msg)
    return _sim.Pose2D(float(pose[0]), float(pose[1]), float(pose[2]))


def _build_native_robot(cfg: SimRobotConfig) -> "_sim.RobotConfig":
    r = _sim.RobotConfig()
    r.width_cm = cfg.width_cm
    r.length_cm = cfg.length_cm
    r.rotation_center_forward_cm = cfg.rotation_center_forward_cm
    r.rotation_center_strafe_cm = cfg.rotation_center_strafe_cm
    r.wheel_radius_m = cfg.wheel_radius_m
    r.track_width_m = cfg.track_width_m
    r.wheelbase_m = cfg.wheelbase_m
    return r


def _build_native_motors(cfg: SimRobotConfig) -> "_sim.SimMotorMap":
    m = _sim.SimMotorMap()
    if cfg.drivetrain == "mecanum":
        m.kind = _sim.DrivetrainKind.MECANUM
        m.fl_port = cfg.fl_motor_port
        m.fr_port = cfg.fr_motor_port
        m.bl_port = cfg.bl_motor_port
        m.br_port = cfg.br_motor_port
        m.fl_inverted = cfg.fl_motor_inverted
        m.fr_inverted = cfg.fr_motor_inverted
        m.bl_inverted = cfg.bl_motor_inverted
        m.br_inverted = cfg.br_motor_inverted
    else:
        m.kind = _sim.DrivetrainKind.DIFFERENTIAL
        m.left_port = cfg.left_motor_port
        m.right_port = cfg.right_motor_port
        m.left_inverted = cfg.left_motor_inverted
        m.right_inverted = cfg.right_motor_inverted
    m.max_wheel_velocity_rad_s = cfg.max_wheel_velocity_rad_s
    m.motor_time_constant_sec = cfg.motor_time_constant_sec
    m.ticks_to_rad = cfg.ticks_to_rad
    m.viscous_drag_coeff = cfg.viscous_drag_coeff
    m.coulomb_friction_rad_s2 = cfg.coulomb_friction_rad_s2
    m.bemf_noise_stddev = cfg.bemf_noise_stddev
    return m


def configure(
    scene: Union[str, Path],
    *,
    robot: Optional[SimRobotConfig] = None,
    start: PoseLike = (0.0, 0.0, 0.0),
    auto_tick: bool = True,
    auto_tick_max_step_sec: float = 0.05,
) -> None:
    """Attach a fresh sim to the MockPlatform singleton.

    After this returns, motor commands written through any HAL ``Motor``
    drive the simulated chassis, and ``OdometryBridge::readOdometry`` reports
    its pose. Auto-tick is enabled by default so a real motion loop reading
    odometry on its own schedule will see the sim advance with wall time.
    """
    cfg = robot or SimRobotConfig()
    world_map = _sim.WorldMap()
    world_map.load_ftmap(str(scene))

    _mock.configure(
        _build_native_robot(cfg),
        _build_native_motors(cfg),
        world_map,
        _to_pose(start),
    )

    for ls in cfg.line_sensors:
        _mock.attach_line_sensor(
            ls.analog_port, ls.forward_cm, ls.strafe_cm, ls.name)
    for ds in cfg.distance_sensors:
        _mock.attach_distance_sensor(
            ds.analog_port, ds.forward_cm, ds.strafe_cm,
            ds.mount_angle_rad, ds.max_range_cm, ds.name)

    _mock.set_auto_tick_max_step(auto_tick_max_step_sec)
    _mock.enable_auto_tick(auto_tick)


def detach() -> None:
    """Disconnect any attached sim. The mock HAL goes back to zero odometry."""
    _mock.enable_auto_tick(False)
    _mock.detach()


def pose() -> "_sim.Pose2D":
    """Return the sim's current ground-truth pose (cm, cm, rad)."""
    return _mock.pose()


def yaw_rate() -> float:
    """Return the sim's current yaw rate in rad/s."""
    return _mock.yaw_rate()


def tick(dt_seconds: float) -> None:
    """Manually advance the sim. Use this for fully deterministic tests."""
    _mock.tick(dt_seconds)


@contextmanager
def use_scene(
    scene: Union[str, Path],
    *,
    robot: Optional[SimRobotConfig] = None,
    start: PoseLike = (0.0, 0.0, 0.0),
    auto_tick: bool = True,
    auto_tick_max_step_sec: float = 0.05,
) -> Iterator[None]:
    """Context manager: attach the sim for the duration of a ``with`` block.

    Example::

        with use_scene("empty_table.ftmap", start=(20, 50, 0)):
            await drive_forward(cm=30).run_step(my_robot)
            x, y, _ = pose()
            assert x == pytest.approx(50, abs=2)
    """
    configure(
        scene,
        robot=robot,
        start=start,
        auto_tick=auto_tick,
        auto_tick_max_step_sec=auto_tick_max_step_sec,
    )
    try:
        yield
    finally:
        detach()


__all__ = [
    "DistanceSensorMount",
    "LineSensorMount",
    "SimRobotConfig",
    "configure",
    "detach",
    "pose",
    "tick",
    "use_scene",
    "yaw_rate",
]
