"""Mecanum drivetrain tests for SimWorld.

Validates the four-wheel forward-kinematics branch added alongside the
existing diff-drive path. Geometric expectations come from the standard
mecanum mixing matrix (matches MecanumKinematics::estimateState):

    vx =  (fl + fr + bl + br) * R / 4
    vy =  (fl - fr - bl + br) * R / 4
    wz = (-fl + fr - bl + br) * R / (4 * L),   L = (wheelbase + track) / 2
"""
from __future__ import annotations

import math

import pytest

from conftest import SCENES_DIR  # type: ignore[import-not-found]


@pytest.fixture
def mecanum_robot(sim_module):
    r = sim_module.RobotConfig()
    r.width_cm = 24.0
    r.length_cm = 30.0
    r.wheel_radius_m = 0.0375
    r.track_width_m = 0.20
    r.wheelbase_m = 0.125
    return r


@pytest.fixture
def mecanum_motors(sim_module):
    m = sim_module.SimMotorMap()
    m.kind = sim_module.DrivetrainKind.MECANUM
    m.fl_port = 0
    m.fr_port = 1
    m.bl_port = 2
    m.br_port = 3
    m.max_wheel_velocity_rad_s = 30.0
    m.motor_time_constant_sec = 0.01  # snappy — clean math for tests
    return m


@pytest.fixture
def mecanum_world(sim_module, mecanum_robot, mecanum_motors):
    world = sim_module.SimWorld()
    world.configure(mecanum_robot, mecanum_motors)
    m = sim_module.WorldMap()
    m.load_ftmap(str(SCENES_DIR / "empty_table.ftmap"))
    world.set_map(m)
    world.set_pose(sim_module.Pose2D(100.0, 100.0, 0.0))
    return world


def _run(world, seconds: float, dt: float = 0.005) -> None:
    steps = round(seconds / dt)
    for _ in range(steps):
        world.tick(dt)


def test_pure_strafe_moves_lateral_only(mecanum_world):
    """FL=+50%, FR=-50%, BL=-50%, BR=+50% → pure strafe, no rotation, no forward."""
    w = mecanum_world
    w.set_motor_command(0, 50)   # FL
    w.set_motor_command(1, -50)  # FR
    w.set_motor_command(2, -50)  # BL
    w.set_motor_command(3, 50)   # BR
    _run(w, 1.0)

    p = w.pose
    # x and theta must not drift; y must move materially.
    assert math.isclose(p.x, 100.0, abs_tol=1.0), f"unexpected x drift: {p.x}"
    assert math.isclose(p.theta, 0.0, abs_tol=1e-3), f"unexpected yaw: {p.theta}"
    # vy = 0.5·R = 0.01875 m/s → ~1.8 cm in 1 s after motor ramp.
    assert abs(p.y - 100.0) > 1.0, f"strafe produced no y-motion: {p.y}"


def test_pure_forward_uses_all_four_wheels(mecanum_world, sim_module):
    """All four wheels at +50% → forward only, no strafe, no rotation."""
    w = mecanum_world
    for port in (0, 1, 2, 3):
        w.set_motor_command(port, 50)
    _run(w, 1.0)

    p = w.pose
    # vx = R·ω with ω = 0.5·30 = 15 rad/s → 0.5625 m/s → ~56 cm.
    assert p.x > 145.0, f"forward fell short: {p.x}"
    assert math.isclose(p.y, 100.0, abs_tol=1.0), f"unexpected lateral drift: {p.y}"
    assert math.isclose(p.theta, 0.0, abs_tol=1e-3), f"unexpected yaw: {p.theta}"


def test_pure_rotation(mecanum_world):
    """FL=-50%, FR=+50%, BL=-50%, BR=+50% → spin in place, no translation."""
    w = mecanum_world
    w.set_motor_command(0, -50)  # FL
    w.set_motor_command(1, 50)   # FR
    w.set_motor_command(2, -50)  # BL
    w.set_motor_command(3, 50)   # BR
    _run(w, 1.0)

    p = w.pose
    assert math.isclose(p.x, 100.0, abs_tol=1.0), f"unexpected x drift: {p.x}"
    assert math.isclose(p.y, 100.0, abs_tol=1.0), f"unexpected y drift: {p.y}"
    assert abs(p.theta) > 0.5, f"rotation did not progress: {p.theta}"


def test_inverted_wheels_flip_command(sim_module, mecanum_robot):
    """Inverting FR + BL turns an "all-forward" command (all four ports
    +50%) into pure strafe. With those two wheels flipped, the wheel-layer
    speeds are FL=+, FR=-, BL=-, BR=+, which is exactly the strafe pattern
    in the mecanum FK: vx = 0, vy ≠ 0, ω = 0."""
    motors = sim_module.SimMotorMap()
    motors.kind = sim_module.DrivetrainKind.MECANUM
    motors.fl_port = 0
    motors.fr_port = 1
    motors.bl_port = 2
    motors.br_port = 3
    motors.fr_inverted = True
    motors.bl_inverted = True
    motors.max_wheel_velocity_rad_s = 30.0
    motors.motor_time_constant_sec = 0.01

    world = sim_module.SimWorld()
    world.configure(mecanum_robot, motors)
    m = sim_module.WorldMap()
    m.load_ftmap(str(SCENES_DIR / "empty_table.ftmap"))
    world.set_map(m)
    world.set_pose(sim_module.Pose2D(100.0, 100.0, 0.0))

    for port in (0, 1, 2, 3):
        world.set_motor_command(port, 50)
    _run(world, 1.0)

    p = world.pose
    assert math.isclose(p.x, 100.0, abs_tol=1.0), f"unexpected x drift: {p.x}"
    assert math.isclose(p.theta, 0.0, abs_tol=1e-3), f"unexpected yaw: {p.theta}"
    assert abs(p.y - 100.0) > 1.0, "inversion did not redirect motion to strafe"
