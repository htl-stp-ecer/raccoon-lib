"""End-to-end tests driving the MockPlatform singleton from Python.

These prove the full chain:
    sim.mock.set_motor_command → MockPlatform::setMotor →
    SimWorld::setMotorCommand → tick → MockPlatform::simPose →
    OdometryBridge::readOdometry

Without needing an installed raccoon package. The real motion steps
(``drive(30)``, ``line_follow``, ``lineup_forward``) can't be exercised here
because they depend on the full motion / async Python stack; those tests live
in the post-install suite. But the HAL → sim → odometry contract we're
proving here is exactly what those steps consume, so if this passes, the
sim is ready for them.
"""

from __future__ import annotations

import math

import pytest
from conftest import SCENES_DIR  # type: ignore[import-not-found]


@pytest.fixture
def mock(sim_module):
    if not hasattr(sim_module, "mock"):
        pytest.skip("sim.mock not present — build with DRIVER_BUNDLE=mock")
    yield sim_module.mock
    sim_module.mock.detach()


def _make_cfg(sim_module):
    robot = sim_module.RobotConfig()
    robot.width_cm = 18.0
    robot.length_cm = 18.0
    robot.wheel_radius_m = 0.03
    robot.track_width_m = 0.15

    motors = sim_module.SimMotorMap()
    motors.left_port = 0
    motors.right_port = 1
    motors.max_wheel_velocity_rad_s = 30.0
    motors.motor_time_constant_sec = 0.01

    return robot, motors


def _configure(
    sim_module, mock, scene_name="empty_table.ftmap", start_x=50.0, start_y=50.0, start_theta=0.0
):
    robot, motors = _make_cfg(sim_module)
    world_map = sim_module.WorldMap()
    world_map.load_ftmap(str(SCENES_DIR / scene_name))
    mock.configure(
        robot,
        motors,
        world_map,
        sim_module.Pose2D(start_x, start_y, start_theta),
    )


def _tick(mock, seconds: float, dt: float = 0.01) -> None:
    for _ in range(round(seconds / dt)):
        mock.tick(dt)


def test_mock_drive_forward(sim_module, mock):
    _configure(sim_module, mock, start_x=20.0, start_y=50.0)

    # Drive through the HAL-equivalent path — this is what Motor::setSpeed
    # does under the hood.
    mock.set_motor_command(0, 100)
    mock.set_motor_command(1, 100)
    _tick(mock, 1.0)

    # read_odometry returns pose RELATIVE to last reset / configure, mirroring
    # the bridge that real STM32 firmware exposes.
    x_m, y_m, heading, _ = mock.read_odometry()
    # Drove ~0.9 m/s × 1 s → ~0.90 m forward of origin.
    assert 0.85 < x_m < 0.95, f"got x_m={x_m}"
    assert math.isclose(y_m, 0.0, abs_tol=0.01)
    assert math.isclose(heading, 0.0, abs_tol=0.01)

    # Ground-truth absolute pose still available via mock.pose().
    abs_pose = mock.pose()
    assert math.isclose(abs_pose.x, 20.0 + x_m * 100.0, abs_tol=0.5)


def test_mock_turn_in_place(sim_module, mock):
    _configure(sim_module, mock, start_x=50.0, start_y=50.0)

    mock.set_motor_command(0, 100)
    mock.set_motor_command(1, -100)
    _tick(mock, 0.2)

    x_m, y_m, heading, yaw_rate = mock.read_odometry()
    # In-place spin → relative position stays at origin (0, 0).
    assert math.isclose(x_m, 0.0, abs_tol=5e-3)
    assert math.isclose(y_m, 0.0, abs_tol=5e-3)
    assert -3.0 < heading < -1.5
    assert yaw_rate < -5.0  # steady-state ≈ -12 rad/s


def test_mock_wall_stops_robot(sim_module, mock):
    _configure(sim_module, mock, start_x=20.0, start_y=50.0)

    mock.set_motor_command(0, 100)
    mock.set_motor_command(1, 100)
    _tick(mock, 5.0)  # long enough to fully traverse a 200 cm table

    # Use absolute pose for the wall assertion — the bridge-relative value
    # depends on the start, but the wall is at a known absolute x.
    x_cm = mock.pose().x
    # East border at 200 cm, half-length 9 cm → stops near 191 cm.
    assert 170 < x_cm < 192


def test_mock_brake_stops_sim(sim_module, mock):
    _configure(sim_module, mock, start_x=20.0, start_y=50.0)

    mock.set_motor_command(0, 100)
    mock.set_motor_command(1, 100)
    _tick(mock, 0.5)
    x_after_drive, *_ = mock.read_odometry()
    # Drove for 0.5 s at ~0.9 m/s → ~0.45 m relative to origin.
    assert x_after_drive > 0.30

    # Brake = zero command.
    mock.set_motor_command(0, 0)
    mock.set_motor_command(1, 0)
    _tick(mock, 1.0)
    x_after_brake, *_ = mock.read_odometry()
    # Small residual drift allowed (first-order motor decay).
    assert (x_after_brake - x_after_drive) < 0.10


def test_mock_line_sensor_through_hal(sim_module, mock):
    _configure(sim_module, mock, scene_name="single_line.ftmap", start_x=100.0, start_y=50.0)
    mock.attach_line_sensor(2, 0.0, 0.0, "center")

    # Centered on the horizontal line (y=50, runs x=50..150) → sensor = 0.
    assert mock.read_analog(2) == 0

    # Move off the line (change pose by setting motor command and ticking).
    # Simpler: reconfigure at a different pose.
    _configure(sim_module, mock, scene_name="single_line.ftmap", start_x=100.0, start_y=20.0)
    mock.attach_line_sensor(2, 0.0, 0.0, "center")
    assert mock.read_analog(2) == 1023


def test_mock_auto_tick_advances_sim_without_manual_tick(sim_module, mock):
    """Auto-tick with a wall-clock source — the sim advances as real time
    passes, which is what a running Python mission loop would need."""
    _configure(sim_module, mock, start_x=20.0, start_y=50.0)

    mock.set_motor_command(0, 100)
    mock.set_motor_command(1, 100)

    mock.set_auto_tick_max_step(0.05)
    mock.enable_auto_tick(True)

    # Burn wall-clock time — the sim should advance on each read_odometry.
    import time

    deadline = time.monotonic() + 0.5
    while time.monotonic() < deadline:
        mock.read_odometry()
        time.sleep(0.005)

    x_m, *_ = mock.read_odometry()
    # 0.5 s at ~0.9 m/s ≈ 0.45 m advance from origin. Wall-clock-driven so
    # tolerances are loose.
    assert 0.20 < x_m < 0.55

    mock.enable_auto_tick(False)


def test_mock_determinism_with_explicit_ticks(sim_module, mock):
    """Explicit tick(dt) loops produce identical results across runs."""

    def run_once():
        _configure(sim_module, mock, start_x=10.0, start_y=10.0, start_theta=0.1)
        mock.set_motor_command(0, 75)
        mock.set_motor_command(1, 50)
        _tick(mock, 1.5)
        return mock.read_odometry()

    a = run_once()
    b = run_once()
    assert a == b
