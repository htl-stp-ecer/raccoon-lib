"""End-to-end tests for the sim Python bindings.

These exercise the same core behaviors as the C++ ``test_sim`` suite but
through the pybind11 API, proving the binding surface matches what teams will
actually use from pytest.
"""
from __future__ import annotations

import math
from pathlib import Path

import pytest

from conftest import SCENES_DIR, run  # type: ignore[import-not-found]


def test_module_loads(sim_module):
    assert sim_module.__doc__
    assert hasattr(sim_module, "SimWorld")
    assert hasattr(sim_module, "WorldMap")
    assert hasattr(sim_module, "Pose2D")


def test_pose2d_basic(sim_module):
    p = sim_module.Pose2D(10.0, 20.0, 0.0)
    assert p.x == 10.0
    assert p.y == 20.0
    assert p.theta == 0.0
    q = sim_module.Pose2D.from_degrees(0.0, 0.0, 90.0)
    assert math.isclose(q.theta, math.pi / 2, abs_tol=1e-5)
    assert math.isclose(q.theta_deg, 90.0, abs_tol=1e-3)


def test_worldmap_load_and_query(sim_module):
    m = sim_module.WorldMap()
    m.load_ftmap(str(SCENES_DIR / "single_line.ftmap"))
    assert m.table_width_cm == 200.0
    assert m.table_height_cm == 100.0
    assert len(m.lines()) == 1
    assert len(m.walls()) == 4  # 4 borders, no internal walls
    # The line is horizontal at y=50 (ftmap Y flipped from 50).
    assert m.is_on_black_line(100.0, 50.0) is True
    assert m.is_on_black_line(10.0, 10.0) is False


def test_worldmap_invalid_ftmap_raises(sim_module):
    m = sim_module.WorldMap()
    with pytest.raises(sim_module.FtmapParseError):
        m.parse_ftmap('{"format":"wrong","version":1,"table":{"widthCm":1,"heightCm":1},"lines":[]}')


def test_drive_forward_updates_pose(configured_world):
    w = configured_world
    w.set_motor_command(0, 100)
    w.set_motor_command(1, 100)
    run(w, 1.0)
    p = w.pose
    # 0.9 m/s × 1 s = 90 cm advance minus near-zero ramp (τ = 10 ms).
    assert 135.0 < p.x < 145.0  # starts at 50
    assert math.isclose(p.y, 50.0, abs_tol=0.01)
    assert math.isclose(p.theta, 0.0, abs_tol=0.01)


def test_turn_in_place(configured_world):
    w = configured_world
    w.set_motor_command(0, 100)
    w.set_motor_command(1, -100)
    run(w, 0.2)
    p = w.pose
    assert math.isclose(p.x, 50.0, abs_tol=0.5)
    assert math.isclose(p.y, 50.0, abs_tol=0.5)
    # Yaw rate = -12 rad/s steady state, 0.2 s → ≈ -2.4 rad.
    assert -3.0 < p.theta < -1.5


def test_wall_stops_forward_drive(sim_module, default_robot, default_motors):
    world = sim_module.SimWorld()
    world.configure(default_robot, default_motors)
    m = sim_module.WorldMap()
    m.set_table(200.0, 100.0)
    world.set_map(m)
    world.set_pose(sim_module.Pose2D(20.0, 50.0, 0.0))

    world.set_motor_command(0, 100)
    world.set_motor_command(1, 100)
    run(world, 5.0)

    p = world.pose
    # East border at x=200, robot half-length = 9 → stops near x ≈ 191.
    assert 170.0 < p.x < 192.0


def test_line_sensor_sampling(sim_module, default_robot, default_motors):
    world = sim_module.SimWorld()
    world.configure(default_robot, default_motors)
    m = sim_module.WorldMap()
    m.set_table(200.0, 100.0)
    seg = sim_module.MapSegment()
    seg.kind = sim_module.MapSegment.Kind.LINE
    seg.start_x = 100.0
    seg.start_y = 0.0
    seg.end_x = 100.0
    seg.end_y = 100.0
    seg.width_cm = 2.0
    m.add_segment(seg)
    world.set_map(m)

    world.attach_line_sensor(analog_port=2, forward_cm=0.0, strafe_cm=0.0, name="center")

    world.set_pose(sim_module.Pose2D(100.0, 50.0, 0.0))
    assert world.read_analog(2) == 0  # over the black line

    world.set_pose(sim_module.Pose2D(50.0, 50.0, 0.0))
    assert world.read_analog(2) == 1023  # white surface


def test_distance_sensor_reads_wall(sim_module, default_robot, default_motors):
    world = sim_module.SimWorld()
    world.configure(default_robot, default_motors)
    m = sim_module.WorldMap()
    m.set_table(200.0, 100.0)
    world.set_map(m)

    world.attach_distance_sensor(
        analog_port=3, forward_cm=0.0, strafe_cm=0.0,
        mount_angle_rad=0.0, max_range_cm=100.0,
    )

    # 15 cm away from east border — in the useful 10–45 cm range.
    world.set_pose(sim_module.Pose2D(185.0, 50.0, 0.0))
    v15 = world.read_analog(3)

    # 30 cm away — deeper in the useful range.
    world.set_pose(sim_module.Pose2D(170.0, 50.0, 0.0))
    v30 = world.read_analog(3)

    assert v15 is not None
    assert v30 is not None
    # Both in valid ADC range and the ET curve is non-trivial (not the floor).
    assert 0 <= v15 <= 1023
    assert 0 <= v30 <= 1023
    assert v15 != 275 and v30 != 275  # not the "no target" floor


def test_detach_sensor_returns_none(configured_world):
    w = configured_world
    w.attach_line_sensor(2, 0.0, 0.0)
    assert w.read_analog(2) is not None
    w.detach_sensor(2)
    assert w.read_analog(2) is None


def test_determinism_under_fixed_step(sim_module, default_robot, default_motors):
    def run_once():
        w = sim_module.SimWorld()
        w.configure(default_robot, default_motors)
        m = sim_module.WorldMap()
        m.set_table(400.0, 400.0)
        w.set_map(m)
        w.set_pose(sim_module.Pose2D(100.0, 100.0, 0.3))
        w.set_motor_command(0, 75)
        w.set_motor_command(1, 50)
        run(w, 1.5)
        return w.pose

    a = run_once()
    b = run_once()
    assert a.x == b.x
    assert a.y == b.y
    assert a.theta == b.theta


def test_mock_submodule_present(sim_module):
    """The mock platform submodule should exist when the wheel is built with
    DRIVER_BUNDLE=mock. If it's missing, we're running against a non-mock
    build and mock-HAL tests are skipped."""
    if not hasattr(sim_module, "mock"):
        pytest.skip("sim.mock not present — build with DRIVER_BUNDLE=mock")


def test_mock_configure_and_tick(sim_module):
    if not hasattr(sim_module, "mock"):
        pytest.skip("sim.mock not present")
    mock = sim_module.mock

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

    world_map = sim_module.WorldMap()
    world_map.load_ftmap(str(SCENES_DIR / "empty_table.ftmap"))

    start = sim_module.Pose2D(50.0, 50.0, 0.0)
    mock.configure(robot, motors, world_map, start)
    assert mock.has_sim() is True

    pose = mock.pose()
    assert math.isclose(pose.x, 50.0, abs_tol=1e-4)
    assert math.isclose(pose.y, 50.0, abs_tol=1e-4)

    # We can advance the mock sim directly.
    # Set motors through the SimWorld returned from mock namespace-less API
    # — we exposed tick() but setting motors still requires going through
    # the HAL or internal SimWorld. For this test just verify the tick path
    # works by attaching a line sensor and reading it.
    mock.attach_line_sensor(2, 0.0, 0.0, "center")
    mock.tick(0.01)  # advance once
    mock.detach()
    assert mock.has_sim() is False
