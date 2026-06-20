"""v2 multi-layer ftmap: parsing, layer-scoped queries, and ramp transitions.

The v2 format stacks the table into `layers` (each a 2D plane at a `zCm`
elevation) connected by `transitions` (ramp seams). The sim tracks which plane
the robot is on and resolves line sensors + collision against that plane, so a
mission running on the raised ramp sees the ramp's lines — not the floor's.
"""

from __future__ import annotations

import pytest

# Synthetic two-layer table: a ground line at x=50, a ramp line at x=150, and a
# seam at x=100 connecting them. Full-height vertical lines so the on-disk
# top-down Y flip is symmetric. (cm; on-disk Y is top-down, flipped on ingest.)
V2_RAMP = """
{
  "format": "flowchart-table-map",
  "version": 2,
  "table": {"widthCm": 200, "heightCm": 100},
  "layers": [
    {"id": "ground", "name": "Ground", "zCm": 0, "lines": [
      {"kind": "line", "startX": 50, "startY": 0, "endX": 50, "endY": 100, "widthCm": 5}
    ]},
    {"id": "ramp", "name": "Ramp", "zCm": 12, "lines": [
      {"kind": "line", "startX": 150, "startY": 0, "endX": 150, "endY": 100, "widthCm": 5}
    ]}
  ],
  "transitions": [
    {"id": "seam", "fromLayer": "ground", "toLayer": "ramp",
     "from": {"startX": 100, "startY": 0, "endX": 100, "endY": 100},
     "to":   {"startX": 100, "startY": 0, "endX": 100, "endY": 100},
     "bidirectional": true, "widthCm": 40}
  ]
}
"""


@pytest.fixture
def ramp_map(sim_module):
    m = sim_module.WorldMap()
    m.parse_ftmap(V2_RAMP)
    return m


def test_v2_layers_and_transitions_parsed(ramp_map):
    assert ramp_map.layer_count == 2
    assert ramp_map.transition_count == 1
    assert ramp_map.layer_index("ground") == 0
    assert ramp_map.layer_index("ramp") == 1
    assert ramp_map.layer_index("nope") == -1
    assert ramp_map.table_width_cm == 200.0


def test_layer_scoped_line_queries(ramp_map):
    # ground line at x=50 lives only on layer 0; ramp line at x=150 only on 1.
    assert ramp_map.is_on_line_layer(50.0, 50.0, 0) is True
    assert ramp_map.is_on_line_layer(50.0, 50.0, 1) is False
    assert ramp_map.is_on_line_layer(150.0, 50.0, 1) is True
    assert ramp_map.is_on_line_layer(150.0, 50.0, 0) is False


def test_legacy_no_arg_queries_use_ground(ramp_map):
    # Back-compat: the no-arg query operates on the ground layer.
    assert ramp_map.is_on_line(50.0, 50.0) is True
    assert ramp_map.is_on_line(150.0, 50.0) is False


def test_v1_still_loads_as_single_ground_layer(sim_module):
    m = sim_module.WorldMap()
    m.parse_ftmap(
        '{"format":"flowchart-table-map","version":1,'
        '"table":{"widthCm":200,"heightCm":100},'
        '"lines":[{"kind":"line","startX":50,"startY":0,"endX":50,"endY":100,"widthCm":5}]}'
    )
    assert m.layer_count == 1
    assert m.transition_count == 0
    assert m.layer_index("ground") == 0
    assert m.is_on_line(50.0, 50.0) is True


def _diff_world(sim_module):
    w = sim_module.SimWorld()
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
    w.configure(robot, motors)
    return w


def test_robot_switches_to_ramp_layer_when_crossing_seam(sim_module, ramp_map):
    w = _diff_world(sim_module)
    w.set_map(ramp_map)
    w.set_pose(sim_module.Pose2D(30.0, 50.0, 0.0))  # ground, facing +x
    w.attach_line_sensor(analog_port=2, forward_cm=0.0, strafe_cm=0.0, name="center")

    # Start on the ground plane, away from any line.
    assert w.current_layer == 0
    assert w.read_analog(2) == 0  # white

    # Drive +x across the seam (x=100) toward the ramp line (x=150).
    w.set_motor_command(0, 100)
    w.set_motor_command(1, 100)
    saw_ground_line = False
    for _ in range(400):
        w.tick(0.05)
        x = w.pose.x
        if 47.0 <= x <= 53.0 and w.current_layer == 0 and w.read_analog(2) == 1023:
            saw_ground_line = True
        if x >= 150.0:
            break

    # Crossed the seam → now on the ramp plane.
    assert w.current_layer == 1
    # The ramp line at x=150 (which does NOT exist on the ground plane) is now
    # detected — proving sensors resolve against the active plane.
    assert w.pose.x >= 150.0
    assert w.read_analog(2) == 1023
    # And we did sense the ground line earlier while still on the ground plane.
    assert saw_ground_line


def test_bidirectional_transition_back_to_ground(sim_module, ramp_map):
    w = _diff_world(sim_module)
    w.set_map(ramp_map)
    # Start on the ramp side, drive -x back across the seam onto the ground.
    w.set_pose(sim_module.Pose2D(170.0, 50.0, 0.0))
    w.set_current_layer(1)  # pretend we're already up the ramp
    assert w.current_layer == 1
    w.set_motor_command(0, -100)
    w.set_motor_command(1, -100)
    for _ in range(400):
        w.tick(0.05)
        if w.pose.x <= 50.0:
            break
    assert w.current_layer == 0  # crossed back down to the ground plane
