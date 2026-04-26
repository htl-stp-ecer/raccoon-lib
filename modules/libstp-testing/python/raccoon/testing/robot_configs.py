"""Pre-built SimRobotConfig instances derived from real competition robots.

These configs capture the actual geometry, encoder calibration, and estimated
motor dynamics from physical robots so that sim-based tests exercise
algorithms under realistic conditions rather than idealized defaults.

Usage::

    from raccoon.testing.robot_configs import DRUMBOT, PACKINGBOT

    with use_scene("empty_table.ftmap", robot=DRUMBOT, start=(50, 50, 0)):
        ...
"""
from __future__ import annotations

from .sim import SimRobotConfig

# ---------------------------------------------------------------------------
# Drumbot — ECER 2026 differential-drive robot
# ---------------------------------------------------------------------------
# Source: Ecer2026/drumbot/config/{robot,motors,hardware}.yml
#
# Physical: 13×19 cm, rotation center at (6.5, 5.5) from origin
# Drivetrain: differential, wheel_radius=34.5mm, wheelbase=160mm
# Motors: port 0 (left, not inverted), port 1 (right, inverted)
# Encoder cal: left=1.955e-5, right=1.574e-5 rad/tick (avg ≈ 1.76e-5)
# Motion limits: v_max=0.237 m/s, accel=0.280 m/s², decel=2.053 m/s²
#   → decel/accel = 7.3× suggests significant mechanical braking/friction
#
# Drag estimates derived from the decel/accel asymmetry:
#   - viscous_drag: 0.8 /s — moderate bearing + back-EMF drag
#   - coulomb_friction: 1.5 rad/s² — gearbox + rubber-on-PVC friction

DRUMBOT = SimRobotConfig(
    width_cm=13.0,
    length_cm=19.0,
    rotation_center_forward_cm=5.5,
    rotation_center_strafe_cm=0.0,
    wheel_radius_m=0.0345,
    track_width_m=0.16,
    wheelbase_m=0.16,
    left_motor_port=0,
    right_motor_port=1,
    left_motor_inverted=False,
    right_motor_inverted=True,
    max_wheel_velocity_rad_s=30.0,
    motor_time_constant_sec=0.06,
    # ticks_to_rad intentionally left at the default (2π/1440) because the
    # mock Motor HAL uses the default MotorCalibration, so the sim's BEMF
    # encoding/decoding must match.  Real encoder resolution only matters
    # on physical hardware.
    viscous_drag_coeff=0.8,
    coulomb_friction_rad_s2=1.5,
    bemf_noise_stddev=2.0,        # ~2 BEMF units of encoder noise
)

# ---------------------------------------------------------------------------
# Packingbot — ECER 2026 mecanum-drive robot
# ---------------------------------------------------------------------------
# Source: Ecer2026/packingbot/config/{robot,motors,hardware}.yml
#
# Physical: 23.5×29.6 cm, rotation center at (11.75, 18.5) from origin
# Drivetrain: mecanum, wheel_radius=37.5mm, track_width=200mm, wheelbase=125mm
# Motors: FL=port 1, FR=port 0 (inv), RL=port 2, RL=port 3 (inv)
# Encoder cal: FL=1.641e-5, FR=1.582e-5, RL=1.653e-5, RR=1.633e-5 (avg ≈ 1.63e-5)
# Motion limits: v_max=0.214 m/s, accel=0.940 m/s², decel=1.664 m/s²
#   → decel/accel = 1.77× — more symmetric than drumbot
#
# Drag estimates:
#   - viscous_drag: 1.2 /s — mecanum rollers add extra bearing drag
#   - coulomb_friction: 2.0 rad/s² — roller bearings + heavier robot

PACKINGBOT = SimRobotConfig(
    width_cm=23.5,
    length_cm=29.6,
    rotation_center_forward_cm=18.5,
    rotation_center_strafe_cm=0.0,
    wheel_radius_m=0.0375,
    track_width_m=0.20,
    wheelbase_m=0.125,
    drivetrain="mecanum",
    fl_motor_port=1,
    fr_motor_port=0,
    bl_motor_port=2,
    br_motor_port=3,
    fl_motor_inverted=False,
    fr_motor_inverted=True,
    bl_motor_inverted=False,
    br_motor_inverted=True,
    max_wheel_velocity_rad_s=30.0,
    motor_time_constant_sec=0.06,
    # ticks_to_rad intentionally left at the default (2π/1440) — see DRUMBOT
    # comment above.
    viscous_drag_coeff=1.2,
    coulomb_friction_rad_s2=2.0,
    bemf_noise_stddev=2.0,        # ~2 BEMF units of encoder noise
)


__all__ = ["DRUMBOT", "PACKINGBOT"]
