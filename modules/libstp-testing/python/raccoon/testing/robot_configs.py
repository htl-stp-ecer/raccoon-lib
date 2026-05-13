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


def _max_wheel_speed_with_headroom(linear_max_mps: float, wheel_radius_m: float) -> float:
    """Keep sim saturation slightly above the motion cap without being idealized."""
    return (linear_max_mps / wheel_radius_m) * 1.5


def _avg_ticks_to_rad(*values: float) -> float:
    return sum(values) / len(values)


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

_DRUMBOT_LEFT_TICKS_TO_RAD = 1.955e-5
_DRUMBOT_RIGHT_TICKS_TO_RAD = 1.574e-5

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
    max_wheel_velocity_rad_s=_max_wheel_speed_with_headroom(0.237, 0.0345),
    motor_time_constant_sec=0.06,
    ticks_to_rad=_avg_ticks_to_rad(_DRUMBOT_LEFT_TICKS_TO_RAD, _DRUMBOT_RIGHT_TICKS_TO_RAD),
    motor_calibration_by_port={
        0: (_DRUMBOT_LEFT_TICKS_TO_RAD, 1.0),
        1: (_DRUMBOT_RIGHT_TICKS_TO_RAD, 1.0),
    },
    viscous_drag_coeff=0.8,
    coulomb_friction_rad_s2=1.5,
    bemf_noise_stddev=2.0,  # ~2 BEMF units of encoder noise
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

_PACKINGBOT_FL_TICKS_TO_RAD = 1.641e-5
_PACKINGBOT_FR_TICKS_TO_RAD = 1.582e-5
_PACKINGBOT_BL_TICKS_TO_RAD = 1.653e-5
_PACKINGBOT_BR_TICKS_TO_RAD = 1.633e-5

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
    max_wheel_velocity_rad_s=_max_wheel_speed_with_headroom(0.2139, 0.0375),
    motor_time_constant_sec=0.06,
    ticks_to_rad=_avg_ticks_to_rad(
        _PACKINGBOT_FL_TICKS_TO_RAD,
        _PACKINGBOT_FR_TICKS_TO_RAD,
        _PACKINGBOT_BL_TICKS_TO_RAD,
        _PACKINGBOT_BR_TICKS_TO_RAD,
    ),
    motor_calibration_by_port={
        0: (_PACKINGBOT_FR_TICKS_TO_RAD, 1.0),
        1: (_PACKINGBOT_FL_TICKS_TO_RAD, 1.0),
        2: (_PACKINGBOT_BL_TICKS_TO_RAD, 1.0),
        3: (_PACKINGBOT_BR_TICKS_TO_RAD, 1.0),
    },
    viscous_drag_coeff=1.2,
    coulomb_friction_rad_s2=2.0,
    bemf_noise_stddev=2.0,  # ~2 BEMF units of encoder noise
)


# ---------------------------------------------------------------------------
# Clawbot — alternate ECER 2026 mecanum robot geometry
# ---------------------------------------------------------------------------
# Source: Ecer2026/clawbot/config/{robot,motors,hardware}.yml
#
# Shares the same drivetrain and velocity config as packingbot, but has a
# shorter body and a less rear-biased rotation center.

CLAWBOT = SimRobotConfig(
    width_cm=23.5,
    length_cm=26.0,
    rotation_center_forward_cm=13.5,
    rotation_center_strafe_cm=0.0,
    wheel_radius_m=0.0375,
    track_width_m=0.20,
    wheelbase_m=0.125,
    drivetrain="mecanum",
    fl_motor_port=0,
    fr_motor_port=1,
    bl_motor_port=2,
    br_motor_port=3,
    fl_motor_inverted=False,
    fr_motor_inverted=True,
    bl_motor_inverted=False,
    br_motor_inverted=True,
    max_wheel_velocity_rad_s=_max_wheel_speed_with_headroom(0.2139, 0.0375),
    motor_time_constant_sec=0.06,
    ticks_to_rad=_avg_ticks_to_rad(
        1.9077897309246966e-05,
        1.8414557269137787e-05,
        1.7478827093512414e-05,
        1.74985158939979e-05,
    ),
    motor_calibration_by_port={
        0: (1.9077897309246966e-05, 1.0),
        1: (1.8414557269137787e-05, 1.0),
        2: (1.7478827093512414e-05, 1.0),
        3: (1.74985158939979e-05, 1.0),
    },
    viscous_drag_coeff=1.2,
    coulomb_friction_rad_s2=2.0,
    bemf_noise_stddev=2.0,
)


__all__ = ["DRUMBOT", "PACKINGBOT", "CLAWBOT"]
