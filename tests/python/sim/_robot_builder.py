from __future__ import annotations

from types import SimpleNamespace


def get_config(name: str):
    from raccoon.testing.sim import SimRobotConfig

    if name == "drumbot":
        from raccoon.testing.robot_configs import DRUMBOT

        return DRUMBOT
    if name == "packingbot":
        from raccoon.testing.robot_configs import PACKINGBOT

        return PACKINGBOT
    return SimRobotConfig(
        wheel_radius_m=0.03,
        track_width_m=0.15,
        wheelbase_m=0.15,
        left_motor_port=0,
        right_motor_port=1,
        max_wheel_velocity_rad_s=30.0,
        motor_time_constant_sec=0.02,
    )


def build_robot(cfg):
    """Build a HAL robot whose kinematics match the given SimRobotConfig."""
    from raccoon.drive import ChassisVelocityControlConfig, Drive
    from raccoon.hal import IMU, Motor, OdometryBridge
    from raccoon.motion import AxisConstraints, UnifiedMotionPidConfig
    from raccoon.odometry_stm32 import Stm32Odometry, Stm32OdometryConfig

    imu = IMU()

    if getattr(cfg, "drivetrain", "diff") == "mecanum":
        from raccoon.kinematics_mecanum import MecanumKinematics

        fl = Motor(cfg.fl_motor_port, cfg.fl_motor_inverted)
        fr = Motor(cfg.fr_motor_port, cfg.fr_motor_inverted)
        bl = Motor(cfg.bl_motor_port, cfg.bl_motor_inverted)
        br = Motor(cfg.br_motor_port, cfg.br_motor_inverted)
        kin = MecanumKinematics(
            front_left_motor=fl,
            front_right_motor=fr,
            back_left_motor=bl,
            back_right_motor=br,
            wheelbase=cfg.wheelbase_m,
            track_width=cfg.track_width_m,
            wheel_radius=cfg.wheel_radius_m,
        )
        refs = (fl, fr, bl, br, imu)
    else:
        from raccoon.kinematics_differential import DifferentialKinematics

        left = Motor(cfg.left_motor_port, cfg.left_motor_inverted)
        right = Motor(cfg.right_motor_port, cfg.right_motor_inverted)
        kin = DifferentialKinematics(left, right, cfg.track_width_m, cfg.wheel_radius_m)
        refs = (left, right, imu)

    drive_obj = Drive(kin, ChassisVelocityControlConfig(), imu)
    bridge = OdometryBridge()
    odom = Stm32Odometry(imu=imu, kinematics=kin, bridge=bridge, config=Stm32OdometryConfig())

    pid_cfg = UnifiedMotionPidConfig()
    pid_cfg.linear = AxisConstraints(0.8, 1.5, 1.5)
    pid_cfg.lateral = AxisConstraints(0.5, 1.0, 1.0)
    pid_cfg.angular = AxisConstraints(6.0, 12.0, 12.0)

    return SimpleNamespace(
        drive=drive_obj,
        odometry=odom,
        kinematics=kin,
        motion_pid_config=pid_cfg,
        _refs=(*refs, bridge),
    )
