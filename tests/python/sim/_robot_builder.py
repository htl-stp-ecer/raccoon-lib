from __future__ import annotations

from types import SimpleNamespace


def _apply_motor_calibrations(motors, cfg) -> None:
    from raccoon.foundation import MotorCalibration

    per_port = getattr(cfg, "motor_calibration_by_port", {}) or {}
    for motor in motors:
        ticks_to_rad, vel_lpf_alpha = per_port.get(
            motor.port,
            (getattr(cfg, "ticks_to_rad", None), 1.0),
        )
        if ticks_to_rad is None:
            continue
        motor.set_calibration(MotorCalibration(ticks_to_rad, vel_lpf_alpha))


def get_config(name: str):
    from dataclasses import replace

    from raccoon.testing.sim import SimRobotConfig

    if name == "drumbot":
        from raccoon.testing.robot_configs import DRUMBOT

        return replace(DRUMBOT, bemf_noise_stddev=0.0)
    if name == "packingbot":
        from raccoon.testing.robot_configs import PACKINGBOT

        return replace(PACKINGBOT, bemf_noise_stddev=0.0)
    return SimRobotConfig(
        wheel_radius_m=0.03,
        track_width_m=0.15,
        wheelbase_m=0.15,
        left_motor_port=0,
        right_motor_port=1,
        max_wheel_velocity_rad_s=30.0,
        motor_time_constant_sec=0.02,
    )


def build_robot(cfg, *, enable_localization: bool = True):
    """Build a HAL robot whose kinematics match the given SimRobotConfig.

    Phase 4 of the absolute-motion plan made ``robot.localization`` a hard
    requirement for any motion step (motions need an absolute
    ``target_heading_rad`` and read it from
    ``robot.localization.get_pose().heading``). The default therefore flips
    to ``True``; pass ``enable_localization=False`` only when the test
    explicitly checks the ``robot.localization is None`` path.
    """
    from raccoon.drive import ChassisVelocityControlConfig, Drive
    from raccoon.hal import IMU, Motor
    from raccoon.hal import platform as _platform
    from raccoon.motion import AxisConstraints, UnifiedMotionPidConfig

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
        _apply_motor_calibrations((fl, fr, bl, br), cfg)
        kin.set_max_wheel_speed(cfg.max_wheel_velocity_rad_s)
        refs = (fl, fr, bl, br, imu)
    else:
        from raccoon.kinematics_differential import DifferentialKinematics

        left = Motor(cfg.left_motor_port, cfg.left_motor_inverted)
        right = Motor(cfg.right_motor_port, cfg.right_motor_inverted)
        kin = DifferentialKinematics(left, right, cfg.track_width_m, cfg.wheel_radius_m)
        _apply_motor_calibrations((left, right), cfg)
        kin.set_max_wheel_speed(cfg.max_wheel_velocity_rad_s)
        refs = (left, right, imu)

    drive_obj = Drive(kin, ChassisVelocityControlConfig(), imu)
    odom = _platform.Platform.create_odometry(kin)

    pid_cfg = UnifiedMotionPidConfig()
    pid_cfg.linear = AxisConstraints(0.8, 1.5, 1.5)
    pid_cfg.lateral = AxisConstraints(0.5, 1.0, 1.0)
    pid_cfg.angular = AxisConstraints(6.0, 12.0, 12.0)

    localization = None
    if enable_localization:
        from raccoon.localization import Localization, LocalizationConfig

        # Fast tick (5 ms) keeps the worker responsive during short test
        # scenarios; the production default is 10 ms.
        localization = Localization(odom, LocalizationConfig(tick_period_ms=5))

    # Checkpoint-aware timing steps (wait_for_checkpoint / do_until_checkpoint)
    # call robot.synchronizer. The real GenericRobot lazily creates one; the
    # fake sim robot must provide it too or those steps raise AttributeError.
    # Under the headless sim there is no match clock / drum-bot to sync against,
    # so checkpoints are skipped (LIBSTP_NO_CHECKPOINTS=1) and this synchronizer
    # only needs to exist — its wait returns immediately in that mode.
    from raccoon.timing.synchronizer import Synchronizer

    return SimpleNamespace(
        drive=drive_obj,
        odometry=odom,
        kinematics=kin,
        motion_pid_config=pid_cfg,
        localization=localization,
        synchronizer=Synchronizer(),
        _refs=refs,
    )
