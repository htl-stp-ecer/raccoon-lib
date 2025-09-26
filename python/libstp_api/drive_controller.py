from __future__ import annotations

from typing import List, Mapping, Any, Optional

from libstp.drive import Drive, ChassisVel, MotionLimits, WheelLimits

from .calibration import build_motor_calibration
from .config import RobotConfig, load_config
from .kinematics import build_kinematics
from .types import ChassisCommand, UpdateDt


class EasyDrive:
    def __init__(self, cfg: RobotConfig, defs):
        self.cfg = cfg
        self.motors: List[MotorLike] = motors if motors is not None else build_mock_motors(cfg)

        # Build dependencies
        kin = build_kinematics(cfg)
        calibrations = self._build_calibrations()

        # Compose the drive
        self._drive = Drive(kin, self.motors, calibrations)
        self._apply_limits()

    # --- construction helpers -------------------------------------------------

    @classmethod
    def from_config(cls, source: str | Mapping[str, Any], motors: Optional[List[MotorLike]] = None) -> "EasyDrive":
        return cls(load_config(source), motors)

    def _build_calibrations(self):
        cals = []
        names = motor_names_for(self.cfg)
        defs = self.cfg.definitions
        for name in names:
            mdef = defs.get(name, {})
            cals.append(build_motor_calibration(mdef))
        return cals

    def _apply_limits(self) -> None:
        lim = self.cfg.limits
        ch = MotionLimits()
        ch.max_v = lim.chassis.max_velocity
        ch.max_omega = lim.chassis.max_angular_velocity
        self._drive.set_chassis_limits(ch)

        wh = WheelLimits()
        wh.max_w = lim.wheels.max_angular_velocity
        wh.max_w_dot = lim.wheels.max_angular_acceleration
        self._drive.set_wheel_limits(wh)

    # --- runtime API ----------------------------------------------------------

    def move(self, cmd: ChassisCommand | None = None, *, vx: float = 0.0, vy: float = 0.0, w: float = 0.0) -> None:
        """Command the robot to move.

        Prefer passing a ``ChassisCommand`` value object; keyword args are
        supported for ergonomics.
        """
        if cmd is None:
            cmd = ChassisCommand(vx=vx, vy=vy, w=w)
        vel = ChassisVel()
        vel.vx, vel.vy, vel.w = cmd.vx, cmd.vy, cmd.w
        self._drive.set_velocity(vel)

    def update(self, dt: UpdateDt | float) -> Optional[object]:
        secs = dt.seconds if isinstance(dt, UpdateDt) else float(dt)
        return self._drive.update(secs)

    def stop(self, hard: bool = False) -> None:
        self._drive.stop(bool(hard))

    def wheel_count(self) -> int:
        return self._drive.wheel_count()
