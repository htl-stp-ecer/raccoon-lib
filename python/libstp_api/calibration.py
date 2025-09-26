from __future__ import annotations
import math
from typing import Mapping, Any

# Import value types from the compiled libstp if available.
try:
    from libstp.foundation import MotorCalibration, PidGains, Feedforward, Deadzone
except Exception:  # pragma: no cover - dev fallback
    # Minimal dummies so editors work without the native module.
    class PidGains:  # type: ignore
        kp: float; ki: float; kd: float
        def __init__(self):
            self.kp = 1.0; self.ki = 0.0; self.kd = 0.0

    class Feedforward:  # type: ignore
        kS: float; kV: float; kA: float
        def __init__(self):
            self.kS = 0.0; self.kV = 0.0; self.kA = 0.0

    class Deadzone:  # type: ignore
        enable: bool; zero_window_percent: float; start_percent: float; release_percent: float
        def __init__(self):
            self.enable = True; self.zero_window_percent = 10.0; self.start_percent = 40.0; self.release_percent = 15.0

    class MotorCalibration:  # type: ignore
        def __init__(self):
            self.ff = Feedforward(); self.pid = PidGains(); self.deadzone = Deadzone()
            self.max_percent_output = 100.0
            self.invert_meas = False; self.invert_cmd = False
            self.vel_lpf_alpha = 0.2
            self.ticks_to_rad = 2.0 * math.pi / 1440.0


def build_motor_calibration(motor_cfg: Mapping[str, Any]) -> MotorCalibration:
    pid_cfg = motor_cfg.get("pid", {})
    pid = PidGains()
    pid.kp = float(pid_cfg.get("kp", 1.0))
    pid.ki = float(pid_cfg.get("ki", 0.0))
    pid.kd = float(pid_cfg.get("kd", 0.0))

    ff_cfg = motor_cfg.get("feedforward", {})
    ff = Feedforward()
    ff.kS = float(ff_cfg.get("ks", 0.0))
    ff.kV = float(ff_cfg.get("kv", 0.0))
    ff.kA = float(ff_cfg.get("ka", 0.0))

    dz_cfg = motor_cfg.get("deadzone", {})
    dz = Deadzone()
    dz.enable = bool(dz_cfg.get("enable", True))
    dz.zero_window_percent = float(dz_cfg.get("zero_window_percent", 10.0))
    dz.start_percent = float(dz_cfg.get("start_percent", 40.0))
    dz.release_percent = float(dz_cfg.get("release_percent", 15.0))

    cal = MotorCalibration()
    cal.ff = ff
    cal.pid = pid
    cal.deadzone = dz
    cal.max_percent_output = float(motor_cfg.get("max_output_percent", 100.0))
    cal.invert_meas = bool(motor_cfg.get("invert_measurement", False))
    cal.invert_cmd = bool(motor_cfg.get("invert_command", False))
    cal.vel_lpf_alpha = float(motor_cfg.get("velocity_filter_alpha", 0.2))

    tpr = float(motor_cfg.get("ticks_per_revolution", 1440))
    gear_ratio = float(motor_cfg.get("gear_ratio", 1.0))
    cal.ticks_to_rad = (2.0 * math.pi) / (tpr * gear_ratio)

    return cal