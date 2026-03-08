"""
Drive telemetry collection for PID tuning.

Runs test drives at specified distances and speeds, collecting per-cycle
telemetry (commanded vs measured vs predicted position/velocity, PID outputs,
saturation state) into CSV files for offline analysis.
"""
import asyncio
import csv
import math
import os
from typing import TYPE_CHECKING

from libstp.motion import (
    LinearMotion,
    LinearMotionConfig,
    LinearMotionTelemetry,
    LinearAxis,
)

from .. import Step
from ..annotation import dsl_step

if TYPE_CHECKING:
    from libstp.robot.api import GenericRobot

CSV_HEADER = [
    "time_s",
    "dt",
    "target_m",
    "position_m",
    "setpoint_position_m",
    "setpoint_velocity_mps",
    "cross_track_m",
    "distance_error_m",
    "actual_error_m",
    "yaw_error_rad",
    "filtered_velocity_mps",
    "cmd_vx_mps",
    "cmd_vy_mps",
    "cmd_wz_radps",
    "pid_primary_raw",
    "pid_cross_raw",
    "pid_heading_raw",
    "heading_rad",
    "speed_scale",
    "heading_scale",
    "saturated",
]


def _telemetry_row(t: LinearMotionTelemetry) -> list:
    return [
        f"{t.time_s:.4f}",
        f"{t.dt:.4f}",
        f"{t.target_m:.4f}",
        f"{t.position_m:.4f}",
        f"{t.setpoint_position_m:.4f}",
        f"{t.setpoint_velocity_mps:.4f}",
        f"{t.cross_track_m:.4f}",
        f"{t.distance_error_m:.4f}",
        f"{t.actual_error_m:.4f}",
        f"{t.yaw_error_rad:.4f}",
        f"{t.filtered_velocity_mps:.4f}",
        f"{t.cmd_vx_mps:.4f}",
        f"{t.cmd_vy_mps:.4f}",
        f"{t.cmd_wz_radps:.4f}",
        f"{t.pid_primary_raw:.4f}",
        f"{t.pid_cross_raw:.4f}",
        f"{t.pid_heading_raw:.4f}",
        f"{t.heading_rad:.4f}",
        f"{t.speed_scale:.4f}",
        f"{t.heading_scale:.4f}",
        "1" if t.saturated else "0",
    ]


def _write_csv(path: str, telemetry: list[LinearMotionTelemetry]) -> None:
    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(CSV_HEADER)
        for t in telemetry:
            writer.writerow(_telemetry_row(t))


@dsl_step(tags=["motion", "calibration", "drive"])
class TuneDrive(Step):
    """Run test drives at various distances and speeds, saving telemetry to CSV.

    Executes every combination of (distance, speed) as a real ``LinearMotion``
    drive, collecting per-cycle telemetry at 50 Hz. Each run produces a CSV
    file containing columns such as ``time_s``, ``position_m``,
    ``setpoint_position_m``, ``setpoint_velocity_mps``, ``distance_error_m``,
    ``actual_error_m``, ``filtered_velocity_mps``, ``cmd_vx_mps``,
    ``pid_primary_raw``, ``heading_rad``, ``saturated``, and more (see
    ``CSV_HEADER`` in the module source for the full list).

    The CSV files are intended for offline analysis -- plot position vs.
    setpoint to check tracking, examine PID outputs for saturation, compare
    overshoot across speeds, etc. This is a diagnostic/tuning tool used
    during robot setup, not during competition runs.

    The robot must have enough clear space to drive the longest requested
    distance.

    Args:
        distances_cm: List of distances to test, in centimeters. Negative
            values drive in reverse. Default ``[10, 25, 50, 100]``.
        speeds: List of speed scales to test (0.0--1.0). Each distance is
            driven at each speed. Default ``[0.3, 0.6, 1.0]``.
        csv_dir: Directory where CSV files are written. Created
            automatically if it does not exist. Default
            ``"/tmp/drive_telemetry"``.
        axis: Drive axis to test: ``"forward"`` or ``"lateral"``. Default
            ``"forward"``.
        settle_time: Seconds to wait between runs for the robot to come to
            rest. Default 1.5.
        timeout: Maximum seconds per run before the drive is aborted.
            Default 15.0.

    Example::

        from libstp.step.motion import tune_drive

        # Quick test at a single distance and speed
        tune_drive(
            distances_cm=[50],
            speeds=[0.5],
            csv_dir="/tmp/quick_test",
        )

        # Full sweep for forward axis tuning
        tune_drive(
            distances_cm=[10, 25, 50, 100],
            speeds=[0.3, 0.6, 1.0],
        )

        # Lateral axis characterization
        tune_drive(
            distances_cm=[20, 40],
            speeds=[0.3, 0.6],
            axis="lateral",
        )
    """

    def __init__(
        self,
        distances_cm: list[float] = None,
        speeds: list[float] = None,
        csv_dir: str = "/tmp/drive_telemetry",
        axis: str = "forward",
        settle_time: float = 1.5,
        timeout: float = 15.0,
    ):
        super().__init__()
        if distances_cm is None:
            distances_cm = [10, 25, 50, 100]
        if speeds is None:
            speeds = [0.3, 0.6, 1.0]
        self.distances_cm = distances_cm
        self.speeds = speeds
        self.csv_dir = csv_dir
        self.axis = (
            LinearAxis.Lateral
            if isinstance(axis, str) and axis.lower().startswith("l")
            else LinearAxis.Forward
            if isinstance(axis, str)
            else axis
        )
        self.settle_time = settle_time
        self.timeout = timeout

    def _generate_signature(self) -> str:
        return (
            f"TuneDrive(distances={self.distances_cm}, "
            f"speeds={self.speeds})"
        )

    async def _execute_step(self, robot: "GenericRobot") -> None:
        os.makedirs(self.csv_dir, exist_ok=True)

        axis_name = "fwd" if self.axis == LinearAxis.Forward else "lat"
        total = len(self.distances_cm) * len(self.speeds)
        run_idx = 0

        self.info("=" * 60)
        self.info("  DRIVE TELEMETRY COLLECTION")
        self.info(f"  {total} runs, output: {self.csv_dir}")
        self.info("=" * 60)

        for dist_cm in self.distances_cm:
            for speed in self.speeds:
                run_idx += 1
                dist_m = dist_cm / 100.0

                self.info(
                    f"\n--- Run {run_idx}/{total}: "
                    f"{dist_cm:.0f} cm @ scale={speed:.2f} ({axis_name}) ---"
                )

                # Run the drive
                telemetry = await self._run_drive(robot, dist_m, speed)

                # Save CSV
                sign = "pos" if dist_cm >= 0 else "neg"
                fname = f"{axis_name}_{sign}_{abs(dist_cm):.0f}cm_{speed:.2f}scale.csv"
                csv_path = os.path.join(self.csv_dir, fname)
                _write_csv(csv_path, telemetry)

                # Summary
                if telemetry:
                    final = telemetry[-1]
                    peak_vel = max(abs(t.filtered_velocity_mps) for t in telemetry)
                    peak_cmd = max(abs(t.cmd_vx_mps if self.axis == LinearAxis.Forward else t.cmd_vy_mps) for t in telemetry)
                    self.info(f"  Duration:    {final.time_s:.2f} s  ({len(telemetry)} samples)")
                    self.info(f"  Final pos:   {final.position_m:.4f} m  (target: {dist_m:.4f} m)")
                    self.info(f"  Final error: {final.actual_error_m:.4f} m")
                    self.info(f"  Peak vel:    {peak_vel:.3f} m/s  (cmd: {peak_cmd:.3f})")
                    self.info(f"  Cross-track: {final.cross_track_m:.4f} m")
                    self.info(f"  Heading err: {math.degrees(final.yaw_error_rad):.1f} deg")
                    self.info(f"  Saved: {csv_path}")
                else:
                    self.info("  WARNING: No telemetry collected!")

                # Settle between runs
                robot.drive.hard_stop()
                await asyncio.sleep(self.settle_time)

        self.info("\n" + "=" * 60)
        self.info(f"  Done. {total} CSV files in {self.csv_dir}")
        self.info("=" * 60)

    async def _run_drive(
        self, robot: "GenericRobot", distance_m: float, max_speed: float
    ) -> list[LinearMotionTelemetry]:
        config = LinearMotionConfig()
        config.axis = self.axis
        config.distance_m = distance_m
        config.speed_scale = max_speed

        motion = LinearMotion(
            robot.drive, robot.odometry, robot.motion_pid_config, config
        )
        motion.start()

        rate = 1 / 50  # 50 Hz for higher resolution telemetry
        t0 = asyncio.get_event_loop().time()
        last = t0 - rate

        while not motion.is_finished():
            now = asyncio.get_event_loop().time()
            if now - t0 > self.timeout:
                self.info(f"  TIMEOUT after {self.timeout:.1f}s")
                break
            dt = max(now - last, 0.0)
            last = now
            if dt < 1e-4:
                await asyncio.sleep(rate)
                continue
            motion.update(dt)
            await asyncio.sleep(rate)

        robot.drive.hard_stop()
        return list(motion.get_telemetry())


