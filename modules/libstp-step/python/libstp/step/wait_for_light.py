"""
Automatic wait-for-light using Kalman-filtered flank detection.

Designed for a downward-facing light sensor (no shielding required).
A Kalman filter tracks the ambient baseline. When the start lamp turns on,
light reflects off the table surface and the sensor value drops sharply.
The step triggers when the raw reading falls below a configurable fraction
of the filtered baseline for several consecutive samples.

Based on: "Comprehensive Light-Start Methods in Botball" (Gosling et al.)
"""

import asyncio
import time

from typing import TYPE_CHECKING

from libstp.hal import AnalogSensor
from .annotation import dsl_step
from libstp.ui import UIStep
from libstp.ui.screens.wfl import WFLDetectScreen

if TYPE_CHECKING:
    from libstp.robot.api import GenericRobot


class _KalmanFilter:
    """1D Kalman filter for tracking a slowly-changing scalar baseline."""

    __slots__ = ("estimate", "error", "process_variance", "measurement_variance", "_initialized")

    def __init__(self, process_variance: float, measurement_variance: float) -> None:
        self.estimate: float = 0.0
        self.error: float = 1000.0
        self.process_variance = process_variance
        self.measurement_variance = measurement_variance
        self._initialized = False

    def update(self, measurement: float) -> float:
        if not self._initialized:
            self.estimate = measurement
            self.error = self.measurement_variance
            self._initialized = True
            return self.estimate

        # Predict
        predicted_error = self.error + self.process_variance

        # Update
        kalman_gain = predicted_error / (predicted_error + self.measurement_variance)
        self.estimate += kalman_gain * (measurement - self.estimate)
        self.error = (1 - kalman_gain) * predicted_error

        return self.estimate

    def set_process_variance(self, pv: float) -> None:
        self.process_variance = pv


@dsl_step(tags=["timing", "wait"])
class WaitForLight(UIStep):
    """Wait for the start lamp using automatic Kalman-filtered flank detection.

    Mount the light sensor facing downward with no shielding. The step
    establishes a stable baseline reading via a 1D Kalman filter during a
    short warm-up phase, then arms and polls for a sharp brightness
    increase (sensor value drop). When the raw reading falls below
    ``baseline * (1 - drop_fraction)`` for ``confirm_count`` consecutive
    samples, the step returns and the mission begins.

    The downward-facing mount reduces environmental noise by up to 76%
    compared to a horizontal mount (Gosling et al., 2023). No black tape
    or straw shielding is required.

    Prerequisites:
        An analog light sensor (LDR) connected to the Wombat and mounted
        facing the table surface. The start lamp should be positioned
        diagonally above the sensor.

    Args:
        sensor: The AnalogSensor instance for the light sensor.
        drop_fraction: Fraction the raw value must drop below the baseline
            to trigger. 0.15 means a 15% brightness increase triggers the
            start. Lower values are more sensitive (faster but riskier),
            higher values are safer but need a stronger signal.
        confirm_count: Number of consecutive triggering samples required
            before starting. At the default 200 Hz poll rate, 3 samples
            equals ~15 ms of confirmation — effectively instant while
            rejecting single-sample noise spikes.
        warmup_seconds: Duration in seconds to collect baseline samples
            before arming the detector.
        poll_interval: Seconds between sensor reads. 0.005 gives ~200 Hz.

    Example::

        from libstp.step.wait_for_light import wait_for_light

        # Default settings — works well for most setups
        wait_for_light(robot.defs.wait_for_light_sensor)

        # More sensitive (10% drop, 2 confirms) for weak lamp signal
        wait_for_light(robot.defs.wait_for_light_sensor, drop_fraction=0.10, confirm_count=2)
    """

    def __init__(
        self,
        sensor: AnalogSensor,
        drop_fraction: float = 0.15,
        confirm_count: int = 3,
        warmup_seconds: float = 1.0,
        poll_interval: float = 0.005,
    ) -> None:
        super().__init__()
        self._sensor = sensor
        self._drop_fraction = drop_fraction
        self._confirm_count = confirm_count
        self._warmup_seconds = warmup_seconds
        self._poll_interval = poll_interval

    def _generate_signature(self) -> str:
        port = getattr(self._sensor, "port", "?")
        return (
            f"WaitForLight(port={port}, "
            f"drop={self._drop_fraction:.0%}, "
            f"confirm={self._confirm_count})"
        )

    async def _execute_step(self, robot: "GenericRobot") -> None:
        sensor = self._sensor
        kf = _KalmanFilter(
            process_variance=0.01,
            measurement_variance=50.0,
        )

        screen = WFLDetectScreen()
        await self.display(screen)

        # --- Warm-up: establish baseline ---
        warmup_end = time.monotonic() + self._warmup_seconds
        self.info(f"WFL: warming up for {self._warmup_seconds}s...")
        last_ui = 0.0

        while time.monotonic() < warmup_end:
            raw = sensor.read()
            kf.update(raw)

            now = time.monotonic()
            if now - last_ui > 0.1:
                screen.status = "WARMING UP"
                screen.status_color = "amber"
                screen.raw_value = raw
                screen.baseline = kf.estimate
                screen.threshold = kf.estimate * (1.0 - self._drop_fraction)
                await screen.refresh()
                last_ui = now

            await asyncio.sleep(self._poll_interval)

        self.info(f"WFL: armed (baseline={kf.estimate:.0f})")

        # --- Armed: detect flank ---
        # Slow down Kalman adaptation so ambient drift is tracked over
        # minutes but a sudden lamp step-change (hundreds of units in one
        # sample) can never be absorbed. Only non-triggering samples are
        # fed into the filter so the lamp signal never contaminates the
        # baseline.
        kf.set_process_variance(0.001)

        consecutive = 0
        threshold_factor = 1.0 - self._drop_fraction

        screen.status = "ARMED"
        screen.status_color = "green"
        screen.baseline = kf.estimate
        screen.threshold = kf.estimate * threshold_factor
        await screen.refresh()

        while True:
            raw = sensor.read()
            trigger_threshold = kf.estimate * threshold_factor

            if raw < trigger_threshold:
                consecutive += 1
                if consecutive >= self._confirm_count:
                    self.info(
                        f"WFL: TRIGGERED! raw={raw}, "
                        f"baseline={kf.estimate:.0f}, "
                        f"threshold={trigger_threshold:.0f}"
                    )
                    screen.status = "GO!"
                    screen.status_color = "blue"
                    screen.raw_value = raw
                    await screen.refresh()
                    # Brief flash so the user sees "GO!"
                    await asyncio.sleep(0.15)
                    return
            else:
                consecutive = 0
                # Only update baseline with non-triggering samples so the
                # lamp signal can never creep into the baseline estimate.
                kf.update(raw)

            now = time.monotonic()
            if now - last_ui > 0.1:
                screen.raw_value = raw
                screen.baseline = kf.estimate
                screen.threshold = kf.estimate * threshold_factor
                await screen.refresh()
                last_ui = now

            await asyncio.sleep(self._poll_interval)


@dsl_step(tags=["timing", "wait"])
class WaitForLightLegacy(UIStep):
    """Wait for light using the legacy manual-calibration threshold method.

    Runs the traditional two-step calibration flow: the operator measures
    the sensor with the lamp off, then with it on, confirms the threshold,
    and the robot starts immediately. This approach requires manual
    interaction at the start of each run and uses a fixed midpoint
    threshold between the dark and light readings.

    Use this only if the automatic flank-detection method
    (``wait_for_light``) does not work for your setup, for example when
    the sensor is not mounted downward or when the lamp signal is too
    weak for reliable flank detection.

    Args:
        sensor: The AnalogSensor instance for the light sensor.

    Example::

        from libstp.step.wait_for_light import wait_for_light_legacy

        wait_for_light_legacy(robot.defs.wait_for_light_sensor)
    """

    def __init__(self, sensor: AnalogSensor) -> None:
        super().__init__()
        self._sensor = sensor

    def _generate_signature(self) -> str:
        port = getattr(self._sensor, "port", "?")
        return f"WaitForLightLegacy(port={port})"

    async def _execute_step(self, robot: "GenericRobot") -> None:
        from libstp.step.calibration import calibrate_wait_for_light

        cal_step = calibrate_wait_for_light(self._sensor)
        await cal_step.run_step(robot)


