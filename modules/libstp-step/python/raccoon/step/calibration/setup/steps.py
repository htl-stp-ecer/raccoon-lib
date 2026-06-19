"""Setup-calibration steps: opportunistic collectors + a finalizing gate.

``collect_drive`` / ``collect_ir_set`` wrap an existing setup motion and, while
it runs, record ground-truth distance and IR samples into the
:class:`SetupCalibrationSession`. ``calibration_gate`` then folds that evidence
into the persisted compensation layers — distance trim via
``MotionTrimService.calibrate_axis`` (the *composing* setter, so an
already-applied scale survives) and IR thresholds via the calibration store —
running a measured fallback drive only for axes/sets that never gathered enough
samples.
"""

from __future__ import annotations

import asyncio
from typing import TYPE_CHECKING

from raccoon.no_calibrate import is_no_calibrate
from raccoon.step.annotation import dsl
from raccoon.step.base import Step
from raccoon.ui.screens.distance import DistanceDrivingScreen, DistanceMeasureScreen
from raccoon.ui.step import UIStep

from .session import CalibrationAxis, DriveCalibrationSample, SetupCalibrationSession

if TYPE_CHECKING:
    from raccoon.robot.api import GenericRobot
    from raccoon.sensor_ir import IRSensor


_IR_MIN_SAMPLES = 20
_FALLBACK_FORWARD_CM = 70.0
_FALLBACK_LATERAL_CM = 50.0
_FALLBACK_IR_DRIVE_CM = 50.0
_FALLBACK_SPEED = 0.4
# Below this, a "ground truth" board reading is treated as a board failure
# (the robot clearly drove, so a ~0cm reference means the board did not track)
# and we fall back to manual measurement instead of recording a bogus sample.
_MIN_GROUND_TRUTH_M = 0.02


def _axis_name(axis: CalibrationAxis) -> str:
    return axis.value


def _axis_abs_cm(value_m: float) -> float:
    return abs(float(value_m)) * 100.0


def _collectable_ir_sensors(
    robot: "GenericRobot",
    sensors: list["IRSensor"] | None,
) -> list["IRSensor"]:
    if sensors is not None:
        return list(sensors)
    from raccoon.sensor_ir import IRSensor as _IRSensor

    return [s for s in robot.defs.analog_sensors if isinstance(s, _IRSensor)]


async def _sample_ir_while_running(
    robot: "GenericRobot",
    step,
    sensors: list["IRSensor"],
) -> dict[int, list[float]]:
    samples: dict[int, list[float]] = {sensor.port: [] for sensor in sensors}
    stop_event = asyncio.Event()

    async def _sample_loop() -> None:
        while not stop_event.is_set():
            for sensor in sensors:
                samples[sensor.port].append(float(sensor.read()))
            await asyncio.sleep(0.01)

    sample_task = asyncio.create_task(_sample_loop())
    try:
        await step.run_step(robot)
    finally:
        stop_event.set()
        await sample_task
    return samples


@dsl(hidden=True)
class CollectDrive(UIStep):
    """Wrap a setup motion and opportunistically record a distance sample.

    Runs the wrapped ``step`` unchanged. When a calibration board is connected,
    it captures the internal-odometry and board-ground-truth displacement across
    the drive, infers the driven axis from the dominant internal displacement,
    and records a :class:`DriveCalibrationSample`. Without a board (or on a board
    dropout / ~0cm reading) it skips silently — the :class:`CalibrationGate` is
    the single place that forces a measured fallback when an axis ends up with no
    samples. Users go through :func:`collect_drive`.
    """

    def __init__(self, step) -> None:
        super().__init__()
        self._step = step

    def _generate_signature(self) -> str:
        return f"CollectDrive(step={self._step})"

    async def _execute_step(self, robot: "GenericRobot") -> None:
        if is_no_calibrate():
            await self._step.run_step(robot)
            return

        session = robot.get_service(SetupCalibrationSession)
        session.ensure_board_probe(robot, self)

        internal_start = session.capture_internal_snapshot(robot)
        reference_start = (
            session.capture_reference_snapshot(robot) if session.board_available else None
        )

        await self._step.run_step(robot)

        internal_end = robot.odometry.get_internal_pose()
        # The driven axis is inferred from the dominant internal-odometry
        # displacement, so callers don't have to declare it up front.
        axis = session.detect_axis(internal_start, internal_end)
        session.require_axis(axis)
        odom_distance_m = session.axis_distance_m(internal_start, internal_end, axis)

        # collect_drive only records a sample opportunistically: when the
        # calibration board is connected and reports a usable ground truth.
        # Without a board (or on a board dropout) it skips silently and shows no
        # UI — the CalibrationGate is the single place that forces a measured
        # fallback drive and the manual measure screen if an axis ends up with
        # no samples at all.
        if not session.board_available or reference_start is None:
            self.debug(
                f"No calibration board; skipping {_axis_name(axis)} drive sample "
                f"(gate forces a measured fallback if this axis stays empty)"
            )
            return

        reference_end = session.reference_pose(robot)
        ground_truth_m = session.axis_distance_m(reference_start, reference_end, axis)
        if abs(ground_truth_m) < _MIN_GROUND_TRUTH_M:
            self.warn(
                f"Calibration board reported ~0cm ground truth for a "
                f"{_axis_abs_cm(odom_distance_m):.1f}cm {_axis_name(axis)} drive; "
                f"skipping sample (gate forces a measured fallback if needed)"
            )
            return

        session.add_drive_sample(
            DriveCalibrationSample(
                axis=axis,
                odom_distance_m=odom_distance_m,
                ground_truth_distance_m=ground_truth_m,
                source="calibration_board",
            )
        )
        self.info(
            f"Collected {_axis_name(axis)} drive sample: "
            f"odom={_axis_abs_cm(odom_distance_m):.1f}cm "
            f"ground_truth={_axis_abs_cm(ground_truth_m):.1f}cm"
        )


@dsl(hidden=True)
class CollectIrSet(Step):
    """Wrap a setup motion and sample IR sensors into a named calibration set.

    Runs the wrapped ``step`` while polling each IR sensor at ~100 Hz, appending
    the readings to ``set_name`` in the :class:`SetupCalibrationSession`. The
    :class:`CalibrationGate` later turns the accumulated samples into stored
    black/white thresholds. Users go through :func:`collect_ir_set`.
    """

    def __init__(self, step, set_name: str, sensors: list["IRSensor"] | None = None) -> None:
        super().__init__()
        self._step = step
        self._set_name = set_name
        self._sensors = sensors

    def _generate_signature(self) -> str:
        return f"CollectIrSet(set={self._set_name!r}, step={self._step})"

    async def _execute_step(self, robot: "GenericRobot") -> None:
        session = robot.get_service(SetupCalibrationSession)
        session.require_ir_set(self._set_name)
        sensors = _collectable_ir_sensors(robot, self._sensors)
        if not sensors or is_no_calibrate():
            await self._step.run_step(robot)
            return
        samples = await _sample_ir_while_running(robot, self._step, sensors)
        session.add_ir_samples(self._set_name, [s.port for s in sensors], samples)
        self.info(
            f"Collected IR set '{self._set_name}' from {len(sensors)} sensor(s) during setup motion"
        )


@dsl(hidden=True)
class CalibrationGate(UIStep):
    """Finalize all accumulated setup-calibration evidence exactly once.

    For each required drive axis it folds the median sample factor into the
    distance trim via ``MotionTrimService.calibrate_axis`` (composing, so a
    previously-applied scale is preserved), running a measured fallback drive
    first if the axis gathered no samples. For each required IR set it applies
    stored thresholds, driving a fallback sampling pass if needed. Idempotent:
    once finalized it returns immediately until new evidence arrives. Under
    ``--no-calibrate`` it is a no-op. Users go through :func:`calibration_gate`.
    """

    def __init__(
        self,
        require_axes: list[CalibrationAxis] | None = None,
        require_ir_sets: list[str] | None = None,
        forward_fallback_cm: float = _FALLBACK_FORWARD_CM,
        lateral_fallback_cm: float = _FALLBACK_LATERAL_CM,
    ) -> None:
        super().__init__()
        self._require_axes = require_axes
        self._require_ir_sets = require_ir_sets
        self._forward_fallback_cm = forward_fallback_cm
        self._lateral_fallback_cm = lateral_fallback_cm

    def _generate_signature(self) -> str:
        return (
            f"CalibrationGate(axes={self._require_axes or []}, "
            f"ir_sets={self._require_ir_sets or []}, "
            f"fwd_fallback={self._forward_fallback_cm:.0f}cm, "
            f"lat_fallback={self._lateral_fallback_cm:.0f}cm)"
        )

    async def _execute_step(self, robot: "GenericRobot") -> None:
        if is_no_calibrate():
            return

        session = robot.get_service(SetupCalibrationSession)
        if session.gate_completed:
            return
        axes = session.axes_to_finalize(self._require_axes)
        ir_sets = session.ir_sets_to_finalize(self._require_ir_sets)

        if not axes and not ir_sets:
            session.finish_gate()
            return

        if axes:
            session.ensure_board_probe(robot, self)
            await self._finalize_drive_axes(robot, session, axes)

        for set_name in ir_sets:
            await self._finalize_ir_set(robot, session, set_name)

        session.finish_gate()
        from raccoon.step.calibration.state import set_distance_calibrated

        set_distance_calibrated()

    async def _finalize_drive_axes(
        self,
        robot: "GenericRobot",
        session: SetupCalibrationSession,
        axes: list[CalibrationAxis],
    ) -> None:
        from raccoon.step.motion._motion_trim import MotionTrimService

        trim_svc = robot.get_service(MotionTrimService)
        supports_lateral = robot.drive.supports_lateral_motion()
        for axis in axes:
            # Only calibrate the lateral axis when the drivetrain can actually
            # strafe — otherwise the fallback strafe drive is meaningless.
            if axis == CalibrationAxis.LATERAL and not supports_lateral:
                self.debug("Drivetrain has no lateral motion; skipping lateral trim calibration")
                continue
            if not session.get_drive_samples(axis):
                await self._run_drive_fallback(robot, session, axis)
            # Fold the median per-sample factor (odom / ground_truth) into the
            # trim via the COMPOSING setter, so an already-applied scale is
            # preserved instead of overwritten:
            #   S_new = S_current * (requested / measured)
            #         = S_current * (median_factor / 1.0)
            factor = session.median_axis_factor(axis)
            trim_svc.calibrate_axis(_axis_name(axis), requested_m=factor, measured_m=1.0)
            self.info(
                f"Applied {_axis_name(axis)} trim factor x{factor:.5f} "
                f"from {len(session.get_drive_samples(axis))} sample(s)"
            )

    async def _run_drive_fallback(
        self,
        robot: "GenericRobot",
        session: SetupCalibrationSession,
        axis: CalibrationAxis,
    ) -> None:
        if axis == CalibrationAxis.FORWARD:
            from raccoon import drive_forward

            target_cm = self._forward_fallback_cm
            step = drive_forward(target_cm, speed=_FALLBACK_SPEED, heading=0)
        else:
            from raccoon import strafe_right

            target_cm = self._lateral_fallback_cm
            step = strafe_right(target_cm, speed=_FALLBACK_SPEED, heading=0)

        await self.run_with_ui(
            DistanceDrivingScreen(target_cm),
            self._collect_fallback_drive(robot, session, step, axis),
        )

    async def _collect_fallback_drive(
        self,
        robot: "GenericRobot",
        session: SetupCalibrationSession,
        step,
        axis: CalibrationAxis,
    ) -> None:
        internal_start = session.capture_internal_snapshot(robot)
        reference_start = (
            session.capture_reference_snapshot(robot) if session.board_available else None
        )
        await step.run_step(robot)
        internal_end = robot.odometry.get_internal_pose()
        odom_distance_m = session.axis_distance_m(internal_start, internal_end, axis)

        ground_truth_m = None
        if session.board_available and reference_start is not None:
            reference_end = session.reference_pose(robot)
            board_m = session.axis_distance_m(reference_start, reference_end, axis)
            if abs(board_m) >= _MIN_GROUND_TRUTH_M:
                ground_truth_m = board_m
            else:
                self.warn(
                    f"Calibration board reported ~0cm ground truth for a "
                    f"{_axis_abs_cm(odom_distance_m):.1f}cm {_axis_name(axis)} drive; "
                    f"falling back to manual measurement"
                )

        if ground_truth_m is not None:
            source = "calibration_board"
        else:
            measured_cm = await self.show(
                DistanceMeasureScreen(
                    requested_distance=_axis_abs_cm(odom_distance_m),
                    default_value=_axis_abs_cm(odom_distance_m),
                )
            )
            sign = 1.0 if odom_distance_m >= 0.0 else -1.0
            ground_truth_m = sign * (float(measured_cm) / 100.0)
            source = "manual_entry"
        session.add_drive_sample(
            DriveCalibrationSample(
                axis=axis,
                odom_distance_m=odom_distance_m,
                ground_truth_distance_m=ground_truth_m,
                source=source,
            )
        )

    async def _finalize_ir_set(
        self,
        robot: "GenericRobot",
        session: SetupCalibrationSession,
        set_name: str,
    ) -> None:
        ir_set = session.get_ir_set(set_name)
        sensors = _collectable_ir_sensors(robot, None)
        if not sensors:
            self.warn(f"No IR sensors available for setup calibration set '{set_name}'")
            return
        if not ir_set.has_minimum_samples(_IR_MIN_SAMPLES):
            await self._run_ir_fallback(robot, session, set_name, sensors)
            ir_set = session.get_ir_set(set_name)
        await self._confirm_ir_set(robot, set_name, sensors, ir_set.samples_by_port)

    async def _run_ir_fallback(
        self,
        robot: "GenericRobot",
        session: SetupCalibrationSession,
        set_name: str,
        sensors: list["IRSensor"],
    ) -> None:
        from raccoon.step.motion.drive import _drive_forward_uncalibrated

        proceed = await self.confirm(
            f"Place sensors on {set_name.upper()} surface, then confirm to drive.",
            title=f"IR Calibration: {set_name.upper()}",
            yes_label="Drive",
            no_label="Skip",
        )
        if not proceed:
            self.warn(f"Skipping fallback IR calibration for set '{set_name}'")
            return

        samples: dict[int, list[float]] = {sensor.port: [] for sensor in sensors}
        stop_event = asyncio.Event()

        async def _sample_loop() -> None:
            while not stop_event.is_set():
                for sensor in sensors:
                    samples[sensor.port].append(float(sensor.read()))
                await asyncio.sleep(0.01)

        async def _drive() -> None:
            sample_task = asyncio.create_task(_sample_loop())
            try:
                await _drive_forward_uncalibrated(
                    _FALLBACK_IR_DRIVE_CM, speed=_FALLBACK_SPEED
                ).run_step(robot)
            finally:
                stop_event.set()
                await sample_task
            for motor in robot.drive.get_motors():
                motor.set_speed(0)

        await self.run_with_ui(DistanceDrivingScreen(_FALLBACK_IR_DRIVE_CM), _drive())
        session.add_ir_samples(set_name, [sensor.port for sensor in sensors], samples)

    async def _confirm_ir_set(
        self,
        robot: "GenericRobot",
        set_name: str,
        sensors: list["IRSensor"],
        samples_by_port: dict[int, list[float]],
    ) -> None:
        from raccoon import calibration_store as CalibrationStore
        from raccoon.calibration_store import CalibrationType
        from raccoon.step.calibration.sensors.dataclasses import SensorCalibrationData
        from raccoon.step.calibration.sensors.ir_results_screen import IRResultsDashboardScreen

        current_samples = {port: list(values) for port, values in samples_by_port.items()}
        while True:
            sensor_data: list[SensorCalibrationData] = []
            for sensor in sensors:
                values = [float(v) for v in current_samples.get(sensor.port, [])]
                if values:
                    sensor.calibrate(values)
                sensor_data.append(
                    SensorCalibrationData(
                        port=sensor.port,
                        samples=values,
                        black_threshold=float(getattr(sensor, "blackThreshold", 0.0)),
                        white_threshold=float(getattr(sensor, "whiteThreshold", 0.0)),
                        black_mean=float(getattr(sensor, "blackMean", 0.0)),
                        white_mean=float(getattr(sensor, "whiteMean", 0.0)),
                        black_std=float(getattr(sensor, "blackStdDev", 0.0)),
                        white_std=float(getattr(sensor, "whiteStdDev", 0.0)),
                    )
                )

            result = await self.show(IRResultsDashboardScreen(sensors=sensor_data))
            if result is None:
                self.warn(f"IR calibration dashboard dismissed for set '{set_name}'")
                return

            if result.confirmed:
                for sensor, data in zip(sensors, sensor_data, strict=False):
                    sensor.setCalibration(data.black_threshold, data.white_threshold)
                    CalibrationStore.store_readings(
                        CalibrationType.IR_SENSOR,
                        data.white_threshold,
                        data.black_threshold,
                        f"{set_name}_port{sensor.port}",
                    )
                self.info(f"Applied IR calibration set '{set_name}'")
                return

            await self._run_ir_fallback(
                robot, robot.get_service(SetupCalibrationSession), set_name, sensors
            )
            current_samples = (
                robot.get_service(SetupCalibrationSession).get_ir_set(set_name).samples_by_port
            )


@dsl(tags=["calibration", "setup"])
def collect_drive(step) -> CollectDrive:
    """Wrap a setup motion to opportunistically record a distance sample.

    The wrapped ``step`` runs unchanged. While it executes, the calibration board
    (if connected) is read as ground truth alongside the internal odometry, and a
    per-axis distance sample is stored in the setup calibration session for the
    :func:`calibration_gate` to fold into the distance trim. Without a board it is
    a transparent pass-through.

    Prerequisites:
        A calibration board for ground-truth sampling (optional — the gate forces
        a measured fallback drive otherwise).

    Args:
        step: Any step (e.g. a ``seq([...])``) whose motion should be measured.

    Returns:
        CollectDrive: A step wrapping ``step``.

    Example::

        from raccoon.step.calibration.setup import collect_drive

        collect_drive(
            seq(
                [
                    drive_forward().until(over_line(front.left)),
                    strafe_right().until(over_line(front.left)),
                ]
            ),
        )
    """
    return CollectDrive(step)


@dsl(tags=["calibration", "setup"])
def collect_ir_set(
    step,
    set_name: str,
    sensors: list["IRSensor"] | None = None,
) -> CollectIrSet:
    """Wrap a setup motion to sample IR sensors into a named calibration set.

    The wrapped ``step`` runs while every IR sensor is polled at ~100 Hz; the
    readings accumulate under ``set_name`` in the setup calibration session. The
    :func:`calibration_gate` later converts the samples into stored black/white
    thresholds.

    Args:
        step: Any step whose motion sweeps the sensors over the surface(s).
        set_name: Name of the IR calibration set (e.g. ``"default"``, ``"upper"``).
        sensors: Explicit IR sensors to sample; defaults to all IR sensors in
            ``robot.defs.analog_sensors``.

    Returns:
        CollectIrSet: A step wrapping ``step``.

    Example::

        from raccoon.step.calibration.setup import collect_ir_set

        collect_ir_set(drive_backward(cm=50), set_name="upper")
    """
    return CollectIrSet(step, set_name=set_name, sensors=sensors)


@dsl(tags=["calibration", "setup"])
def calibration_gate(
    require_axes: list[CalibrationAxis] | None = None,
    require_ir_sets: list[str] | None = None,
    forward_fallback_cm: float = _FALLBACK_FORWARD_CM,
    lateral_fallback_cm: float = _FALLBACK_LATERAL_CM,
) -> CalibrationGate:
    """Finalize the accumulated setup-calibration evidence exactly once.

    Folds each required drive axis' median sample into the distance trim via the
    composing ``MotionTrimService.calibrate_axis`` (preserving any already-applied
    scale) and applies each required IR set's thresholds. Axes or IR sets that
    never gathered enough samples trigger a measured fallback drive (and a manual
    measurement screen if no calibration board is present). Idempotent and a no-op
    under ``--no-calibrate``.

    Args:
        require_axes: Drive axes that must be finalized. ``None`` finalizes every
            axis that accumulated a sample or was marked required by a collector.
        require_ir_sets: IR set names that must be finalized. ``None`` finalizes
            every set seen so far.
        forward_fallback_cm: Distance (cm) of the forward fallback drive when the
            forward axis has no samples.
        lateral_fallback_cm: Distance (cm) of the lateral fallback strafe.

    Returns:
        CalibrationGate: The finalizing step.

    Example::

        from raccoon.step.calibration.setup import CalibrationAxis, calibration_gate

        calibration_gate(
            require_axes=[CalibrationAxis.FORWARD],
            require_ir_sets=["default", "upper"],
        )
    """
    return CalibrationGate(
        require_axes=require_axes,
        require_ir_sets=require_ir_sets,
        forward_fallback_cm=forward_fallback_cm,
        lateral_fallback_cm=lateral_fallback_cm,
    )
