"""Setup-calibration steps: opportunistic collectors + a finalizing gate.

``collect_drive`` / ``collect_ir_set`` wrap an existing setup motion and, while
it runs, record ground-truth distance and IR samples into the
:class:`SetupCalibrationSession`. ``calibration_gate`` then folds that evidence
into the persisted compensation layers — distance trim via
``MotionTrimService.set_axis_scale`` (the measured scale is the absolute target,
so it is assigned, not composed) and IR thresholds via the calibration
store — running a measured fallback drive only for axes/sets that never gathered
enough samples.
"""

from __future__ import annotations

import asyncio
from typing import TYPE_CHECKING

from raccoon.no_calibrate import is_no_calibrate
from raccoon.step.annotation import dsl
from raccoon.step.base import Step
from raccoon.ui.screens.distance import (
    DISTANCE_MEASURE_RETRY,
    DistanceConfirmScreen,
    DistanceDrivingScreen,
    DistanceMeasureScreen,
)
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
# The distance fallback drive runs at full speed: a longer, faster drive gives a
# better odom-vs-ground-truth ratio than a slow crawl, and there is nothing to
# sample mid-drive (unlike the IR fallback, which stays slow at _FALLBACK_SPEED).
_FALLBACK_DRIVE_SPEED = 1.0
# Below this, a "ground truth" board reading is treated as a board failure
# (the robot clearly drove, so a ~0cm reference means the board did not track)
# and we fall back to manual measurement instead of recording a bogus sample.
_MIN_GROUND_TRUTH_M = 0.02


def _axis_name(axis: CalibrationAxis) -> str:
    return axis.value


async def _run_step_untrimmed(robot: "GenericRobot", step) -> None:
    """Run ``step`` with the distance trim bypassed.

    Distance-calibration measurement drives must travel a distance that depends
    only on the command, not on the currently-stored scale — otherwise the
    ``odom / ground_truth`` ratio they yield is contaminated by the scale being
    measured and cannot correct a bad one (nor is it reproducible run to run).
    The :class:`MotionTrimService` bypass makes every ``cm``-based drive inside
    this call target its raw commanded distance.
    """
    from raccoon.step.motion._motion_trim import MotionTrimService

    trim_svc = robot.get_service(MotionTrimService)
    with trim_svc.bypass_scaling():
        await step.run_step(robot)


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
    and records a :class:`DriveCalibrationSample`.

    Without a usable board ground truth (no board, dropout, or ~0cm reading) the
    behaviour depends on ``manual_measurement``: when ``True`` (default) it prompts
    the operator to measure the travelled distance by hand and records that as the
    sample; when ``False`` it rejects the drive and records nothing (the axis is
    left for the :class:`CalibrationGate` to handle, or left untrimmed). Users go
    through :func:`collect_drive`.
    """

    def __init__(self, step, manual_measurement: bool = True) -> None:
        super().__init__()
        self._step = step
        self._manual_measurement = manual_measurement

    def _generate_signature(self) -> str:
        return f"CollectDrive(step={self._step}, manual={self._manual_measurement})"

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

        # Drive the wrapped motion RAW: a distance-calibration measurement must
        # not run through the trim layer, or the physical distance travelled —
        # and thus the odom/ground-truth ratio recorded below — would depend on
        # the currently-stored (possibly wrong) scale and never converge.
        await _run_step_untrimmed(robot, self._step)

        internal_end = robot.odometry.get_internal_pose()
        # The driven axis is inferred from the dominant internal-odometry
        # displacement, so callers don't have to declare it up front.
        axis = session.detect_axis(internal_start, internal_end)
        session.require_axis(axis)
        odom_distance_m = session.axis_distance_m(internal_start, internal_end, axis)

        # Prefer the calibration board's ground truth when it tracked the drive.
        if session.board_available and reference_start is not None:
            reference_end = session.reference_pose(robot)
            ground_truth_m = session.axis_distance_m(reference_start, reference_end, axis)
            if abs(ground_truth_m) >= _MIN_GROUND_TRUTH_M:
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
                return
            self.warn(
                f"Calibration board reported ~0cm ground truth for a "
                f"{_axis_abs_cm(odom_distance_m):.1f}cm {_axis_name(axis)} drive; "
                f"no usable board reference"
            )

        # No usable board ground truth. Either ask the operator to measure by
        # hand, or reject the drive entirely — controlled by manual_measurement.
        if not self._manual_measurement:
            self.debug(
                f"No board ground truth for {_axis_name(axis)} drive; rejecting "
                f"sample (manual_measurement disabled)"
            )
            return

        measured_cm = await self.show(
            DistanceMeasureScreen(
                requested_distance=_axis_abs_cm(odom_distance_m),
                default_value=_axis_abs_cm(odom_distance_m),
            )
        )
        if measured_cm is None:
            self.warn(f"Manual measurement dismissed for {_axis_name(axis)} drive; skipping sample")
            return
        # Preserve the travel direction so the axis projection stays consistent;
        # the scale itself is computed from absolute distances.
        sign = 1.0 if odom_distance_m >= 0.0 else -1.0
        ground_truth_m = sign * (float(measured_cm) / 100.0)
        if abs(ground_truth_m) < _MIN_GROUND_TRUTH_M:
            self.warn(f"Manual measurement of ~0cm for {_axis_name(axis)} drive; skipping sample")
            return
        session.add_drive_sample(
            DriveCalibrationSample(
                axis=axis,
                odom_distance_m=odom_distance_m,
                ground_truth_distance_m=ground_truth_m,
                source="manual_entry",
            )
        )
        self.info(
            f"Collected manual {_axis_name(axis)} sample: "
            f"odom={_axis_abs_cm(odom_distance_m):.1f}cm measured={float(measured_cm):.1f}cm"
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

    For each required drive axis it assigns the median sample scale as the
    distance trim via ``MotionTrimService.set_axis_scale`` (it is the absolute
    target scale, so composing it would diverge on repeat), running a
    measured fallback drive first if the axis gathered no samples — using the
    calibration board for
    ground truth when present, otherwise prompting the operator to measure the
    travelled distance by hand. For each required IR set it applies stored
    thresholds, driving a fallback sampling pass if needed. Idempotent: once
    finalized it returns immediately until new evidence arrives. Distance is only
    marked calibrated when an axis was actually trimmed. Under ``--no-calibrate``
    it is a no-op. Users go through :func:`calibration_gate`.
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

        trimmed_axes = 0
        if axes:
            session.ensure_board_probe(robot, self)
            trimmed_axes = await self._finalize_drive_axes(robot, session, axes)

        for set_name in ir_sets:
            await self._finalize_ir_set(robot, session, set_name)

        session.finish_gate()

        # Only advertise distance as calibrated when an axis was actually
        # trimmed. If every axis was left untrimmed (e.g. the operator dismissed
        # the fallback drive or its measurement), the distance data is still
        # missing — keep the flag false so distance-based drives keep warning and
        # a later setup run can finish the job.
        if trimmed_axes:
            from raccoon.step.calibration.state import set_distance_calibrated

            set_distance_calibrated()

    async def _finalize_drive_axes(
        self,
        robot: "GenericRobot",
        session: SetupCalibrationSession,
        axes: list[CalibrationAxis],
    ) -> int:
        from raccoon.step.motion._motion_trim import MotionTrimService

        trim_svc = robot.get_service(MotionTrimService)
        supports_lateral = robot.drive.supports_lateral_motion()
        trimmed = 0
        for axis in axes:
            # Only calibrate the lateral axis when the drivetrain can actually
            # strafe — otherwise the fallback strafe drive is meaningless.
            if axis == CalibrationAxis.LATERAL and not supports_lateral:
                self.debug("Drivetrain has no lateral motion; skipping lateral trim calibration")
                continue
            # Collect (driving a fallback if no opportunistic samples), then show a
            # summary the operator confirms before the trim is applied. "Redo
            # Calibration" discards the evidence and drives a fresh measured
            # fallback. The loop repeats until the operator applies, dismisses, or
            # the redo produces no usable ground truth.
            applied = False
            while True:
                if not session.get_drive_samples(axis):
                    # No opportunistic samples (or the operator asked to redo), so
                    # drive a measured fallback for ground truth. With a calibration
                    # board the board supplies it; without one the fallback prompts
                    # the operator to measure the travelled distance by hand.
                    await self._run_drive_fallback(robot, session, axis)
                    if not session.get_drive_samples(axis):
                        self.warn(
                            f"No usable ground truth for the {_axis_name(axis)} drive; "
                            f"leaving axis untrimmed"
                        )
                        break

                # SET the trim to the measured scale (absolute), never compose it.
                #
                # The per-sample scale is odom / ground_truth, both measured over the
                # SAME physical drive, so it is the absolute target scale and is
                # INDEPENDENT of the currently-applied scale. Composing it
                # (S_new = S_current * scale) would multiply the scale on every run and
                # diverge geometrically (the robot drives further and further). Absolute
                # assignment converges in one run and is idempotent.
                scale = session.median_axis_scale(axis)
                sample = session.median_axis_sample(axis)
                # Show a representative (requested, measured) pair; derive measured
                # from the applied scale so the summary's scale matches what is set.
                requested_cm = _axis_abs_cm(sample.odom_distance_m) if sample else 0.0
                measured_cm = requested_cm / scale if scale else requested_cm
                result = await self.show(
                    DistanceConfirmScreen(requested=requested_cm, measured=measured_cm)
                )
                if result is None:
                    self.warn(
                        f"Distance summary dismissed for {_axis_name(axis)}; "
                        f"leaving axis untrimmed"
                    )
                    break
                if not result.confirmed and result.reenter:
                    # "Re-enter Value": the drive was fine, only the typed
                    # measurement was wrong. Re-open the keypad and replace the
                    # ground truth without driving again.
                    if await self._reenter_measurement(session, axis, sample, measured_cm):
                        continue
                    # Operator dismissed the keypad: leave the existing sample as-is
                    # and re-show the summary so they can still apply or redo.
                    continue
                if not result.confirmed:
                    # "Redo Calibration": throw away the samples and drive again.
                    self.info(f"Operator chose to redo {_axis_name(axis)} distance calibration")
                    session.clear_drive_samples(axis)
                    continue

                trim_svc.set_axis_scale(_axis_name(axis), scale)
                applied = True
                self.info(
                    f"Applied {_axis_name(axis)} trim scale {scale:.5f} "
                    f"from {len(session.get_drive_samples(axis))} sample(s)"
                )
                break

            if applied:
                trimmed += 1
        return trimmed

    async def _reenter_measurement(
        self,
        session: SetupCalibrationSession,
        axis: CalibrationAxis,
        sample: DriveCalibrationSample | None,
        current_measured_cm: float,
    ) -> bool:
        """Re-prompt only the measured distance for an already-driven axis.

        Keeps the drive's odometry displacement and replaces the ground-truth
        measurement with a freshly typed value, so a typo on the measurement
        screen can be fixed without re-driving. Returns ``True`` when a new value
        was entered and the sample was updated, ``False`` when the operator
        dismissed the keypad (the existing sample is left untouched).
        """
        if sample is None:
            return False
        odom_cm = _axis_abs_cm(sample.odom_distance_m)
        measured_cm = await self.show(
            DistanceMeasureScreen(
                requested_distance=odom_cm,
                default_value=current_measured_cm,
            )
        )
        if measured_cm is None:
            self.warn(f"Re-entry dismissed for {_axis_name(axis)}; keeping previous measurement")
            return False
        # Preserve the drive direction; the scale is computed from absolute distances.
        sign = 1.0 if sample.odom_distance_m >= 0.0 else -1.0
        ground_truth_m = sign * (float(measured_cm) / 100.0)
        if abs(ground_truth_m) < _MIN_GROUND_TRUTH_M:
            self.warn(f"Re-entered ~0cm for {_axis_name(axis)}; keeping previous measurement")
            return False
        # Replace every sample for this axis with the corrected single sample: the
        # operator's hand measurement is the authoritative ground truth now.
        session.clear_drive_samples(axis)
        session.add_drive_sample(
            DriveCalibrationSample(
                axis=axis,
                odom_distance_m=sample.odom_distance_m,
                ground_truth_distance_m=ground_truth_m,
                source="manual_entry",
            )
        )
        self.info(
            f"Re-entered {_axis_name(axis)} measurement: "
            f"odom={odom_cm:.1f}cm measured={float(measured_cm):.1f}cm"
        )
        return True

    async def _run_drive_fallback(
        self,
        robot: "GenericRobot",
        session: SetupCalibrationSession,
        axis: CalibrationAxis,
    ) -> None:
        if axis == CalibrationAxis.FORWARD:
            from raccoon import drive_forward

            target_cm = self._forward_fallback_cm
            verb, direction = "drive", "forward"
        else:
            from raccoon import strafe_right

            target_cm = self._lateral_fallback_cm
            verb, direction = "strafe", "right"

        # Announce the motion before it starts: the robot is about to move on its
        # own, so give the operator a chance to clear the path (or skip).
        proceed = await self.confirm(
            f"The robot will now {verb} {target_cm:.0f} cm {direction} to calibrate "
            f"the {_axis_name(axis)} distance axis. Make sure it has a clear path, "
            f"then start.",
            title=f"{_axis_name(axis).capitalize()} Distance Calibration",
            yes_label="Drive",
            no_label="Skip",
        )
        if not proceed:
            self.warn(
                f"Operator skipped the {_axis_name(axis)} fallback drive; "
                f"leaving axis untrimmed"
            )
            return

        # Drive → measure loop. On the manual-measurement screen the operator can
        # tap Retry (e.g. the robot was bumped or moved), which re-drives instead
        # of recording a bogus sample. A fresh step is built each pass since a
        # motion step is not re-runnable.
        while True:
            # Do NOT pin an absolute heading: the gate may run before any
            # mark_heading_reference(), so an absolute target would crash. Leaving
            # it unset holds the current world heading, which is enough to measure
            # the straight-line drive.
            if axis == CalibrationAxis.FORWARD:
                step = drive_forward(target_cm, speed=_FALLBACK_DRIVE_SPEED)
            else:
                step = strafe_right(target_cm, speed=_FALLBACK_DRIVE_SPEED)

            odom_distance_m, board_ground_truth_m = await self.run_with_ui(
                DistanceDrivingScreen(target_cm),
                self._collect_fallback_drive(robot, session, step, axis),
            )

            ground_truth_m = board_ground_truth_m
            source = "calibration_board"
            if ground_truth_m is None:
                # No calibration-board ground truth (no board, dropout, or ~0cm
                # reading). Prompt the operator to measure the distance the robot
                # actually travelled — that hand measurement becomes the ground
                # truth. Default the entry to what the odometry believes so the
                # operator only nudges it.
                measured_cm = await self.show(
                    DistanceMeasureScreen(
                        requested_distance=_axis_abs_cm(odom_distance_m),
                        default_value=_axis_abs_cm(odom_distance_m),
                        allow_retry=True,
                    )
                )
                if measured_cm is DISTANCE_MEASURE_RETRY:
                    self.info(
                        f"Operator asked to re-drive the {_axis_name(axis)} fallback "
                        f"(robot moved); driving again"
                    )
                    continue
                if measured_cm is None:
                    self.warn(
                        f"Manual measurement dismissed for the {_axis_name(axis)} fallback "
                        f"drive; leaving axis untrimmed"
                    )
                    return
                # Preserve the travel direction; the scale uses absolute distances.
                sign = 1.0 if odom_distance_m >= 0.0 else -1.0
                ground_truth_m = sign * (float(measured_cm) / 100.0)
                if abs(ground_truth_m) < _MIN_GROUND_TRUTH_M:
                    self.warn(
                        f"Manual measurement of ~0cm for the {_axis_name(axis)} fallback "
                        f"drive; discarding sample"
                    )
                    return
                source = "manual_entry"

            session.add_drive_sample(
                DriveCalibrationSample(
                    axis=axis,
                    odom_distance_m=odom_distance_m,
                    ground_truth_distance_m=ground_truth_m,
                    source=source,
                )
            )
            return

    async def _collect_fallback_drive(
        self,
        robot: "GenericRobot",
        session: SetupCalibrationSession,
        step,
        axis: CalibrationAxis,
    ) -> tuple[float, float | None]:
        """Drive the fallback and return ``(odom_distance_m, board_ground_truth_m)``.

        ``board_ground_truth_m`` is ``None`` when no calibration board is present
        (or it dropped out / reported ~0cm); the caller then falls back to manual
        measurement.
        """
        internal_start = session.capture_internal_snapshot(robot)
        reference_start = (
            session.capture_reference_snapshot(robot) if session.board_available else None
        )
        # Same rule as the opportunistic collector: the fallback measurement
        # drive runs raw so its odom/ground-truth ratio is independent of the
        # stored scale (see :func:`_run_step_untrimmed`).
        await _run_step_untrimmed(robot, step)
        internal_end = robot.odometry.get_internal_pose()
        odom_distance_m = session.axis_distance_m(internal_start, internal_end, axis)

        board_ground_truth_m = None
        if session.board_available and reference_start is not None:
            reference_end = session.reference_pose(robot)
            board_m = session.axis_distance_m(reference_start, reference_end, axis)
            if abs(board_m) >= _MIN_GROUND_TRUTH_M:
                board_ground_truth_m = board_m
            else:
                self.warn(
                    f"Calibration board reported ~0cm ground truth for a "
                    f"{_axis_abs_cm(odom_distance_m):.1f}cm {_axis_name(axis)} drive; "
                    f"falling back to manual measurement"
                )

        return odom_distance_m, board_ground_truth_m

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

            result = await self.show(
                IRResultsDashboardScreen(sensors=sensor_data, set_name=set_name)
            )
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
def collect_drive(step, manual_measurement: bool = True) -> CollectDrive:
    """Wrap a setup motion to opportunistically record a distance sample.

    The wrapped ``step`` runs unchanged. While it executes, the calibration board
    (if connected) is read as ground truth alongside the internal odometry, and a
    per-axis distance sample is stored in the setup calibration session for the
    :func:`calibration_gate` to fold into the distance trim.

    Without a usable board ground truth, ``manual_measurement`` decides what
    happens: when ``True`` (default) the operator is prompted to measure the
    travelled distance by hand and that becomes the sample; when ``False`` the
    drive is rejected and no sample is recorded (the axis is left for the gate
    fallback, or left untrimmed).

    Prerequisites:
        A calibration board for ground-truth sampling (optional — see
        ``manual_measurement``).

    Args:
        step: Any step (e.g. a ``seq([...])``) whose motion should be measured.
        manual_measurement: When no board ground truth is available, prompt the
            operator to measure the distance by hand (``True``, default) instead of
            rejecting the drive (``False``).

    Returns:
        CollectDrive: A step wrapping ``step``.

    Example::

        from raccoon.step.calibration.setup import collect_drive

        # Ask for a manual measurement if no board is connected (default):
        collect_drive(
            seq(
                [
                    drive_forward().until(over_line(front.left)),
                    strafe_right().until(over_line(front.left)),
                ]
            ),
        )

        # Board-only: silently skip the sample when no board is present.
        collect_drive(drive_forward().until(over_line(front.left)), manual_measurement=False)
    """
    return CollectDrive(step, manual_measurement=manual_measurement)


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
    fallback_cm: float | None = None,
    forward_fallback_cm: float = _FALLBACK_FORWARD_CM,
    lateral_fallback_cm: float = _FALLBACK_LATERAL_CM,
) -> CalibrationGate:
    """Finalize the accumulated setup-calibration evidence exactly once.

    Assigns each required drive axis' median sample scale as the distance trim via
    ``MotionTrimService.set_axis_scale`` (it is the absolute target scale, so it is
    set, not composed — composing would diverge on repeated calibration) and
    applies each required IR set's thresholds. A drive axis with no
    samples triggers a measured fallback drive: with a calibration board the board
    supplies the ground truth, otherwise the operator is prompted to measure the
    travelled distance by hand. Idempotent and a no-op under ``--no-calibrate``.

    Args:
        require_axes: Drive axes that must be finalized. ``None`` finalizes every
            axis that accumulated a sample or was marked required by a collector.
        require_ir_sets: IR set names that must be finalized. ``None`` finalizes
            every set seen so far.
        fallback_cm: Distance (cm) for the fallback drive, applied to *both* axes.
            A convenience override — when given it replaces both
            ``forward_fallback_cm`` and ``lateral_fallback_cm``. ``None`` keeps the
            per-axis defaults.
        forward_fallback_cm: Distance (cm) of the forward fallback drive when the
            forward axis has no samples.
        lateral_fallback_cm: Distance (cm) of the lateral fallback strafe.

    Returns:
        CalibrationGate: The finalizing step.

    Example::

        from raccoon.step.calibration.setup import CalibrationAxis, calibration_gate

        # Default per-axis fallback distances:
        calibration_gate(
            require_axes=[CalibrationAxis.FORWARD],
            require_ir_sets=["default", "upper"],
        )

        # Drive 80 cm for any fallback (both axes):
        calibration_gate(require_axes=[CalibrationAxis.FORWARD], fallback_cm=80)
    """
    if fallback_cm is not None:
        forward_fallback_cm = float(fallback_cm)
        lateral_fallback_cm = float(fallback_cm)
    return CalibrationGate(
        require_axes=require_axes,
        require_ir_sets=require_ir_sets,
        forward_fallback_cm=forward_fallback_cm,
        lateral_fallback_cm=lateral_fallback_cm,
    )
