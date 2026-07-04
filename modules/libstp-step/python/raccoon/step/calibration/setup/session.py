"""Robot service that accumulates setup-time calibration evidence.

The service is a passive ledger: :class:`CollectDrive` / :class:`CollectIrSet`
record opportunistic samples into it during the setup mission, and
:class:`CalibrationGate` reads them back to finalize the persisted compensation
layers. It never changes the active odometry source or applies a trim itself —
that is the gate's job.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from raccoon.robot.api import GenericRobot


class CalibrationAxis(Enum):
    FORWARD = "forward"
    LATERAL = "lateral"


@dataclass(frozen=True)
class _PoseSnapshot:
    x: float
    y: float
    heading: float

    @classmethod
    def from_pose(cls, pose) -> "_PoseSnapshot":
        return cls(
            x=float(pose.position[0]),
            y=float(pose.position[1]),
            heading=float(pose.heading),
        )

    def project(self, pose) -> tuple[float, float, float]:
        dx = float(pose.position[0]) - self.x
        dy = float(pose.position[1]) - self.y
        cos_h = math.cos(self.heading)
        sin_h = math.sin(self.heading)
        forward = dx * cos_h + dy * sin_h
        lateral = -dx * sin_h + dy * cos_h
        straight = math.hypot(dx, dy)
        return forward, lateral, straight


@dataclass(frozen=True)
class DriveCalibrationSample:
    axis: CalibrationAxis
    odom_distance_m: float
    ground_truth_distance_m: float
    source: str

    @property
    def scale(self) -> float:
        """Absolute distance-trim scale this sample implies (``odom / ground_truth``).

        The trim is applied as ``commanded * scale`` and the gate assigns this
        value directly via ``MotionTrimService.set_axis_scale`` (it is the target
        scale, not a delta — so it is set, never composed; composing would diverge
        geometrically on repeated calibration). ``odom_distance`` and
        ``ground_truth`` are measured over the same physical drive, so the ratio is
        independent of whatever scale was active during the drive.

        Direction: the internal odometry regulates the drive, so when it
        over-reports (``odom > ground_truth``, e.g. 56.7cm odom for 53.0cm actually
        travelled) a commanded distance under-shoots physically. The scale is then
        ``> 1`` to lengthen the commanded target back onto the real distance — this
        is verified against hardware (a ``< 1`` scale here drives the robot SHORT).

        A distance trim is a magnitude correction, so it is inherently positive.
        Absolute distances are used because the calibration board's pose frame
        has a constant 180° mounting offset — its projected displacement may
        carry the opposite sign of the internal odometry even though both
        measure the same forward travel, and a signed ratio would yield a bogus
        negative scale.
        """
        if abs(self.ground_truth_distance_m) < 1e-9:
            return 1.0
        return abs(self.odom_distance_m) / abs(self.ground_truth_distance_m)


@dataclass
class IrCalibrationSet:
    sensor_ports: set[int] = field(default_factory=set)
    samples_by_port: dict[int, list[float]] = field(default_factory=dict)

    def add(self, port: int, samples: list[float]) -> None:
        self.sensor_ports.add(port)
        self.samples_by_port.setdefault(port, []).extend(float(v) for v in samples)

    def has_minimum_samples(self, min_samples: int) -> bool:
        return bool(self.sensor_ports) and all(
            len(self.samples_by_port.get(port, [])) >= min_samples for port in self.sensor_ports
        )


class SetupCalibrationSession:
    """Accumulates setup-time calibration evidence and applies a final trim."""

    def __init__(self, robot: "GenericRobot") -> None:
        self._robot = robot
        self._board_probe_done = False
        self._board_available = False
        self._gate_completed = False
        self._required_axes: set[CalibrationAxis] = set()
        self._required_ir_sets: set[str] = set()
        self._drive_samples: dict[CalibrationAxis, list[DriveCalibrationSample]] = {
            CalibrationAxis.FORWARD: [],
            CalibrationAxis.LATERAL: [],
        }
        self._ir_sets: dict[str, IrCalibrationSet] = {}

    @property
    def board_available(self) -> bool:
        return self._board_available

    @property
    def gate_completed(self) -> bool:
        return self._gate_completed

    def mark_pending(self) -> None:
        self._gate_completed = False

    def require_axis(self, axis: CalibrationAxis) -> None:
        self._required_axes.add(axis)
        self.mark_pending()

    def require_ir_set(self, set_name: str) -> None:
        self._required_ir_sets.add(set_name)
        self._ir_sets.setdefault(set_name, IrCalibrationSet())
        self.mark_pending()

    def axes_to_finalize(self, requested: list[CalibrationAxis] | None) -> list[CalibrationAxis]:
        axes = requested if requested else sorted(self._required_axes, key=lambda a: a.value)
        return list(axes)

    def ir_sets_to_finalize(self, requested: list[str] | None) -> list[str]:
        sets = requested if requested else sorted(self._required_ir_sets)
        return list(sets)

    def add_drive_sample(self, sample: DriveCalibrationSample) -> None:
        self._drive_samples[sample.axis].append(sample)
        self.mark_pending()

    def get_drive_samples(self, axis: CalibrationAxis) -> list[DriveCalibrationSample]:
        return list(self._drive_samples[axis])

    def clear_drive_samples(self, axis: CalibrationAxis) -> None:
        """Drop all accumulated samples for an axis (operator asked to re-drive)."""
        self._drive_samples[axis] = []
        self.mark_pending()

    def add_ir_samples(
        self,
        set_name: str,
        sensor_ports: list[int],
        samples_by_port: dict[int, list[float]],
    ) -> None:
        bucket = self._ir_sets.setdefault(set_name, IrCalibrationSet())
        for port in sensor_ports:
            bucket.add(port, samples_by_port.get(port, []))
        self._required_ir_sets.add(set_name)
        self.mark_pending()

    def get_ir_set(self, set_name: str) -> IrCalibrationSet:
        return self._ir_sets.setdefault(set_name, IrCalibrationSet())

    def finish_gate(self) -> None:
        self._gate_completed = True

    def median_axis_scale(self, axis: CalibrationAxis) -> float:
        """Median distance-trim scale across this axis' samples (1.0 if none)."""
        samples = self._drive_samples[axis]
        if not samples:
            return 1.0
        scales = sorted(sample.scale for sample in samples)
        mid = len(scales) // 2
        if len(scales) % 2 == 1:
            return float(scales[mid])
        return float((scales[mid - 1] + scales[mid]) / 2.0)

    def median_axis_sample(self, axis: CalibrationAxis) -> DriveCalibrationSample | None:
        """The sample whose scale is the median, for a representative summary.

        Returns ``None`` when the axis has no samples. The applied trim is always
        :meth:`median_axis_scale`; this just picks a real ``(odom, ground_truth)``
        pair to display alongside it.
        """
        samples = self._drive_samples[axis]
        if not samples:
            return None
        ordered = sorted(samples, key=lambda sample: sample.scale)
        return ordered[len(ordered) // 2]

    def ensure_board_probe(self, robot: "GenericRobot", log_step) -> None:
        # Probe whether the calibration board is connected so it can be read as
        # ground truth. This is purely passive: the preferred odometry source is
        # NEVER changed here. Switching the preference would re-route the whole
        # motion system onto the board and hard-reset the odometry frame — the
        # collect_* steps only watch the board alongside the internal estimate
        # to measure their difference, so the motion system keeps using whatever
        # source it was already on.
        if self._board_probe_done:
            return
        from raccoon.hal import OdometrySource

        self._board_available = robot.odometry.is_source_available(OdometrySource.CALIBRATION_BOARD)
        self._board_probe_done = True
        if self._board_available:
            log_step.info("Setup calibration: calibration board detected (ground-truth reference)")
        else:
            log_step.warn(  # noqa: G010 — ClassNameLogger method is named warn, not warning
                "Setup calibration: calibration board unavailable; collect_drive will "
                "ask for a manual distance measurement"
            )

    @staticmethod
    def capture_internal_snapshot(robot: "GenericRobot") -> _PoseSnapshot:
        return _PoseSnapshot.from_pose(robot.odometry.get_internal_pose())

    @staticmethod
    def capture_reference_snapshot(robot: "GenericRobot") -> _PoseSnapshot:
        return _PoseSnapshot.from_pose(SetupCalibrationSession.reference_pose(robot))

    @staticmethod
    def reference_pose(robot: "GenericRobot"):
        """Read the calibration-board pose directly, without changing the active source."""
        from raccoon.hal import OdometrySource

        return robot.odometry.get_pose_from_source(OdometrySource.CALIBRATION_BOARD)

    @staticmethod
    def axis_distance_m(start: _PoseSnapshot, end_pose, axis: CalibrationAxis) -> float:
        forward, lateral, _ = start.project(end_pose)
        return forward if axis == CalibrationAxis.FORWARD else lateral

    @staticmethod
    def detect_axis(start: _PoseSnapshot, end_pose) -> CalibrationAxis:
        """Infer the driven axis from the dominant internal-odometry displacement."""
        forward, lateral, _ = start.project(end_pose)
        return CalibrationAxis.FORWARD if abs(forward) >= abs(lateral) else CalibrationAxis.LATERAL
