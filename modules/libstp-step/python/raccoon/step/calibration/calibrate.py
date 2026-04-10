"""
Unified calibration step (distance + IR sensors).
"""
from typing import Optional, List, TYPE_CHECKING

from raccoon.step.annotation import dsl_step

from .calibrate_distance import CalibrateDistance

if TYPE_CHECKING:
    from raccoon.sensor_ir import IRSensor


@dsl_step(tags=["calibration"])
class Calibrate(CalibrateDistance):
    """Run a unified distance and IR sensor calibration.

    This is the recommended all-in-one calibration entry point. It drives
    the robot a known distance, prompts the user to measure the actual
    distance traveled, then adjusts the per-wheel ``ticks_to_rad`` values
    to correct odometry. After distance calibration, it automatically
    calibrates IR sensors by sampling them during a drive over the
    calibration surface(s).

    Calibration values are persisted to ``raccoon.project.yml`` using an
    exponential moving average (EMA), so the baseline converges toward the
    true value over multiple calibration runs.

    Prerequisites:
        The robot must have drive motors with encoder feedback and
        a configured kinematics model. For IR calibration, IR sensors
        must be registered in ``robot.defs.analog_sensors``.

    Args:
        distance_cm: Distance (in cm) the robot drives during calibration.
            Longer distances yield more accurate results.
        persist_to_yaml: If ``True``, write the EMA-filtered baseline
            back to ``raccoon.project.yml`` so it persists across runs.
        ema_alpha: EMA smoothing coefficient between 0.0 and 1.0. Higher
            values produce slower convergence but a more stable baseline.
        calibration_sets: List of named IR calibration surface sets to
            run (e.g. ``["default", "transparent"]``). Each set beyond
            the first triggers an additional drive-and-sample cycle.
        exclude_ir_sensors: List of ``IRSensor`` instances to skip during
            IR calibration.

    Example::

        from raccoon.step.calibration import calibrate

        # Basic calibration with defaults
        calibrate()

        # Custom: longer drive, two IR surface sets
        calibrate(
            distance_cm=50.0,
            calibration_sets=["default", "transparent"],
        )
    """

    def __init__(
        self,
        distance_cm: float = 30.0,
        persist_to_yaml: bool = True,
        ema_alpha: float = 0.7,
        calibration_sets: Optional[List[str]] = None,
        exclude_ir_sensors: Optional[List["IRSensor"]] = None,
    ) -> None:
        super().__init__(
            distance_cm=distance_cm,
            calibrate_light_sensors=True,
            persist_to_yaml=persist_to_yaml,
            ema_alpha=ema_alpha,
            calibration_sets=calibration_sets,
            exclude_ir_sensors=exclude_ir_sensors,
        )
