"""
Unified calibration step (distance + IR sensors).
"""

from libstp.step.annotation import dsl

from .calibrate_distance import CalibrateDistance, DEFAULT_EMA_ALPHA


@dsl(tags=["calibration"])
def calibrate(
    distance_cm: float = 30.0,
    persist_to_yaml: bool = True,
    ema_alpha: float = DEFAULT_EMA_ALPHA,
    calibration_sets=None,
    exclude_ir_sensors=None,
) -> CalibrateDistance:
    """
    Run a unified distance and IR sensor calibration.

    This is the recommended all-in-one calibration entry point. It drives
    the robot a known distance, prompts the user to measure the actual
    distance traveled, then adjusts the per-wheel ``ticks_to_rad`` values
    to correct odometry. After distance calibration, it automatically
    calibrates IR sensors by sampling them during a drive over the
    calibration surface(s).

    Calibration values are persisted to ``raccoon.project.yml`` using an
    exponential moving average (EMA), so the baseline converges toward the
    true value over multiple calibration runs. The EMA formula is:
    ``new_baseline = old_baseline * alpha + measured * (1 - alpha)``.

    Prerequisites:
        The robot must have drive motors with encoder feedback and
        a configured kinematics model. For IR calibration, IR sensors
        must be registered in ``robot.defs.analog_sensors``.

    Args:
        distance_cm: Distance (in cm) the robot drives during calibration.
            Default is 30 cm. Longer distances yield more accurate results.
        persist_to_yaml: If ``True``, write the EMA-filtered baseline
            back to ``raccoon.project.yml`` so it persists across runs.
        ema_alpha: EMA smoothing coefficient between 0.0 and 1.0. Higher
            values produce slower convergence but a more stable baseline.
            With the default of 0.7, approximately 83% of systematic error
            is absorbed after 5 calibration runs.
        calibration_sets: List of named IR calibration surface sets to
            run (e.g. ``["default", "transparent"]``). Each set beyond
            the first triggers an additional drive-and-sample cycle.
            Defaults to ``["default"]``.
        exclude_ir_sensors: List of ``IRSensor`` instances to skip during
            IR calibration (e.g. sensors not mounted near the ground).

    Returns:
        CalibrateDistance: A step that performs the full calibration flow.

    Example::

        from libstp.step.calibration import calibrate

        # Basic calibration with defaults (30cm drive, persist to YAML)
        seq([
            calibrate(),
            drive_forward(100),
        ])

        # Custom calibration: longer drive, two IR surface sets
        calibrate(
            distance_cm=50.0,
            calibration_sets=["default", "transparent"],
            ema_alpha=0.8,
        )
    """
    return CalibrateDistance(
        distance_cm,
        calibrate_light_sensors=True,
        persist_to_yaml=persist_to_yaml,
        ema_alpha=ema_alpha,
        calibration_sets=calibration_sets,
        exclude_ir_sensors=exclude_ir_sensors,
    )
