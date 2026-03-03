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
) -> CalibrateDistance:
    """
    Create a unified calibration step.

    Runs distance calibration and then IR sensor calibration.
    Persists calibration to YAML using EMA for convergence over time.

    Args:
        distance_cm: Distance to drive for calibration (default 30cm)
        persist_to_yaml: If True, update raccoon.project.yml with EMA baseline
        ema_alpha: EMA coefficient (0.0-1.0, higher = slower convergence)
        calibration_sets: Named IR calibration sets (e.g. ["default", "transparent"])

    Returns:
        CalibrateDistance step instance
    """
    return CalibrateDistance(
        distance_cm,
        calibrate_light_sensors=True,
        persist_to_yaml=persist_to_yaml,
        ema_alpha=ema_alpha,
        calibration_sets=calibration_sets,
    )
