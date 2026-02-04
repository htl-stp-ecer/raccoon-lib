"""
Unified calibration step (distance + IR sensors).
"""

from libstp.step.annotation import dsl

from .calibrate_distance import CalibrateDistance


@dsl(tags=["calibration"])
def calibrate(distance_cm: float = 30.0) -> CalibrateDistance:
    """
    Create a unified calibration step.

    Runs distance calibration and then IR sensor calibration.

    Args:
        distance_cm: Distance to drive for calibration (default 30cm)

    Returns:
        CalibrateDistance step instance
    """
    return CalibrateDistance(distance_cm, calibrate_light_sensors=True)
