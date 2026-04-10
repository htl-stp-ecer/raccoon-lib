"""
Pre-built reusable UI screens.
"""

from .basic import (
    WaitForButtonScreen,
    ConfirmScreen,
    MessageScreen,
    ChoiceScreen,
    ProgressScreen,
    StatusScreen,
)

from .input import (
    NumberInputScreen,
    NumberInputResult,
    TextInputScreen,
    TextInputResult,
    SliderInputScreen,
    SliderInputResult,
)

from .wfl import (
    WFLMeasureScreen,
    WFLConfirmScreen,
    WFLMeasureResult,
    WFLConfirmResult,
    WFLDetectScreen,
)

from .distance import (
    DistancePrepareScreen,
    DistanceDrivingScreen,
    DistanceMeasureScreen,
    DistanceConfirmScreen,
    DistanceConfirmResult,
)

__all__ = [
    # Basic
    "WaitForButtonScreen",
    "ConfirmScreen",
    "MessageScreen",
    "ChoiceScreen",
    "ProgressScreen",
    "StatusScreen",
    # Input
    "NumberInputScreen",
    "NumberInputResult",
    "TextInputScreen",
    "TextInputResult",
    "SliderInputScreen",
    "SliderInputResult",
    # WFL Calibration
    "WFLMeasureScreen",
    "WFLConfirmScreen",
    "WFLMeasureResult",
    "WFLConfirmResult",
    # Distance Calibration
    "DistancePrepareScreen",
    "DistanceDrivingScreen",
    "DistanceMeasureScreen",
    "DistanceConfirmScreen",
    "DistanceConfirmResult",
]
