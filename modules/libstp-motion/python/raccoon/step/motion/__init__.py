"""Contributor-facing motion step DSL built on top of the native motion controllers."""

from __future__ import annotations

from .motion_step import MotionStep
from .drive import (
    DriveForward,
    DriveBackward,
    StrafeLeft,
    StrafeRight,
)
from .drive_dsl import (
    drive_forward,
    drive_backward,
    strafe_left,
    strafe_right,
)
from .drive_angle import DriveAngle, DriveAngleLeft, DriveAngleRight
from .drive_angle_dsl import drive_angle
from .arc import (
    Arc,
    DriveArcLeft,
    DriveArcRight,
    DriveArc,
    StrafeArcLeft,
    StrafeArcRight,
    StrafeArc,
)
from .arc_dsl import drive_arc_left, drive_arc_right, strafe_arc_left, strafe_arc_right
from .arc_segment import drive_arc_segment
from .stop import Stop
from .stop_dsl import stop
from .turn import TurnLeft, TurnRight
from .turn_dsl import turn_left, turn_right
from .heading_reference import (
    MarkHeadingReference,
    turn_to_heading_right,
    turn_to_heading_left,
)
from .heading_reference_dsl import mark_heading_reference
from .lineup import (
    SurfaceColor,
    lineup,
    forward_lineup_on_black,
    forward_lineup_on_white,
    backward_lineup_on_black,
    backward_lineup_on_white,
    strafe_right_lineup_on_black,
    strafe_right_lineup_on_white,
    strafe_left_lineup_on_black,
    strafe_left_lineup_on_white,
    CorrectionSide,
    SingleSensorCrossing,
    SingleSensorLineupConfig,
    forward_single_lineup,
    backward_single_lineup,
)
from .wall_align import (
    WallAlign,
    WallDirection,
    BumpResult,
    WallAlignForward,
    WallAlignBackward,
    WallAlignStrafeLeft,
    WallAlignStrafeRight,
)
from .wall_align_dsl import (
    wall_align_forward,
    wall_align_backward,
    wall_align_strafe_left,
    wall_align_strafe_right,
)
from .line_follow import (
    LineFollow,
    LineFollowConfig,
    LineSide,
    SingleSensorLineFollow,
    SingleLineFollowConfig,
    FollowLine,
    FollowLineSingle,
    DirectionalLineFollow,
    DirectionalLineFollowConfig,
    DirectionalSingleLineFollow,
    DirectionalSingleLineFollowConfig,
    DirectionalFollowLine,
    StrafeFollowLine,
    StrafeFollowLineSingle,
    DirectionalFollowLineSingle,
)
from .line_follow_dsl import (
    follow_line,
    follow_line_single,
    directional_follow_line,
    strafe_follow_line,
    strafe_follow_line_single,
    directional_follow_line_single,
)

from .at_distance import WaitUntilDistance
from .at_distance_dsl import wait_until_distance
from .at_heading import WaitUntilDegrees, HeadingOrigin
from .at_heading_dsl import wait_until_degrees
from .tune_drive import TuneDrive
from .tune_drive_dsl import tune_drive
from .characterize_drive import CharacterizeDrive
from .characterize_drive_dsl import characterize_drive
from .auto_tune import (
    AutoTune,
    AutoTuneVelocity,
    AutoTuneMotion,
)
from .auto_tune_dsl import (
    auto_tune,
    auto_tune_velocity,
    auto_tune_motion,
)
from .sensor_group import SensorGroup
from .drive_to_analog_target import DriveToAnalogTarget
from .drive_to_analog_target_dsl import drive_to_analog_target
from .custom_velocity import CustomVelocity
from .custom_velocity_dsl import CustomVelocityBuilder, custom_velocity
from .smooth_path import SmoothPath, smooth_path
from .spline_path import SplinePath, spline

# Public API: only factories, plus enums / config dataclasses that users
# legitimately need to construct (LineFollowConfig, SurfaceColor, etc.) and
# the MotionStep base class for users who write their own motion steps.
#
# Step *classes* (DriveForward, StrafeLeft, ...) are intentionally NOT
# exported — users construct them through their factory. Re-exporting the
# class lets callers bypass the factory's parameter validation and unit
# conversion, then breaks them when an internal refactor renames the field.
__all__ = [
    # Base class (extension point)
    "MotionStep",
    # Basic motion factories
    "drive_forward",
    "drive_backward",
    "stop",
    "strafe_left",
    "strafe_right",
    "drive_angle",
    "drive_arc_left",
    "drive_arc_right",
    "drive_arc_segment",
    "strafe_arc_left",
    "strafe_arc_right",
    "turn_left",
    "turn_right",
    # Heading reference
    "mark_heading_reference",
    "turn_to_heading_right",
    "turn_to_heading_left",
    # Lineup
    "SurfaceColor",
    "CorrectionSide",
    "SingleSensorLineupConfig",
    "lineup",
    "forward_lineup_on_black",
    "forward_lineup_on_white",
    "backward_lineup_on_black",
    "backward_lineup_on_white",
    "strafe_right_lineup_on_black",
    "strafe_right_lineup_on_white",
    "strafe_left_lineup_on_black",
    "strafe_left_lineup_on_white",
    "forward_single_lineup",
    "backward_single_lineup",
    # Wall align
    "WallDirection",
    "BumpResult",
    "wall_align_forward",
    "wall_align_backward",
    "wall_align_strafe_left",
    "wall_align_strafe_right",
    # Line follow
    "LineFollowConfig",
    "LineSide",
    "SingleLineFollowConfig",
    "follow_line",
    "follow_line_single",
    # Directional line follow
    "DirectionalLineFollowConfig",
    "DirectionalSingleLineFollowConfig",
    "directional_follow_line",
    "strafe_follow_line",
    "strafe_follow_line_single",
    "directional_follow_line_single",
    # Distance wait
    "wait_until_distance",
    # Heading wait
    "HeadingOrigin",
    "wait_until_degrees",
    # Drive telemetry
    "tune_drive",
    # Drive characterization
    "characterize_drive",
    # Auto-tune PID
    "auto_tune",
    "auto_tune_velocity",
    "auto_tune_motion",
    # Sensor group
    "SensorGroup",
    # Analog sensor target drive
    "drive_to_analog_target",
    # Custom velocity
    "CustomVelocityBuilder",
    "custom_velocity",
    # Smooth path
    "smooth_path",
    # Spline path
    "spline",
]
