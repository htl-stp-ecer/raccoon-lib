"""Contributor-facing motion step DSL built on top of the native motion controllers."""

from .motion_step import MotionStep
from .drive import (
    DriveForward, DriveBackward, StrafeLeft, StrafeRight,
)
from .drive_dsl import (
    drive_forward, drive_backward, strafe_left, strafe_right,
)
from .drive_angle import DriveAngle
from .drive_angle_dsl import drive_angle
from .arc import Arc, DriveArcLeft, DriveArcRight, DriveArc, StrafeArcLeft, StrafeArcRight, StrafeArc
from .arc_dsl import drive_arc_left, drive_arc_right, drive_arc, strafe_arc_left, strafe_arc_right, strafe_arc
from .stop import Stop
from .stop_dsl import stop
from .turn import TurnLeft, TurnRight
from .turn_dsl import turn_left, turn_right
from .heading_reference import (
    MarkHeadingReference, turn_to_heading_right, turn_to_heading_left,
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
from .at_heading import WaitUntilDegrees
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

__all__ = [
    # Base
    "MotionStep",
    # Basic motion
    "DriveForward",
    "drive_forward",
    "DriveBackward",
    "drive_backward",
    "Stop",
    "stop",
    "StrafeLeft",
    "strafe_left",
    "StrafeRight",
    "strafe_right",
    "DriveAngle",
    "drive_angle",
    "Arc",
    "DriveArcLeft",
    "drive_arc_left",
    "DriveArcRight",
    "drive_arc_right",
    "DriveArc",
    "drive_arc",
    "StrafeArcLeft",
    "strafe_arc_left",
    "StrafeArcRight",
    "strafe_arc_right",
    "StrafeArc",
    "strafe_arc",
    "TurnLeft",
    "turn_left",
    "TurnRight",
    "turn_right",
    # Heading reference
    "MarkHeadingReference",
    "mark_heading_reference",
    "turn_to_heading_right",
    "turn_to_heading_left",
    # Lineup
    "SurfaceColor",
    "lineup",
    "forward_lineup_on_black",
    "forward_lineup_on_white",
    "backward_lineup_on_black",
    "backward_lineup_on_white",
    "strafe_right_lineup_on_black",
    "strafe_right_lineup_on_white",
    "strafe_left_lineup_on_black",
    "strafe_left_lineup_on_white",
    "CorrectionSide",
    "SingleSensorCrossing",
    "SingleSensorLineupConfig",
    "forward_single_lineup",
    "backward_single_lineup",
    # Wall align
    "WallAlign",
    "WallDirection",
    "BumpResult",
    "WallAlignForward",
    "wall_align_forward",
    "WallAlignBackward",
    "wall_align_backward",
    "WallAlignStrafeLeft",
    "wall_align_strafe_left",
    "WallAlignStrafeRight",
    "wall_align_strafe_right",
    # Line follow
    "LineFollow",
    "LineFollowConfig",
    "LineSide",
    "SingleSensorLineFollow",
    "SingleLineFollowConfig",
    "FollowLine",
    "follow_line",
    "FollowLineSingle",
    "follow_line_single",
    # Directional line follow
    "DirectionalLineFollow",
    "DirectionalLineFollowConfig",
    "DirectionalSingleLineFollow",
    "DirectionalSingleLineFollowConfig",
    "DirectionalFollowLine",
    "directional_follow_line",
    "StrafeFollowLine",
    "strafe_follow_line",
    "StrafeFollowLineSingle",
    "strafe_follow_line_single",
    "DirectionalFollowLineSingle",
    "directional_follow_line_single",
    # Distance wait
    "WaitUntilDistance",
    "wait_until_distance",
    # Heading wait
    "WaitUntilDegrees",
    "wait_until_degrees",
    # Drive telemetry
    "TuneDrive",
    "tune_drive",
    # Drive characterization
    "CharacterizeDrive",
    "characterize_drive",
    # Auto-tune PID
    "AutoTune",
    "auto_tune",
    "AutoTuneVelocity",
    "auto_tune_velocity",
    "AutoTuneMotion",
    "auto_tune_motion",
    # Sensor group
    "SensorGroup",
]
