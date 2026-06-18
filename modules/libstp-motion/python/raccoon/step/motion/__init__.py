"""Contributor-facing motion step DSL built on top of the native motion controllers."""

from __future__ import annotations

from raccoon.hal import OdometrySource

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
    LateralFollowLine,
    LateralFollowLineSingle,
    DirectionalFollowLineSingle,
)
from .line_follow_dsl import (
    follow_line,
    follow_line_single,
    directional_follow_line,
    strafe_follow_line,
    strafe_follow_line_single,
    lateral_follow_line,
    lateral_follow_line_single,
    directional_follow_line_single,
)
from .line_follow_builder import (
    ConfigurableLineFollowBuilder,
    FollowCorrection,
    line_follow,
)
from .resync import (
    AlignToWallResync,
    FindLineResync,
    ResyncAtStartPose,
    align_to_wall_resync,
    find_line_resync,
    resync_at_start_pose,
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
    AutoTuneFirmwarePid,
    AutoTuneStaticFriction,
    AutoTuneVelLpf,
)
from .auto_tune_dsl import (
    auto_tune,
    auto_tune_velocity,
    auto_tune_motion,
    auto_tune_firmware_pid,
    auto_tune_static_friction,
    auto_tune_vel_lpf,
    auto_tune_bemf_velocity,
)
from .sensor_group import SensorGroup
from .drive_to_analog_target import DriveToAnalogTarget
from .drive_to_analog_target_dsl import drive_to_analog_target
from .custom_velocity import CustomVelocity
from .custom_velocity_dsl import CustomVelocityBuilder, custom_velocity
from .goto import goto
from .smooth_path import SmoothPath, smooth_path
from .path.optimize import Optimizer, PathBuildError, optimize
from .spline_path import SplinePath, spline
from .set_odometry_source import SetOdometrySource
from .set_odometry_source_dsl import set_odometry_source
from .set_speed_mode import SetSpeedMode
from .set_speed_mode_dsl import set_speed_mode

# Public API: only factories, plus enums / config dataclasses that users
# legitimately need to construct (LineFollowConfig, SurfaceColor, etc.) and
# the MotionStep base class for users who write their own motion steps.
#
# Step *classes* (DriveForward, StrafeLeft, ...) are intentionally NOT
# exported — users construct them through their factory. Re-exporting the
# class lets callers bypass the factory's parameter validation and unit
# conversion, then breaks them when an internal refactor renames the field.
__all__ = [
    "BumpResult",
    "ConfigurableLineFollowBuilder",
    "CorrectionSide",
    # Custom velocity
    "CustomVelocityBuilder",
    # Directional line follow
    "DirectionalLineFollowConfig",
    "DirectionalSingleLineFollowConfig",
    "FollowCorrection",
    # Heading wait
    "HeadingOrigin",
    # Line follow
    "LineFollowConfig",
    "LineSide",
    # Base class (extension point)
    "MotionStep",
    # Odometry source selection
    "OdometrySource",
    # Sensor group
    "SensorGroup",
    "SingleLineFollowConfig",
    "SingleSensorLineupConfig",
    # Lineup
    "SurfaceColor",
    # Wall align
    "WallDirection",
    # Localization resync
    "align_to_wall_resync",
    # Auto-tune PID
    "auto_tune",
    "auto_tune_bemf_velocity",
    "auto_tune_firmware_pid",
    "auto_tune_motion",
    "auto_tune_static_friction",
    "auto_tune_vel_lpf",
    "auto_tune_velocity",
    "backward_lineup_on_black",
    "backward_lineup_on_white",
    "backward_single_lineup",
    # Drive characterization
    "characterize_drive",
    "custom_velocity",
    "directional_follow_line",
    "directional_follow_line_single",
    "drive_angle",
    "drive_arc_left",
    "drive_arc_right",
    "drive_arc_segment",
    "drive_backward",
    # Basic motion factories
    "drive_forward",
    # Analog sensor target drive
    "drive_to_analog_target",
    "find_line_resync",
    "follow_line",
    "follow_line_single",
    "forward_lineup_on_black",
    "forward_lineup_on_white",
    "forward_single_lineup",
    # Closed-loop navigate-to-pose
    "goto",
    "lateral_follow_line",
    "lateral_follow_line_single",
    "line_follow",
    "lineup",
    # Heading reference
    "mark_heading_reference",
    "resync_at_start_pose",
    "set_odometry_source",
    # SpeedMode (BEMF on/off)
    "set_speed_mode",
    # Smooth path
    "smooth_path",
    # Fluent path optimizer
    "Optimizer",
    "PathBuildError",
    "optimize",
    # Spline path
    "spline",
    "stop",
    "strafe_arc_left",
    "strafe_arc_right",
    "strafe_follow_line",
    "strafe_follow_line_single",
    "strafe_left",
    "strafe_left_lineup_on_black",
    "strafe_left_lineup_on_white",
    "strafe_right",
    "strafe_right_lineup_on_black",
    "strafe_right_lineup_on_white",
    # Drive telemetry
    "tune_drive",
    "turn_left",
    "turn_right",
    "turn_to_heading_left",
    "turn_to_heading_right",
    "wait_until_degrees",
    # Distance wait
    "wait_until_distance",
    "wall_align_backward",
    "wall_align_forward",
    "wall_align_strafe_left",
    "wall_align_strafe_right",
]
