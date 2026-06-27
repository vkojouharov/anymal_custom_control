"""Camera-relative base servo policy for AprilTag approach."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .constants import (
    LATERAL_KP,
    RANGE_KP,
    TARGET_LATERAL_TOLERANCE_M,
    TARGET_RANGE_TOLERANCE_M,
    TARGET_YAW_TOLERANCE_RAD,
    YAW_KP,
)
from .messages import TagPose


@dataclass(frozen=True)
class ServoCommand:
    heading: float
    lateral: float
    turning: float
    range_error_m: float
    lateral_error_m: float
    yaw_error_rad: float
    target_reached: bool


@dataclass(frozen=True)
class ServoLimits:
    max_heading: float
    max_lateral: float
    max_turning: float


def compute_servo_command(
    tag: TagPose,
    *,
    target_distance_m: float,
    limits: ServoLimits,
) -> ServoCommand:
    """Compute normalized joy-manager base commands from camera-frame tag pose.

    OpenCV camera frame is +X right, +Y down, +Z forward. MovementController
    uses heading +forward, lateral +left, turning +CCW/left. Therefore positive
    camera X drives negative lateral and negative turning.
    """

    range_error_m = tag.forward_m - target_distance_m
    lateral_error_m = tag.lateral_right_m
    yaw_error_rad = tag.bearing_rad

    heading = _clip(RANGE_KP * range_error_m, limits.max_heading)
    lateral = _clip(-LATERAL_KP * lateral_error_m, limits.max_lateral)
    turning = _clip(-YAW_KP * yaw_error_rad, limits.max_turning)

    target_reached = (
        abs(range_error_m) <= TARGET_RANGE_TOLERANCE_M
        and abs(lateral_error_m) <= TARGET_LATERAL_TOLERANCE_M
        and abs(yaw_error_rad) <= TARGET_YAW_TOLERANCE_RAD
    )
    if target_reached:
        heading = 0.0
        lateral = 0.0
        turning = 0.0

    return ServoCommand(
        heading=float(heading),
        lateral=float(lateral),
        turning=float(turning),
        range_error_m=float(range_error_m),
        lateral_error_m=float(lateral_error_m),
        yaw_error_rad=float(yaw_error_rad),
        target_reached=target_reached,
    )


def _clip(value: float, limit: float) -> float:
    return float(np.clip(float(value), -abs(float(limit)), abs(float(limit))))
