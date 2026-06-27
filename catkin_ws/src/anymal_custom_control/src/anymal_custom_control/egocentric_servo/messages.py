"""Message parsing and small geometry helpers for egocentric visual servoing."""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from typing import Optional

import numpy as np


@dataclass(frozen=True)
class TagPose:
    tag_id: int
    stamp_sec: float
    decision_margin: float
    position_camera_m: np.ndarray
    rotation_camera_tag: Optional[np.ndarray]
    tag_size_m: Optional[float]

    @property
    def range_m(self) -> float:
        return float(np.linalg.norm(self.position_camera_m))

    @property
    def forward_m(self) -> float:
        return float(self.position_camera_m[2])

    @property
    def lateral_right_m(self) -> float:
        return float(self.position_camera_m[0])

    @property
    def bearing_rad(self) -> float:
        return math.atan2(float(self.position_camera_m[0]), float(self.position_camera_m[2]))


@dataclass(frozen=True)
class OdomPose:
    stamp_sec: float
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class ImuQuat:
    stamp_sec: float
    x: float
    y: float
    z: float
    w: float


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_angle_rad(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def parse_tag_detections_json(data: str, *, target_tag_id: int | None = None) -> list[TagPose]:
    payload = json.loads(data)
    stamp_sec = float(payload.get("stamp_sec", 0.0))
    tags = payload.get("tags", [])
    parsed: list[TagPose] = []
    for tag in tags:
        if not isinstance(tag, dict):
            continue
        tag_id = int(tag.get("id", -1))
        if target_tag_id is not None and tag_id != target_tag_id:
            continue
        pose = _finite_vector3(tag.get("pose_t_camera_m"))
        if pose is None:
            continue
        rotation = _finite_matrix3(tag.get("pose_R_camera_tag"))
        parsed.append(
            TagPose(
                tag_id=tag_id,
                stamp_sec=stamp_sec,
                decision_margin=float(tag.get("decision_margin", 0.0)),
                position_camera_m=pose,
                rotation_camera_tag=rotation,
                tag_size_m=_optional_float(tag.get("tag_size_m")),
            )
        )
    parsed.sort(key=lambda item: item.decision_margin, reverse=True)
    return parsed


def odom_relative_xy(current: OdomPose, origin: OdomPose) -> tuple[float, float, float]:
    dx = current.x - origin.x
    dy = current.y - origin.y
    c = math.cos(origin.yaw)
    s = math.sin(origin.yaw)
    rel_x = c * dx + s * dy
    rel_y = -s * dx + c * dy
    rel_yaw = wrap_angle_rad(current.yaw - origin.yaw)
    return rel_x, rel_y, rel_yaw


def _finite_vector3(value: object) -> Optional[np.ndarray]:
    if not isinstance(value, list) or len(value) != 3:
        return None
    try:
        vec = np.asarray([float(item) for item in value], dtype=float)
    except (TypeError, ValueError):
        return None
    if not np.all(np.isfinite(vec)):
        return None
    return vec


def _finite_matrix3(value: object) -> Optional[np.ndarray]:
    if value is None:
        return None
    try:
        mat = np.asarray(value, dtype=float).reshape(3, 3)
    except (TypeError, ValueError):
        return None
    if not np.all(np.isfinite(mat)):
        return None
    return mat


def _optional_float(value: object) -> Optional[float]:
    if value is None:
        return None
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None
