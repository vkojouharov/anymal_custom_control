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

    @property
    def face_normal_camera(self) -> Optional[tuple[float, float, float]]:
        if self.rotation_camera_tag is None:
            return None
        normal_camera = self.rotation_camera_tag[:, 2]
        return (
            float(normal_camera[0]),
            float(normal_camera[1]),
            float(normal_camera[2]),
        )

    @property
    def face_yaw_error_rad(self) -> Optional[float]:
        normal_camera = self.face_normal_camera
        if normal_camera is None:
            return None
        return math.atan2(normal_camera[0], normal_camera[2])


def rotate_tag_pose_camera_180(tag: TagPose) -> TagPose:
    """Express a tag pose in a camera frame rotated 180 degrees about +Z."""
    camera_rotation = np.diag([-1.0, -1.0, 1.0])
    rotation_camera_tag = tag.rotation_camera_tag
    if rotation_camera_tag is not None:
        rotation_camera_tag = camera_rotation @ rotation_camera_tag
    return TagPose(
        tag_id=tag.tag_id,
        stamp_sec=tag.stamp_sec,
        decision_margin=tag.decision_margin,
        position_camera_m=camera_rotation @ tag.position_camera_m,
        rotation_camera_tag=rotation_camera_tag,
        tag_size_m=tag.tag_size_m,
    )


@dataclass(frozen=True)
class OdomPose:
    stamp_sec: float
    x: float
    y: float
    yaw: float


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


def tag_pose_tag_aligned_xy(current: TagPose, origin: TagPose) -> Optional[tuple[float, float]]:
    if current.rotation_camera_tag is None or origin.rotation_camera_tag is None:
        return None
    camera_motion_c0 = (
        origin.position_camera_m
        - origin.rotation_camera_tag @ current.rotation_camera_tag.T @ current.position_camera_m
    )
    return camera_vector_to_initial_tag_frame_xy(camera_motion_c0, origin)


def odom_tag_aligned_xy(odom_rel_x: float, odom_rel_y: float, origin: TagPose) -> Optional[tuple[float, float]]:
    basis = initial_tag_frame_basis(origin)
    if basis is None:
        return None
    into_tag, tag_left = basis
    odom_vec = np.asarray([float(odom_rel_x), float(odom_rel_y)], dtype=float)
    return (float(np.dot(odom_vec, into_tag)), float(np.dot(odom_vec, tag_left)))


def initial_tag_center_xy(origin: TagPose) -> Optional[tuple[float, float]]:
    return camera_vector_to_initial_tag_frame_xy(origin.position_camera_m, origin)


def camera_vector_to_initial_tag_frame_xy(vector_camera_m: np.ndarray, origin: TagPose) -> Optional[tuple[float, float]]:
    basis = initial_tag_frame_basis(origin)
    if basis is None:
        return None
    into_tag, tag_left = basis
    topdown_vec = camera_topdown_vector(vector_camera_m)
    return (float(np.dot(topdown_vec, into_tag)), float(np.dot(topdown_vec, tag_left)))


def initial_tag_frame_basis(origin: TagPose) -> Optional[tuple[np.ndarray, np.ndarray]]:
    if origin.rotation_camera_tag is None:
        return None
    normal_topdown = camera_topdown_vector(origin.rotation_camera_tag[:, 2])
    normal_norm = float(np.linalg.norm(normal_topdown))
    if normal_norm <= 1e-9 or not math.isfinite(normal_norm):
        return None

    outward = normal_topdown / normal_norm
    camera_to_tag = camera_topdown_vector(origin.position_camera_m)
    if float(np.dot(outward, camera_to_tag)) > 0.0:
        outward = -outward

    into_tag = -outward
    tag_left = np.asarray([-into_tag[1], into_tag[0]], dtype=float)
    return into_tag, tag_left


def camera_topdown_vector(vector_camera_m: np.ndarray) -> np.ndarray:
    return np.asarray(
        [float(vector_camera_m[2]), -float(vector_camera_m[0])],
        dtype=float,
    )


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
