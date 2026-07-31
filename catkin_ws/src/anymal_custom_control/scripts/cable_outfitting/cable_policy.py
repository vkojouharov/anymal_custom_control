#!/usr/bin/env python3
"""Autonomous cable-operation policy interface."""

from dataclasses import dataclass
from typing import Optional

import numpy as np


STAGE_COARSE_APPROACH = 1
STAGE_FINE_APPROACH = 2
STAGE_FINAL_INSERTION = 3

COARSE_TARGET_CAMERA_M = np.array([0.0, 0.0, 0.20], dtype=float)
COARSE_LINEAR_KP = 0.5
COARSE_LINEAR_MAX_M_S = 0.05
COARSE_ANGULAR_KP = 1.0
COARSE_ANGULAR_MAX_RAD_S = 0.25
COARSE_TOLERANCE_M = 0.01
TAG_TIMEOUT_SEC = 0.25

# Columns are camera +X, +Y, +Z expressed in the gripper frame.
R_GRIPPER_CAMERA = np.array(
    [[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]],
    dtype=float,
)


@dataclass(frozen=True)
class PolicyObservation:
    T_camera_tag: Optional[np.ndarray]
    tag_age_sec: float
    tag_visible: bool
    tag_decision_margin: Optional[float]
    camera_available: bool
    joint_position: np.ndarray
    T_base_tool: np.ndarray
    gripper_closed: bool
    dt_sec: float


@dataclass(frozen=True)
class PolicyCommand:
    task_velocity_base: np.ndarray
    gripper_closed: Optional[bool]


class CablePolicy:
    """Three-stage cable-connector policy; only coarse approach is active."""

    def __init__(self) -> None:
        self.stage = STAGE_COARSE_APPROACH

    def reset(self) -> None:
        self.stage = STAGE_COARSE_APPROACH

    @staticmethod
    def _zero() -> PolicyCommand:
        return PolicyCommand(np.zeros(6, dtype=float), None)

    @staticmethod
    def _limit_norm(vector: np.ndarray, maximum: float) -> np.ndarray:
        norm = float(np.linalg.norm(vector))
        return vector if norm <= maximum else vector * (maximum / norm)

    def step(self, observation: PolicyObservation) -> PolicyCommand:
        if self.stage != STAGE_COARSE_APPROACH:
            return self._zero()
        if (
            not observation.camera_available
            or not observation.tag_visible
            or observation.T_camera_tag is None
            or observation.tag_age_sec > TAG_TIMEOUT_SEC
        ):
            return self._zero()

        tag_position_camera = np.asarray(observation.T_camera_tag, dtype=float)[:3, 3]
        error_camera = tag_position_camera - COARSE_TARGET_CAMERA_M
        if np.all(np.abs(error_camera) <= COARSE_TOLERANCE_M):
            self.stage = STAGE_FINE_APPROACH
            print("coarse approach complete; holding for fine-approach policy")
            return self._zero()

        linear_camera = self._limit_norm(
            COARSE_LINEAR_KP * error_camera,
            COARSE_LINEAR_MAX_M_S,
        )

        x, y, z = tag_position_camera
        horizontal_error = np.arctan2(x, z)
        vertical_error = np.arctan2(y, z)
        angular_camera = self._limit_norm(
            COARSE_ANGULAR_KP
            * np.array([-vertical_error, -horizontal_error, 0.0], dtype=float),
            COARSE_ANGULAR_MAX_RAD_S,
        )

        rotation_base_gripper = np.asarray(observation.T_base_tool, dtype=float)[:3, :3]
        rotation_base_camera = rotation_base_gripper @ R_GRIPPER_CAMERA
        task_velocity = np.concatenate(
            # (rotation_base_camera @ linear_camera, rotation_base_camera @ angular_camera)
            (rotation_base_camera @ linear_camera, np.array([0.0, 0.0, 0.0], dtype=float))
        )
        return PolicyCommand(task_velocity_base=task_velocity, gripper_closed=None)
