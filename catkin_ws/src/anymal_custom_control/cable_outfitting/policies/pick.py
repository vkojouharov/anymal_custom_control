#!/usr/bin/env python3
"""Standalone pick-cable autonomous policy."""

from dataclasses import dataclass
from typing import Optional

import numpy as np


STAGE1 = 1
STAGE2 = 2
STAGE3 = 3
STAGE4 = 4
STAGE5 = 5

# Change this one line to select the last autonomous stage that may run.
FINAL_STAGE = STAGE5

# TARGET is the desired tag position in the camera frame [x, y, z], in metres.
# Stage 1 only centers the tag while approaching its target position. Stages
# 2-4 also control tag orientation using their TARGET_ROTATION matrices.

# -------------------- STAGE 1 CONSTANTS --------------------
STAGE1_TARGET = np.array([0.0, -0.008, 0.30], dtype=float)
STAGE1_POS_TOLERANCE = 0.015
STAGE1_KP_POS = 1.0
STAGE1_KP_ROT = 2.0
STAGE1_MAX_POS_SPEED = 0.1
STAGE1_MAX_ROT_SPEED = 0.5

# -------------------- STAGE 2 CONSTANTS --------------------
STAGE2_TARGET = np.array([0.0, -0.008, 0.20], dtype=float)
STAGE2_TARGET_ROTATION = np.eye(3, dtype=float)
STAGE2_POS_TOLERANCE = 0.01
STAGE2_ANGLE_TOLERANCE = np.deg2rad(5.0)
STAGE2_KP_POS = 0.5
STAGE2_KP_ROT = 0.5
STAGE2_MAX_POS_SPEED = 0.05
STAGE2_MAX_ROT_SPEED = 0.5
STAGE2_STABLE_TIME = 0.5

# -------------------- STAGE 3 CONSTANTS --------------------
STAGE3_TARGET = np.array([0.0, -0.008, 0.17], dtype=float)
# STAGE3_TARGET = np.array([0.0, -0.010, 0.155], dtype=float)
STAGE3_TARGET_ROTATION = np.eye(3, dtype=float)
# STAGE3_POS_TOLERANCE = 0.002
STAGE3_POS_TOLERANCE = 0.003
# STAGE3_ANGLE_TOLERANCE = np.deg2rad(2.0)
STAGE3_ANGLE_TOLERANCE = np.deg2rad(3.0)
STAGE3_KP_POS = 0.5
STAGE3_KP_ROT = 0.5
STAGE3_MAX_POS_SPEED = 0.02
STAGE3_MAX_ROT_SPEED = 0.2
# STAGE3_STABLE_TIME = 1.0
STAGE3_STABLE_TIME = 0.5

# -------------------- STAGE 4 CONSTANTS --------------------
# Stage 4 initially duplicates stage 3, but every value is independent so this
# stage can be tuned without changing stage 3.
STAGE4_TARGET = np.array([0.0, -0.008, 0.135], dtype=float)
# STAGE4_TARGET = np.array([0.0, -0.008, 0.14], dtype=float)
STAGE4_TARGET_ROTATION = np.eye(3, dtype=float)
STAGE4_POS_TOLERANCE = 0.0075
# STAGE4_POS_TOLERANCE = 0.005
STAGE4_ANGLE_TOLERANCE = np.deg2rad(45.0)
STAGE4_KP_POS = 0.5
STAGE4_KP_ROT = 0.5
STAGE4_MAX_POS_SPEED = 0.02
STAGE4_MAX_ROT_SPEED = 0.2
STAGE4_STABLE_TIME = 0.1

# -------------------- STAGE 5 CONSTANTS --------------------
# Close the gripper, wait for the position-controlled gripper to move, then
# reverse along end-effector -Z captured at stage-5 entry.
STAGE5_GRIPPER_CLOSED = True
STAGE5_GRIPPER_ACTION_WAIT = 1.0
STAGE5_REVERSE_DIRECTION_GRIPPER = np.array([0.0, 0.0, -1.0], dtype=float)
STAGE5_REVERSE_DISTANCE = 0.300
STAGE5_REVERSE_SPEED = 0.1

# -------------------- SHARED SENSOR AND TOOL CONSTANTS --------------------
TAG_TIMEOUT_SEC = 0.25

# Camera +X = gripper -X, camera +Y = gripper -Y, camera +Z = gripper +Z.
# Columns are camera +X, +Y, +Z expressed in the gripper frame.
R_GRIPPER_CAMERA = np.array(
    [[-1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, 1.0]],
    dtype=float,
)

# Vector from the kinematic endpoint/gripper origin to the camera optical
# center, expressed in the gripper frame [x, y, z], in metres.
P_GRIPPER_CAMERA_M = np.array([0.0, 0.030, -0.015], dtype=float)


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
    """Pick-cable autonomous policy."""

    def __init__(self) -> None:
        if FINAL_STAGE not in (STAGE1, STAGE2, STAGE3, STAGE4, STAGE5):
            raise ValueError(f"invalid FINAL_STAGE: {FINAL_STAGE}")
        self.stage = STAGE1
        self.finished = False
        self._pose_stable_time_sec = 0.0
        self._reset_pose_errors()
        self._reset_stage5()

    def reset(self) -> None:
        self.stage = STAGE1
        self.finished = False
        self._pose_stable_time_sec = 0.0
        self._reset_pose_errors()
        self._reset_stage5()

    @staticmethod
    def _zero() -> PolicyCommand:
        return PolicyCommand(np.zeros(6, dtype=float), None)

    @staticmethod
    def _limit_norm(vector: np.ndarray, maximum: float) -> np.ndarray:
        norm = float(np.linalg.norm(vector))
        return vector if norm <= maximum else vector * (maximum / norm)

    def _reset_stage5(self) -> None:
        self._gripper_action_elapsed_sec = 0.0
        self._reverse_axis_base = None
        self._reverse_start_position_base = None

    def _reset_pose_errors(self) -> None:
        self._position_error_camera = None
        self._rotation_error_camera = None

    def get_pose_errors(self):
        position_error = (
            None
            if self._position_error_camera is None
            else self._position_error_camera.copy()
        )
        rotation_error = (
            None
            if self._rotation_error_camera is None
            else self._rotation_error_camera.copy()
        )
        return position_error, rotation_error

    @staticmethod
    def _tag_ready(observation: PolicyObservation) -> bool:
        return not (
            not observation.camera_available
            or not observation.tag_visible
            or observation.T_camera_tag is None
            or observation.tag_age_sec > TAG_TIMEOUT_SEC
        )

    @staticmethod
    def _rotation_vector(rotation: np.ndarray) -> np.ndarray:
        """Return Log(R)^vee with stable handling near zero and pi."""
        rotation = np.asarray(rotation, dtype=float).reshape(3, 3)
        u, _, vh = np.linalg.svd(rotation)
        rotation = u @ vh
        if np.linalg.det(rotation) < 0.0:
            u[:, -1] *= -1.0
            rotation = u @ vh

        cosine = float(np.clip((np.trace(rotation) - 1.0) / 2.0, -1.0, 1.0))
        angle = float(np.arccos(cosine))
        skew_vector = np.array(
            [
                rotation[2, 1] - rotation[1, 2],
                rotation[0, 2] - rotation[2, 0],
                rotation[1, 0] - rotation[0, 1],
            ],
            dtype=float,
        )
        if angle < 1e-7:
            return 0.5 * skew_vector
        if np.pi - angle < 1e-5:
            eigenvalues, eigenvectors = np.linalg.eig(rotation)
            index = int(np.argmin(np.abs(eigenvalues - 1.0)))
            axis = np.real(eigenvectors[:, index])
            axis_norm = float(np.linalg.norm(axis))
            if axis_norm <= 1e-9:
                return np.zeros(3, dtype=float)
            axis /= axis_norm
            dominant = int(np.argmax(np.abs(axis)))
            if axis[dominant] * skew_vector[dominant] < 0.0:
                axis = -axis
            return angle * axis
        return (angle / (2.0 * np.sin(angle))) * skew_vector

    def _finish_or_advance(self, completed_stage: int) -> None:
        self._reset_pose_errors()
        if completed_stage >= FINAL_STAGE:
            self.finished = True
            print(f"stage {completed_stage} complete; holding at configured FINAL_STAGE")
            return
        self.stage = completed_stage + 1
        self._pose_stable_time_sec = 0.0
        if self.stage == STAGE5:
            self._reset_stage5()
        print(f"stage {completed_stage} complete; advancing to stage {self.stage}")

    def _step_pose_servo(
        self,
        observation: PolicyObservation,
        target_position_camera: np.ndarray,
        target_rotation_camera_tag: np.ndarray,
        completed_stage: int,
        position_tolerance_m: float,
        orientation_tolerance_rad: float,
        position_kp: float,
        rotation_kp: float,
        max_position_speed: float,
        max_rotation_speed: float,
        stable_time_sec: float,
    ) -> PolicyCommand:
        if not self._tag_ready(observation):
            self._pose_stable_time_sec = 0.0
            self._reset_pose_errors()
            return self._zero()

        transform_camera_tag = np.asarray(observation.T_camera_tag, dtype=float)
        tag_position_camera = transform_camera_tag[:3, 3]
        tag_rotation_camera = transform_camera_tag[:3, :3]
        position_error_camera = tag_position_camera - target_position_camera
        rotation_error_camera = (
            tag_rotation_camera @ target_rotation_camera_tag.T
        )
        orientation_error_camera = self._rotation_vector(rotation_error_camera)
        self._position_error_camera = position_error_camera.copy()
        self._rotation_error_camera = orientation_error_camera.copy()

        position_aligned = (
            float(np.linalg.norm(position_error_camera))
            <= position_tolerance_m
        )
        orientation_aligned = (
            float(np.linalg.norm(orientation_error_camera))
            <= orientation_tolerance_rad
        )
        if position_aligned and orientation_aligned:
            self._pose_stable_time_sec += max(0.0, float(observation.dt_sec))
            if self._pose_stable_time_sec >= stable_time_sec:
                self._finish_or_advance(completed_stage)
                return self._zero()
        else:
            self._pose_stable_time_sec = 0.0

        angular_camera = self._limit_norm(
            rotation_kp * orientation_error_camera,
            max_rotation_speed,
        )
        # This cross term translates around the tag while rotating, reducing
        # the tendency for orientation correction to disturb tag position.
        linear_camera = self._limit_norm(
            position_kp * position_error_camera
            - np.cross(angular_camera, tag_position_camera),
            max_position_speed,
        )

        transform_base_gripper = np.asarray(observation.T_base_tool, dtype=float)
        rotation_base_gripper = transform_base_gripper[:3, :3]
        rotation_base_camera = rotation_base_gripper @ R_GRIPPER_CAMERA
        angular_base = rotation_base_camera @ angular_camera
        camera_linear_base = rotation_base_camera @ linear_camera

        # The Jacobian controls the gripper origin, while the visual command is
        # the desired camera-origin velocity. Account for the measured lever arm:
        # v_camera = v_gripper + omega x r_gripper_to_camera.
        offset_base = rotation_base_gripper @ P_GRIPPER_CAMERA_M
        gripper_linear_base = camera_linear_base - np.cross(angular_base, offset_base)
        task_velocity = np.concatenate((gripper_linear_base, angular_base))
        return PolicyCommand(task_velocity_base=task_velocity, gripper_closed=None)

    def _step_stage1(self, observation: PolicyObservation) -> PolicyCommand:
        if not self._tag_ready(observation):
            self._reset_pose_errors()
            return self._zero()

        tag_position_camera = np.asarray(observation.T_camera_tag, dtype=float)[:3, 3]
        position_error_camera = tag_position_camera - STAGE1_TARGET
        self._position_error_camera = position_error_camera.copy()
        self._rotation_error_camera = None
        if np.all(np.abs(position_error_camera) <= STAGE1_POS_TOLERANCE):
            self._finish_or_advance(STAGE1)
            return self._zero()

        linear_camera = self._limit_norm(
            STAGE1_KP_POS * position_error_camera,
            STAGE1_MAX_POS_SPEED,
        )

        x, y, z = tag_position_camera
        horizontal_error = np.arctan2(x, z)
        vertical_error = np.arctan2(y, z)
        angular_camera = self._limit_norm(
            STAGE1_KP_ROT
            * np.array([-vertical_error, horizontal_error, 0.0], dtype=float),
            STAGE1_MAX_ROT_SPEED,
        )

        rotation_base_gripper = np.asarray(observation.T_base_tool, dtype=float)[:3, :3]
        rotation_base_camera = rotation_base_gripper @ R_GRIPPER_CAMERA
        task_velocity = np.concatenate(
            (rotation_base_camera @ linear_camera, rotation_base_camera @ angular_camera)
        )
        return PolicyCommand(task_velocity_base=task_velocity, gripper_closed=None)

    def _step_stage2(self, observation: PolicyObservation) -> PolicyCommand:
        return self._step_pose_servo(
            observation,
            STAGE2_TARGET,
            STAGE2_TARGET_ROTATION,
            STAGE2,
            STAGE2_POS_TOLERANCE,
            STAGE2_ANGLE_TOLERANCE,
            STAGE2_KP_POS,
            STAGE2_KP_ROT,
            STAGE2_MAX_POS_SPEED,
            STAGE2_MAX_ROT_SPEED,
            STAGE2_STABLE_TIME,
        )

    def _step_stage3(self, observation: PolicyObservation) -> PolicyCommand:
        return self._step_pose_servo(
            observation,
            STAGE3_TARGET,
            STAGE3_TARGET_ROTATION,
            STAGE3,
            STAGE3_POS_TOLERANCE,
            STAGE3_ANGLE_TOLERANCE,
            STAGE3_KP_POS,
            STAGE3_KP_ROT,
            STAGE3_MAX_POS_SPEED,
            STAGE3_MAX_ROT_SPEED,
            STAGE3_STABLE_TIME,
        )

    def _step_stage4(self, observation: PolicyObservation) -> PolicyCommand:
        return self._step_pose_servo(
            observation,
            STAGE4_TARGET,
            STAGE4_TARGET_ROTATION,
            STAGE4,
            STAGE4_POS_TOLERANCE,
            STAGE4_ANGLE_TOLERANCE,
            STAGE4_KP_POS,
            STAGE4_KP_ROT,
            STAGE4_MAX_POS_SPEED,
            STAGE4_MAX_ROT_SPEED,
            STAGE4_STABLE_TIME,
        )

    def _step_stage5(
        self,
        observation: PolicyObservation,
    ) -> PolicyCommand:
        self._reset_pose_errors()
        transform_base_gripper = np.asarray(observation.T_base_tool, dtype=float)
        rotation_base_gripper = transform_base_gripper[:3, :3]

        if self._reverse_axis_base is None:
            reverse_direction_gripper = np.asarray(
                STAGE5_REVERSE_DIRECTION_GRIPPER,
                dtype=float,
            ).reshape(3)
            reverse_axis = rotation_base_gripper @ reverse_direction_gripper
            reverse_axis_norm = float(np.linalg.norm(reverse_axis))
            if reverse_axis_norm <= 1e-9:
                raise ValueError("end-effector reverse direction has zero length")
            self._reverse_axis_base = reverse_axis / reverse_axis_norm

        # Close the gripper and hold still for the configured action delay.
        if self._gripper_action_elapsed_sec < STAGE5_GRIPPER_ACTION_WAIT:
            self._gripper_action_elapsed_sec += max(0.0, float(observation.dt_sec))
            return PolicyCommand(
                np.zeros(6, dtype=float),
                STAGE5_GRIPPER_CLOSED,
            )

        current_position_base = transform_base_gripper[:3, 3]
        if self._reverse_start_position_base is None:
            self._reverse_start_position_base = current_position_base.copy()
            print(
                "gripper action wait complete; starting "
                f"{STAGE5_REVERSE_DISTANCE * 1000.0:.0f} mm reverse along "
                "the configured end-effector direction"
            )
            return PolicyCommand(
                np.zeros(6, dtype=float),
                STAGE5_GRIPPER_CLOSED,
            )

        displacement_base = current_position_base - self._reverse_start_position_base
        reverse_progress_m = float(
            np.dot(displacement_base, self._reverse_axis_base)
        )
        if reverse_progress_m >= STAGE5_REVERSE_DISTANCE:
            self._finish_or_advance(STAGE5)
            return PolicyCommand(
                np.zeros(6, dtype=float),
                STAGE5_GRIPPER_CLOSED,
            )

        task_velocity = np.zeros(6, dtype=float)
        task_velocity[:3] = STAGE5_REVERSE_SPEED * self._reverse_axis_base
        return PolicyCommand(task_velocity, STAGE5_GRIPPER_CLOSED)

    def step(self, observation: PolicyObservation) -> PolicyCommand:
        if self.finished:
            return self._zero()
        if self.stage == STAGE1:
            return self._step_stage1(observation)
        if self.stage == STAGE2:
            return self._step_stage2(observation)
        if self.stage == STAGE3:
            return self._step_stage3(observation)
        if self.stage == STAGE4:
            return self._step_stage4(observation)
        if self.stage == STAGE5:
            return self._step_stage5(observation)
        return self._zero()
