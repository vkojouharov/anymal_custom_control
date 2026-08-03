#!/usr/bin/env python3
"""Place-cable autonomous policy and shared cable-policy implementation."""

from dataclasses import dataclass
from typing import Optional

import numpy as np


STAGE_COARSE_APPROACH = 1
STAGE_FINE_APPROACH = 2
STAGE_FINAL_APPROACH = 3
STAGE_RELEASE_AND_RETREAT = 4

# Change this one line to select the last autonomous stage that may run.
# Keep coarse as the default so adding/testing later stages cannot change the
# existing autonomous behavior until explicitly enabled.
FINAL_STAGE = STAGE_RELEASE_AND_RETREAT

COARSE_TARGET_CAMERA_M = np.array([0.0, -0.01, 0.3], dtype=float)
COARSE_LINEAR_KP = 1.0
COARSE_LINEAR_MAX_M_S = 0.05
COARSE_ANGULAR_KP = 2.0
COARSE_ANGULAR_MAX_RAD_S = 0.25
COARSE_TOLERANCE_M = 0.005
TAG_TIMEOUT_SEC = 0.25

# -------------------- STAGE 2 FINAL GOAL (EDIT THESE) --------------------
# Desired tag position expressed in the camera frame [x, y, z], in metres.
# This conservative initial value is the same stand-off used by stage 1, so
# enabling stage 2 does not immediately drive closer to the tag.
FINE_TARGET_CAMERA_M = np.array([0.0, -0.008, 0.16], dtype=float)

# Desired tag orientation relative to the camera. This matrix maps vectors
# from the tag frame into the camera frame at the final stage-2 pose. Identity
# means the tag is upright and viewed straight-on under the AprilTag convention.
FINE_TARGET_ROTATION_CAMERA_TAG = np.eye(3, dtype=float)

FINE_LINEAR_KP = 0.5
FINE_LINEAR_MAX_M_S = 0.02
FINE_ANGULAR_KP = 0.5
FINE_ANGULAR_MAX_RAD_S = 0.2
FINE_POSITION_TOLERANCE_M = 0.005
FINE_ORIENTATION_TOLERANCE_RAD = np.deg2rad(3.0)
FINE_STABLE_TIME_SEC = 0.5

# -------------------- STAGE 3 FINAL GOAL (EDIT THIS) --------------------
# Desired tag position in the camera frame [x, y, z], in metres. Replace this
# conservative placeholder before enabling stage 3. Stage 3 deliberately uses
# the same desired orientation, gains, limits, and steering law as stage 2.
FINAL_APPROACH_TARGET_CAMERA_M = np.array([0.0, -0.008, 0.14], dtype=float)

# Stage-3-only criteria for advancing to stage 4. These do not affect stage 2.
FINAL_APPROACH_POSITION_TOLERANCE_M = 0.005
FINAL_APPROACH_ORIENTATION_TOLERANCE_RAD = np.deg2rad(10.0)
FINAL_APPROACH_STABLE_TIME_SEC = 0.5

# -------------------- STAGE 4 RELEASE AND RETREAT --------------------
# The place policy opens the gripper, waits for the position-controlled gripper
# to move, then retreats along end-effector -Z captured at stage-4 entry.
FINAL_GRIPPER_CLOSED = False
GRIPPER_ACTION_WAIT_SEC = 1.0
RETREAT_DIRECTION_GRIPPER = np.array([0.0, 0.0, -1.0], dtype=float)
RETREAT_DISTANCE_M = 0.300
RETREAT_SPEED_M_S = 0.02

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
    """Place policy; subclasses may override targets and final gripper state."""

    # These class attributes let pick/hook policies keep independent editable
    # target poses while reusing this exact steering implementation.
    COARSE_TARGET_CAMERA_M = COARSE_TARGET_CAMERA_M
    FINE_TARGET_CAMERA_M = FINE_TARGET_CAMERA_M
    TARGET_ROTATION_CAMERA_TAG = FINE_TARGET_ROTATION_CAMERA_TAG
    FINAL_APPROACH_TARGET_CAMERA_M = FINAL_APPROACH_TARGET_CAMERA_M
    FINAL_GRIPPER_CLOSED = FINAL_GRIPPER_CLOSED
    GRIPPER_ACTION_WAIT_SEC = GRIPPER_ACTION_WAIT_SEC
    RETREAT_DIRECTION_GRIPPER = RETREAT_DIRECTION_GRIPPER
    RETREAT_DISTANCE_M = RETREAT_DISTANCE_M
    RETREAT_SPEED_M_S = RETREAT_SPEED_M_S

    def __init__(self) -> None:
        if FINAL_STAGE not in (
            STAGE_COARSE_APPROACH,
            STAGE_FINE_APPROACH,
            STAGE_FINAL_APPROACH,
            STAGE_RELEASE_AND_RETREAT,
        ):
            raise ValueError(f"invalid FINAL_STAGE: {FINAL_STAGE}")
        self.stage = STAGE_COARSE_APPROACH
        self.finished = False
        self._pose_stable_time_sec = 0.0
        self._reset_release_and_retreat()

    def reset(self) -> None:
        self.stage = STAGE_COARSE_APPROACH
        self.finished = False
        self._pose_stable_time_sec = 0.0
        self._reset_release_and_retreat()

    @staticmethod
    def _zero() -> PolicyCommand:
        return PolicyCommand(np.zeros(6, dtype=float), None)

    @staticmethod
    def _limit_norm(vector: np.ndarray, maximum: float) -> np.ndarray:
        norm = float(np.linalg.norm(vector))
        return vector if norm <= maximum else vector * (maximum / norm)

    def _reset_release_and_retreat(self) -> None:
        self._gripper_action_elapsed_sec = 0.0
        self._retreat_axis_base = None
        self._retreat_start_position_base = None

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
        if completed_stage >= FINAL_STAGE:
            self.finished = True
            print(f"stage {completed_stage} complete; holding at configured FINAL_STAGE")
            return
        self.stage = completed_stage + 1
        self._pose_stable_time_sec = 0.0
        if self.stage == STAGE_RELEASE_AND_RETREAT:
            self._reset_release_and_retreat()
        print(f"stage {completed_stage} complete; advancing to stage {self.stage}")

    def _step_coarse(self, observation: PolicyObservation) -> PolicyCommand:
        if not self._tag_ready(observation):
            return self._zero()

        tag_position_camera = np.asarray(observation.T_camera_tag, dtype=float)[:3, 3]
        error_camera = tag_position_camera - self.COARSE_TARGET_CAMERA_M
        if np.all(np.abs(error_camera) <= COARSE_TOLERANCE_M):
            self._finish_or_advance(STAGE_COARSE_APPROACH)
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
            * np.array([-vertical_error, horizontal_error, 0.0], dtype=float),
            COARSE_ANGULAR_MAX_RAD_S,
        )

        rotation_base_gripper = np.asarray(observation.T_base_tool, dtype=float)[:3, :3]
        rotation_base_camera = rotation_base_gripper @ R_GRIPPER_CAMERA
        task_velocity = np.concatenate(
            (rotation_base_camera @ linear_camera, rotation_base_camera @ angular_camera)
            # Translation only
            # (rotation_base_camera @ linear_camera, np.array([0.0, 0.0, 0.0], dtype=float))
        )
        return PolicyCommand(task_velocity_base=task_velocity, gripper_closed=None)

    def _step_pose_servo(
        self,
        observation: PolicyObservation,
        target_position_camera: np.ndarray,
        completed_stage: int,
        position_tolerance_m: float,
        orientation_tolerance_rad: float,
        stable_time_sec: float,
    ) -> PolicyCommand:
        if not self._tag_ready(observation):
            self._pose_stable_time_sec = 0.0
            return self._zero()

        transform_camera_tag = np.asarray(observation.T_camera_tag, dtype=float)
        tag_position_camera = transform_camera_tag[:3, 3]
        tag_rotation_camera = transform_camera_tag[:3, :3]
        position_error_camera = tag_position_camera - target_position_camera
        rotation_error_camera = (
            tag_rotation_camera @ self.TARGET_ROTATION_CAMERA_TAG.T
        )
        orientation_error_camera = self._rotation_vector(rotation_error_camera)

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
            FINE_ANGULAR_KP * orientation_error_camera,
            FINE_ANGULAR_MAX_RAD_S,
        )
        # This cross term translates around the tag while rotating, reducing
        # the tendency for orientation correction to disturb tag position.
        linear_camera = self._limit_norm(
            FINE_LINEAR_KP * position_error_camera
            - np.cross(angular_camera, tag_position_camera),
            FINE_LINEAR_MAX_M_S,
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

    def _step_fine(self, observation: PolicyObservation) -> PolicyCommand:
        return self._step_pose_servo(
            observation,
            self.FINE_TARGET_CAMERA_M,
            STAGE_FINE_APPROACH,
            FINE_POSITION_TOLERANCE_M,
            FINE_ORIENTATION_TOLERANCE_RAD,
            FINE_STABLE_TIME_SEC,
        )

    def _step_final_approach(self, observation: PolicyObservation) -> PolicyCommand:
        return self._step_pose_servo(
            observation,
            self.FINAL_APPROACH_TARGET_CAMERA_M,
            STAGE_FINAL_APPROACH,
            FINAL_APPROACH_POSITION_TOLERANCE_M,
            FINAL_APPROACH_ORIENTATION_TOLERANCE_RAD,
            FINAL_APPROACH_STABLE_TIME_SEC,
        )

    def _step_release_and_retreat(
        self,
        observation: PolicyObservation,
    ) -> PolicyCommand:
        transform_base_gripper = np.asarray(observation.T_base_tool, dtype=float)
        rotation_base_gripper = transform_base_gripper[:3, :3]

        if self._retreat_axis_base is None:
            retreat_direction_gripper = np.asarray(
                self.RETREAT_DIRECTION_GRIPPER,
                dtype=float,
            ).reshape(3)
            retreat_axis = rotation_base_gripper @ retreat_direction_gripper
            retreat_axis_norm = float(np.linalg.norm(retreat_axis))
            if retreat_axis_norm <= 1e-9:
                raise ValueError("end-effector retreat direction has zero length")
            self._retreat_axis_base = retreat_axis / retreat_axis_norm

        # Place opens, pick closes, and hook leaves the existing gripper command
        # unchanged. All variants hold still for the same action delay.
        if self._gripper_action_elapsed_sec < self.GRIPPER_ACTION_WAIT_SEC:
            self._gripper_action_elapsed_sec += max(0.0, float(observation.dt_sec))
            return PolicyCommand(
                np.zeros(6, dtype=float),
                self.FINAL_GRIPPER_CLOSED,
            )

        current_position_base = transform_base_gripper[:3, 3]
        if self._retreat_start_position_base is None:
            self._retreat_start_position_base = current_position_base.copy()
            print(
                "gripper action wait complete; starting "
                f"{self.RETREAT_DISTANCE_M * 1000.0:.0f} mm retreat along "
                "the configured end-effector direction"
            )
            return PolicyCommand(
                np.zeros(6, dtype=float),
                self.FINAL_GRIPPER_CLOSED,
            )

        displacement_base = current_position_base - self._retreat_start_position_base
        retreat_progress_m = float(
            np.dot(displacement_base, self._retreat_axis_base)
        )
        if retreat_progress_m >= self.RETREAT_DISTANCE_M:
            self._finish_or_advance(STAGE_RELEASE_AND_RETREAT)
            return PolicyCommand(
                np.zeros(6, dtype=float),
                self.FINAL_GRIPPER_CLOSED,
            )

        task_velocity = np.zeros(6, dtype=float)
        task_velocity[:3] = self.RETREAT_SPEED_M_S * self._retreat_axis_base
        return PolicyCommand(task_velocity, self.FINAL_GRIPPER_CLOSED)

    def step(self, observation: PolicyObservation) -> PolicyCommand:
        if self.finished:
            return self._zero()
        if self.stage == STAGE_COARSE_APPROACH:
            return self._step_coarse(observation)
        if self.stage == STAGE_FINE_APPROACH:
            return self._step_fine(observation)
        if self.stage == STAGE_FINAL_APPROACH:
            return self._step_final_approach(observation)
        if self.stage == STAGE_RELEASE_AND_RETREAT:
            return self._step_release_and_retreat(observation)
        return self._zero()
