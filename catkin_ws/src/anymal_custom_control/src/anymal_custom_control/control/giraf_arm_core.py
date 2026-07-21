"""ROS- and hardware-independent GIRAF task-space control core.

Both the hardware controller and the dry-run backend use this class.  Keeping
the kinematics, J-PARSE inversion, integration, and joint clamps here prevents
the dry run from becoming a second, subtly different controller.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from anymal_custom_control.RRP_kinematic_model import get_boom_length_d3, get_boom_motor_rad
from anymal_custom_control.RRPRRR_kinematic_model import num_forward_kinematics, num_jacobian
from anymal_custom_control.jparse_controller import JParseController, compute_metrics

from .giraf_arm_common import (
    BOOM_MAX,
    BOOM_MIN,
    D3_MIN,
    GRIPPER_SPEED,
    JPARSE_ANG_GAIN,
    JPARSE_GAMMA,
    JPARSE_POS_GAIN,
    PITCH_KIN_OFFSET,
    PITCH_MAX,
    PITCH_MIN,
    PURE_ROTATION_ANGULAR_EPS,
    PURE_ROTATION_LINEAR_EPS,
    QDOT_LIMITS,
    ROLL_LIMIT,
    THETA4_KIN_OFFSET,
    THETA5_KIN_OFFSET,
    THETA6_KIN_OFFSET,
    TRANSLATION_LOCK_DAMPING,
    TASK_VELOCITY_LIMITS,
)


@dataclass
class ArmCoordinates:
    """Controller coordinates; ``d3`` is extension in metres."""

    roll: float = 0.0
    pitch: float = 0.0
    d3: float = D3_MIN
    th4: float = 0.0
    th5: float = 0.0
    th6: float = 0.0
    grip: float = 1.0

    @property
    def boom(self) -> float:
        return float(get_boom_motor_rad(self.d3))

    def kinematic_coordinates(self) -> list[float]:
        return [
            self.roll,
            self.pitch + PITCH_KIN_OFFSET,
            self.d3,
            self.th4 + THETA4_KIN_OFFSET,
            self.th5 + THETA5_KIN_OFFSET,
            self.th6 + THETA6_KIN_OFFSET,
        ]


@dataclass
class ControlStep:
    coordinates: ArmCoordinates
    task_velocity: np.ndarray
    joint_velocity: np.ndarray
    control_mode: str
    transform: np.ndarray
    metrics: dict[str, object]


class GirafArmControlCore:
    """J-PARSE control and bounded command-state integration."""

    def __init__(
        self,
        theta_limits: tuple[tuple[float, float], tuple[float, float], tuple[float, float]],
        initial_coordinates: ArmCoordinates | None = None,
        task_velocity_limits: np.ndarray | None = None,
    ) -> None:
        self.theta_limits = theta_limits
        self.task_velocity_limits = np.asarray(
            TASK_VELOCITY_LIMITS if task_velocity_limits is None else task_velocity_limits,
            dtype=float,
        )
        if self.task_velocity_limits.shape != (6,) or not np.all(np.isfinite(self.task_velocity_limits)):
            raise ValueError("task velocity limits must contain six finite values")
        if np.any(self.task_velocity_limits <= 0.0):
            raise ValueError("task velocity limits must be positive")
        self.coordinates = initial_coordinates or ArmCoordinates()
        self._jparse = JParseController(gamma=JPARSE_GAMMA)
        self._clamp_coordinates()

    @staticmethod
    def _is_pure_rotation_command(velocity: np.ndarray) -> bool:
        linear_norm = float(np.linalg.norm(velocity[:3]))
        angular_norm = float(np.linalg.norm(velocity[3:]))
        return linear_norm <= PURE_ROTATION_LINEAR_EPS and angular_norm > PURE_ROTATION_ANGULAR_EPS

    def _compute_joint_velocity(self, jacobian: np.ndarray, velocity: np.ndarray) -> tuple[np.ndarray, str]:
        if self._is_pure_rotation_command(velocity):
            linear_jacobian = jacobian[:3, :]
            angular_jacobian = jacobian[3:, :]
            linear_inverse = JParseController.damped_least_squares(
                linear_jacobian,
                damping=TRANSLATION_LOCK_DAMPING,
            )
            translation_nullspace = np.eye(jacobian.shape[1], dtype=float) - linear_inverse.dot(linear_jacobian)
            locked_angular_jacobian = angular_jacobian.dot(translation_nullspace)
            locked_inverse = self._jparse.compute(
                locked_angular_jacobian,
                singular_direction_gain_angular=JPARSE_ANG_GAIN,
                angular_dimensions=3,
            )
            joint_velocity = translation_nullspace.dot(locked_inverse.dot(velocity[3:]))
            return np.asarray(joint_velocity, dtype=float).reshape(-1), "rot-lock"

        inverse = self._jparse.compute(
            jacobian,
            singular_direction_gain_position=JPARSE_POS_GAIN,
            singular_direction_gain_angular=JPARSE_ANG_GAIN,
            position_dimensions=3,
            angular_dimensions=3,
        )
        joint_velocity = inverse.dot(velocity)
        return np.asarray(joint_velocity, dtype=float).reshape(-1), "full"

    def _clamp_coordinates(self) -> None:
        q = self.coordinates
        q.roll = float(np.clip(q.roll, -ROLL_LIMIT, ROLL_LIMIT))
        q.pitch = float(np.clip(q.pitch, PITCH_MIN, PITCH_MAX))
        q.d3 = max(float(q.d3), D3_MIN)
        q.th4 = float(np.clip(q.th4, *self.theta_limits[0]))
        q.th5 = float(np.clip(q.th5, *self.theta_limits[1]))
        q.th6 = float(np.clip(q.th6, *self.theta_limits[2]))
        q.grip = float(np.clip(q.grip, 0.0, 1.0))

        boom = float(np.clip(get_boom_motor_rad(q.d3), BOOM_MIN, BOOM_MAX))
        q.d3 = float(get_boom_length_d3(boom))

    def step(self, task_velocity: np.ndarray, gripper_velocity: float, dt_sec: float) -> ControlStep:
        task_velocity = np.asarray(task_velocity, dtype=float).reshape(-1)
        if task_velocity.shape != (6,) or not np.all(np.isfinite(task_velocity)):
            raise ValueError("task velocity must contain six finite values")
        if not math.isfinite(float(gripper_velocity)):
            raise ValueError("gripper velocity must be finite")
        if not math.isfinite(float(dt_sec)) or dt_sec <= 0.0:
            raise ValueError("control timestep must be finite and positive")

        task_velocity = np.clip(task_velocity, -self.task_velocity_limits, self.task_velocity_limits)
        gripper_velocity = float(np.clip(gripper_velocity, -GRIPPER_SPEED, GRIPPER_SPEED))
        jacobian = np.asarray(num_jacobian(self.coordinates.kinematic_coordinates()), dtype=float)
        joint_velocity, control_mode = self._compute_joint_velocity(jacobian, task_velocity)
        joint_velocity = np.clip(joint_velocity, -QDOT_LIMITS, QDOT_LIMITS)
        if not np.all(np.isfinite(joint_velocity)):
            raise ValueError("J-PARSE produced a non-finite joint velocity")

        q = self.coordinates
        q.roll += float(joint_velocity[0]) * dt_sec
        q.pitch += float(joint_velocity[1]) * dt_sec
        q.d3 += float(joint_velocity[2]) * dt_sec
        q.th4 += float(joint_velocity[3]) * dt_sec
        q.th5 += float(joint_velocity[4]) * dt_sec
        q.th6 += float(joint_velocity[5]) * dt_sec
        q.grip += gripper_velocity * dt_sec
        self._clamp_coordinates()

        final_coordinates = q.kinematic_coordinates()
        final_jacobian = np.asarray(num_jacobian(final_coordinates), dtype=float)
        return ControlStep(
            coordinates=q,
            task_velocity=task_velocity,
            joint_velocity=joint_velocity,
            control_mode=control_mode,
            transform=np.asarray(num_forward_kinematics(final_coordinates), dtype=float),
            metrics=compute_metrics(final_jacobian),
        )
