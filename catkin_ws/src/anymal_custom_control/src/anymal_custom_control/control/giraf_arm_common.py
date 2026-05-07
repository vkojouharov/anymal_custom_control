"""Shared constants and helpers for the GIRAF arm ROS stack."""

from __future__ import annotations

import numpy as np


CONTROL_LOOP_HZ = 200.0
TELEOP_PUBLISH_HZ = 100.0
STATE_PUBLISH_HZ = 10.0
COMMAND_TIMEOUT_SEC = 0.15

TELEOP_TASK_VELOCITY_TOPIC = "/giraf_arm/teleop_task_velocity_cmd"
AUTO_TASK_VELOCITY_TOPIC = "/giraf_arm/auto_task_velocity_cmd"
TELEOP_GRIPPER_VELOCITY_TOPIC = "/giraf_arm/teleop_gripper_velocity_cmd"
AUTO_GRIPPER_VELOCITY_TOPIC = "/giraf_arm/auto_gripper_velocity_cmd"
STOP_TOPIC = "/giraf_arm/stop"
STATE_TOPIC = "/giraf_arm/state"
DEBUG_TOPIC = "/giraf_arm/debug"
COMMAND_SOURCE_TOPIC = "/giraf_arm/command_source"

ARM_X_SPEED = 0.2
ARM_Y_SPEED = 0.2
ARM_Z_SPEED = 0.1
ARM_WX_SPEED = 0.5
ARM_WY_SPEED = 0.5
ARM_WZ_SPEED = 0.5
TASK_WX_LIMIT = 1.0
TASK_WY_LIMIT = 1.0
TASK_WZ_LIMIT = 1.0
TASK_VELOCITY_LIMITS = np.array(
    [
        ARM_X_SPEED,
        ARM_Y_SPEED,
        ARM_Z_SPEED,
        TASK_WX_LIMIT,
        TASK_WY_LIMIT,
        TASK_WZ_LIMIT,
    ],
    dtype=float,
)

GRIPPER_SPEED = 2.0

JPARSE_GAMMA = 0.10
JPARSE_POS_GAIN = 1.0
JPARSE_ANG_GAIN = 1.0
QDOT_LIMITS = np.array([3.0, 2.0, 0.5, 3.0, 3.0, 3.0], dtype=float)
TRANSLATION_LOCK_DAMPING = 0.05
PURE_ROTATION_LINEAR_EPS = 1e-6
PURE_ROTATION_ANGULAR_EPS = 1e-4

ROLL_LIMIT = np.pi / 2
PITCH_MIN = 0.0
PITCH_MAX = np.pi / 2
D3_MIN = 0.31
BOOM_MIN = -30.0
BOOM_MAX = 0.0

# Boom arm offsets
PITCH_KIN_OFFSET = np.pi / 2
# wrist motor offsets
THETA4_KIN_OFFSET = np.pi / 2
THETA5_KIN_OFFSET = -np.pi / 2  # change this to PRR metal wrist model
THETA6_KIN_OFFSET = 0

THETA4_DXL_SIGN = 1.0
THETA5_DXL_SIGN = -1.0   # change this to -1 for PRR metal wrist model

DEFAULT_COMMAND_SOURCE = "teleop"


def clamp_task_velocity(velocity: np.ndarray) -> np.ndarray:
    """Clip task-space velocity to configured safety limits."""
    return np.clip(np.asarray(velocity, dtype=float), -TASK_VELOCITY_LIMITS, TASK_VELOCITY_LIMITS)


def zero_task_velocity() -> np.ndarray:
    """Return a zero 6-DoF task velocity vector."""
    return np.zeros(6, dtype=float)
