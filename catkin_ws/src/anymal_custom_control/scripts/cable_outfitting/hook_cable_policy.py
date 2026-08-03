#!/usr/bin/env python3
"""Hook-cable policy: approach and retreat without changing the gripper."""

import numpy as np

from place_cable_policy import (
    CablePolicy as PlaceCablePolicy,
    PolicyCommand,
    PolicyObservation,
)


# -------------------- HOOK POLICY TARGET POSES (EDIT THESE) --------------------
# All positions are tag positions expressed in the camera frame [x, y, z], m.
COARSE_TARGET_CAMERA_M = np.array([0.0, -0.01, 0.30], dtype=float)
FINE_TARGET_CAMERA_M = np.array([0.0, -0.01, 0.20], dtype=float)
FINAL_APPROACH_TARGET_CAMERA_M = np.array([0.0, -0.01, 0.155], dtype=float)

# Desired tag orientation relative to the camera for stages 2 and 3.
TARGET_ROTATION_CAMERA_TAG = np.eye(3, dtype=float)

# -------------------- HOOK POLICY STAGE 4 (EDIT THESE) --------------------
# Direction is expressed in the moving end-effector frame [x, y, z].
FINAL_GRIPPER_CLOSED = None
GRIPPER_ACTION_WAIT_SEC = 1.0
RETREAT_DIRECTION_GRIPPER = np.array([0.0, 1.0, 0.0], dtype=float)
RETREAT_DISTANCE_M = 0.100
RETREAT_SPEED_M_S = 0.02


class CablePolicy(PlaceCablePolicy):
    """Leave the gripper state unchanged during the common retreat."""

    COARSE_TARGET_CAMERA_M = COARSE_TARGET_CAMERA_M
    FINE_TARGET_CAMERA_M = FINE_TARGET_CAMERA_M
    TARGET_ROTATION_CAMERA_TAG = TARGET_ROTATION_CAMERA_TAG
    FINAL_APPROACH_TARGET_CAMERA_M = FINAL_APPROACH_TARGET_CAMERA_M
    FINAL_GRIPPER_CLOSED = FINAL_GRIPPER_CLOSED
    GRIPPER_ACTION_WAIT_SEC = GRIPPER_ACTION_WAIT_SEC
    RETREAT_DIRECTION_GRIPPER = RETREAT_DIRECTION_GRIPPER
    RETREAT_DISTANCE_M = RETREAT_DISTANCE_M
    RETREAT_SPEED_M_S = RETREAT_SPEED_M_S


__all__ = ["CablePolicy", "PolicyCommand", "PolicyObservation"]
