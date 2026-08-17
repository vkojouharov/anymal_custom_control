#!/usr/bin/env python3
"""Joint-space policy that returns the cable arm to its startup pose."""

from dataclasses import dataclass
from typing import Optional

import numpy as np

from anymal_custom_control.RRP_kinematic_model import get_boom_motor_rad


BOOM_HOME_SPEED_RAD_S = 1.0
BASE_HOME_SPEED_RAD_S = 0.5
HOME_TOLERANCE_RAD = 1e-6

STAGE_BOOM = 1
STAGE_BASE = 2


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
    mab_position: np.ndarray


class CablePolicy:
    """Home the wrist immediately and the three MAB joints sequentially."""

    def __init__(self) -> None:
        self.stage = STAGE_BOOM
        self.finished = False

    def reset(self) -> None:
        self.stage = STAGE_BOOM
        self.finished = False

    @staticmethod
    def _move_toward_zero(value: float, maximum_step: float) -> float:
        if abs(value) <= maximum_step:
            return 0.0
        return value - np.copysign(maximum_step, value)

    @staticmethod
    def _command(mab_position: np.ndarray) -> PolicyCommand:
        return PolicyCommand(
            task_velocity_base=np.zeros(6, dtype=float),
            gripper_closed=False,
            mab_position=np.asarray(mab_position, dtype=float),
        )

    def get_pose_errors(self):
        return None, None

    def step(self, observation: PolicyObservation) -> PolicyCommand:
        joints = np.asarray(observation.joint_position, dtype=float).reshape(6)
        mab_position = np.array(
            [joints[0], joints[1], get_boom_motor_rad(joints[2])],
            dtype=float,
        )
        dt_sec = max(0.0, float(observation.dt_sec))

        if self.stage == STAGE_BOOM:
            mab_position[2] = self._move_toward_zero(
                mab_position[2],
                BOOM_HOME_SPEED_RAD_S * dt_sec,
            )
            if abs(float(get_boom_motor_rad(joints[2]))) <= HOME_TOLERANCE_RAD:
                self.stage = STAGE_BASE

        if self.stage == STAGE_BASE:
            mab_position[2] = 0.0
            mab_position[0] = self._move_toward_zero(
                mab_position[0],
                BASE_HOME_SPEED_RAD_S * dt_sec,
            )
            mab_position[1] = self._move_toward_zero(
                mab_position[1],
                BASE_HOME_SPEED_RAD_S * dt_sec,
            )
            mab_home = np.all(np.abs(joints[:2]) <= HOME_TOLERANCE_RAD)
            wrist_home = np.all(np.abs(joints[3:]) <= HOME_TOLERANCE_RAD)
            if mab_home and wrist_home and not observation.gripper_closed:
                mab_position[:2] = 0.0
                self.finished = True

        return self._command(mab_position)
