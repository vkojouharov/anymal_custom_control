#!/usr/bin/env python3
"""Autonomous cable-operation policy interface."""

from dataclasses import dataclass
from typing import Optional

import numpy as np


@dataclass(frozen=True)
class PolicyObservation:
    T_camera_tag: Optional[np.ndarray]
    tag_age_sec: float
    tag_visible: bool
    tag_decision_margin: Optional[float]
    camera_available: bool
    joint_position: np.ndarray
    T_base_tool: np.ndarray
    dt_sec: float


@dataclass(frozen=True)
class PolicyCommand:
    task_velocity_base: np.ndarray
    gripper_velocity: float


class CablePolicy:
    """Replace ``step`` with the current cable-manipulation state machine."""

    def reset(self) -> None:
        pass

    def step(self, observation: PolicyObservation) -> PolicyCommand:
        del observation
        return PolicyCommand(
            task_velocity_base=np.zeros(6, dtype=float),
            gripper_velocity=0.0,
        )
