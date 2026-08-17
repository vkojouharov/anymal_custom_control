"""State-machine tests for camera-independent cable-arm homing."""

import unittest
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np

from cable_outfitting.executor import CableExecutor
from cable_outfitting.trajectory import TaskPoint, Trajectory


class CableHomeStateMachineTest(unittest.TestCase):
    @staticmethod
    def executor():
        point = TaskPoint(
            name="home",
            navigation_tag_id=123,
            navigation_goal=np.zeros(3),
            deployment_twist=np.zeros(6),
            deployment_timeout_s=1.0,
            manipulation_tag_id=456,
            policy="home",
        )
        executor = CableExecutor.__new__(CableExecutor)
        executor.trajectory = Trajectory("home", (point,))
        executor.point_index = 0
        return executor

    def test_home_bypasses_navigation_and_deployment(self):
        executor = self.executor()
        executor._begin_policy = Mock()
        executor._begin_deployment = Mock()

        executor._begin_navigation(10.0)

        executor._begin_policy.assert_called_once_with(10.0)
        executor._begin_deployment.assert_not_called()

    def test_home_starts_without_reading_navigation_tag(self):
        executor = self.executor()
        executor.navigation_camera = object()
        executor._tag = Mock(side_effect=AssertionError("home must not read a tag"))
        executor._begin_navigation = Mock()

        executor._start_waiting(20.0)

        executor._tag.assert_not_called()
        executor._begin_navigation.assert_called_once_with(20.0)

    def test_running_home_does_not_read_manipulation_tag(self):
        class Command:
            task_velocity_base = np.zeros(6)
            gripper_closed = False
            mab_position = np.zeros(3)

        executor = self.executor()
        executor._base = Mock()
        executor._tag = Mock(side_effect=AssertionError("home must not read a tag"))
        executor.arm_camera = SimpleNamespace(failure=None)
        executor.arm = Mock()
        executor.arm.snapshot.return_value = SimpleNamespace(
            joints=np.zeros(6),
            T_base_tool=np.eye(4),
            gripper_closed=False,
        )
        executor.policy_observation = lambda **values: SimpleNamespace(**values)
        executor.policy_command = Command
        executor.policy = Mock(finished=False)
        executor.policy.step.return_value = Command()
        executor._last_step = 29.0

        executor._step_policy(30.0)

        executor._tag.assert_not_called()
        executor.arm.command_home.assert_called_once()


if __name__ == "__main__":
    unittest.main()
