"""State-machine tests for camera-independent open-loop arm policies."""

import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np

from cable_outfitting.executor import CableExecutor
from cable_outfitting.trajectory import (
    DEFAULT_DEPLOYMENT_TIMEOUT_S,
    TaskPoint,
    Trajectory,
    load_trajectory,
)


class CableHomeStateMachineTest(unittest.TestCase):
    @staticmethod
    def executor(policy="home"):
        point = TaskPoint(
            name="navigation" if policy is None else policy,
            navigation_tag_id=123,
            navigation_goal=np.zeros(3),
            deployment_twist=np.zeros(6),
            deployment_timeout_s=1.0,
            manipulation_tag_id=456,
            policy=policy,
        )
        executor = CableExecutor.__new__(CableExecutor)
        executor.trajectory = Trajectory(point.name, (point,))
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

    def test_final_stage_runs_without_reading_manipulation_tag(self):
        class Command:
            task_velocity_base = np.array([0.1, 0.0, 0.0, 0.0, 0.0, 0.0])
            gripper_closed = True

        for policy_name in ("pick", "place", "hook"):
            with self.subTest(policy=policy_name):
                executor = self.executor(policy_name)
                executor._base = Mock()
                executor._tag = Mock(side_effect=AssertionError("stage 5 must not read a tag"))
                executor.arm_camera = SimpleNamespace(failure=None)
                executor.arm = Mock()
                executor.arm.snapshot.return_value = SimpleNamespace(
                    joints=np.zeros(6),
                    T_base_tool=np.eye(4),
                    gripper_closed=False,
                )
                executor.policy_observation = lambda **values: SimpleNamespace(**values)
                executor.policy_command = Command
                executor.policy = SimpleNamespace(
                    stage=5,
                    finished=False,
                    step=Mock(return_value=Command()),
                )
                executor._last_step = 39.0

                executor._step_policy(40.0)

                executor._tag.assert_not_called()
                commanded_task, commanded_gripper = executor.arm.command.call_args.args
                np.testing.assert_array_equal(commanded_task, Command.task_velocity_base)
                self.assertTrue(commanded_gripper)

    def test_final_stage_resumes_without_reading_manipulation_tag(self):
        executor = self.executor("pick")
        executor.policy = SimpleNamespace(stage=5)
        executor.resume_phase = "MANIPULATING"
        executor._tag = Mock(side_effect=AssertionError("stage 5 must not read a tag"))

        executor._resume(50.0)

        executor._tag.assert_not_called()
        self.assertEqual(executor.phase, "MANIPULATING")
        self.assertEqual(executor._last_step, 50.0)

    def test_navigation_only_point_completes_without_deployment(self):
        executor = self.executor(None)
        executor._complete_point = Mock()
        executor._begin_deployment = Mock()

        executor._finish_navigation(60.0)

        executor._complete_point.assert_called_once_with(60.0)
        executor._begin_deployment.assert_not_called()

    def test_omitted_deployment_defaults_to_zero_motion(self):
        data = {
            "name": "default_deployment",
            "task_points": [
                {
                    "name": "pick",
                    "navigation": {"tag_id": 11, "goal": [0.75, 0.0, 0.0]},
                    "manipulation": {"tag_id": 7, "policy": "pick"},
                }
            ],
        }
        with patch("cable_outfitting.trajectory._read_yaml", return_value=data):
            point = load_trajectory("unused.yaml").task_points[0]

        np.testing.assert_array_equal(point.deployment_twist, np.zeros(6))
        self.assertEqual(point.deployment_timeout_s, DEFAULT_DEPLOYMENT_TIMEOUT_S)

    def test_omitted_navigation_inherits_previous_point(self):
        data = {
            "name": "inherited_navigation",
            "task_points": [
                {
                    "name": "pick",
                    "navigation": {"tag_id": 11, "goal": [0.75, 0.0, 0.0]},
                    "manipulation": {"tag_id": 7, "policy": "pick"},
                },
                {
                    "name": "hook",
                    "manipulation": {"tag_id": 4, "policy": "hook"},
                },
            ],
        }
        with patch("cable_outfitting.trajectory._read_yaml", return_value=data):
            points = load_trajectory("unused.yaml").task_points

        self.assertEqual(points[1].navigation_tag_id, points[0].navigation_tag_id)
        np.testing.assert_array_equal(points[1].navigation_goal, points[0].navigation_goal)
        self.assertIsNot(points[1].navigation_goal, points[0].navigation_goal)

    def test_first_point_still_requires_navigation(self):
        data = {
            "name": "invalid",
            "task_points": [
                {
                    "name": "pick",
                    "manipulation": {"tag_id": 7, "policy": "pick"},
                }
            ],
        }
        with patch("cable_outfitting.trajectory._read_yaml", return_value=data):
            with self.assertRaisesRegex(ValueError, "navigation is required for the first task point"):
                load_trajectory("unused.yaml")

    def test_omitted_manipulation_creates_navigation_only_point(self):
        data = {
            "name": "navigation_only",
            "task_points": [
                {
                    "name": "reposition",
                    "navigation": {"tag_id": 11, "goal": [0.75, 0.0, 0.0]},
                }
            ],
        }
        with patch("cable_outfitting.trajectory._read_yaml", return_value=data):
            point = load_trajectory("unused.yaml").task_points[0]

        self.assertIsNone(point.deployment_twist)
        self.assertIsNone(point.deployment_timeout_s)
        self.assertIsNone(point.manipulation_tag_id)
        self.assertIsNone(point.policy)

    def test_deployment_without_manipulation_is_rejected(self):
        data = {
            "name": "invalid",
            "task_points": [
                {
                    "name": "deployment_without_policy",
                    "navigation": {"tag_id": 11, "goal": [0.75, 0.0, 0.0]},
                    "deployment": {
                        "twist": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                        "timeout_s": 5.0,
                    },
                }
            ],
        }
        with patch("cable_outfitting.trajectory._read_yaml", return_value=data):
            with self.assertRaisesRegex(ValueError, "deployment requires manipulation"):
                load_trajectory("unused.yaml")


if __name__ == "__main__":
    unittest.main()
