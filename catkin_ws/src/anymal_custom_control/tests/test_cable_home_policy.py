"""Hardware-free tests for the cable-outfitting home policy."""

import unittest

import numpy as np

from anymal_custom_control.RRP_kinematic_model import get_boom_length_d3
from cable_outfitting.policies.home import (
    BASE_HOME_SPEED_RAD_S,
    BOOM_HOME_SPEED_RAD_S,
    CablePolicy,
    PolicyObservation,
    STAGE_BOOM,
    STAGE_PITCH,
    STAGE_ROLL,
)


class CableHomePolicyTest(unittest.TestCase):
    @staticmethod
    def observation(mab_position, wrist_position, gripper_closed, dt_sec=0.1):
        joints = np.concatenate(
            (
                np.asarray(mab_position[:2], dtype=float),
                [get_boom_length_d3(float(mab_position[2]))],
                np.asarray(wrist_position, dtype=float),
            )
        )
        return PolicyObservation(
            T_camera_tag=None,
            tag_age_sec=float("inf"),
            tag_visible=False,
            tag_decision_margin=None,
            camera_available=False,
            joint_position=joints,
            T_base_tool=np.eye(4),
            gripper_closed=gripper_closed,
            dt_sec=dt_sec,
        )

    def test_homes_boom_then_roll_then_pitch_without_camera(self):
        policy = CablePolicy()
        mab = np.array([0.4, 0.3, -0.15], dtype=float)
        wrist = np.array([0.2, -0.3, 0.4], dtype=float)

        command = policy.step(self.observation(mab, wrist, True))

        self.assertEqual(policy.stage, STAGE_BOOM)
        self.assertFalse(policy.finished)
        np.testing.assert_allclose(command.mab_position[:2], mab[:2])
        self.assertAlmostEqual(
            command.mab_position[2],
            mab[2] + BOOM_HOME_SPEED_RAD_S * 0.1,
        )
        np.testing.assert_array_equal(command.task_velocity_base, np.zeros(6))
        self.assertFalse(command.gripper_closed)

        mab = command.mab_position
        wrist = np.zeros(3)
        command = policy.step(self.observation(mab, wrist, False))
        self.assertEqual(policy.stage, STAGE_BOOM)
        np.testing.assert_allclose(command.mab_position[:2], mab[:2])
        self.assertAlmostEqual(command.mab_position[2], 0.0)

        mab = command.mab_position
        command = policy.step(self.observation(mab, wrist, False))
        self.assertEqual(policy.stage, STAGE_ROLL)
        self.assertAlmostEqual(command.mab_position[2], 0.0)
        self.assertAlmostEqual(
            command.mab_position[0],
            mab[0] - BASE_HOME_SPEED_RAD_S * 0.1,
        )
        self.assertAlmostEqual(command.mab_position[1], mab[1])

        mab = np.array([0.0, command.mab_position[1], 0.0])
        command = policy.step(self.observation(mab, wrist, False))
        self.assertEqual(policy.stage, STAGE_PITCH)
        self.assertAlmostEqual(command.mab_position[0], 0.0)
        self.assertAlmostEqual(
            command.mab_position[1],
            mab[1] - BASE_HOME_SPEED_RAD_S * 0.1,
        )

    def test_finishes_only_after_home_state_is_observed(self):
        policy = CablePolicy()
        mab = np.array([0.04, 0.03, 0.0], dtype=float)
        wrist = np.zeros(3)

        command = policy.step(self.observation(mab, wrist, False))
        np.testing.assert_allclose(command.mab_position, [0.0, 0.03, 0.0])
        self.assertFalse(policy.finished)

        mab = command.mab_position
        command = policy.step(
            self.observation(mab, wrist, False)
        )
        np.testing.assert_array_equal(command.mab_position, np.zeros(3))
        self.assertFalse(policy.finished)

        command = policy.step(self.observation(command.mab_position, wrist, False))
        self.assertTrue(policy.finished)
        np.testing.assert_array_equal(command.mab_position, np.zeros(3))


if __name__ == "__main__":
    unittest.main()
