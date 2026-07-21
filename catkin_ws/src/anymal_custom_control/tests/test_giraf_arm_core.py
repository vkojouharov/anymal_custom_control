"""Hardware-free tests for the shared GIRAF control core."""

import unittest

import numpy as np

from anymal_custom_control.control.giraf_arm_common import QDOT_LIMITS, TASK_VELOCITY_LIMITS
from anymal_custom_control.control.giraf_arm_core import GirafArmControlCore


WRIST_LIMITS = ((-1.534, 2.546), (-1.534, 1.534), (-3.068, 3.068))


class GirafArmControlCoreTest(unittest.TestCase):
    def setUp(self):
        self.core = GirafArmControlCore(theta_limits=WRIST_LIMITS)

    def test_nominal_home_and_zero_command_hold(self):
        before = self.core.coordinates
        result = self.core.step(np.zeros(6), 0.0, 0.005)
        self.assertAlmostEqual(result.coordinates.roll, before.roll)
        self.assertAlmostEqual(result.coordinates.pitch, before.pitch)
        self.assertAlmostEqual(result.coordinates.d3, before.d3)
        self.assertAlmostEqual(result.coordinates.boom, 0.0, places=3)
        np.testing.assert_allclose(result.joint_velocity, np.zeros(6), atol=1e-12)

    def test_task_and_joint_velocity_limits_are_applied(self):
        result = self.core.step(np.full(6, 1e6), 0.0, 0.005)
        self.assertTrue(np.all(np.abs(result.task_velocity) <= TASK_VELOCITY_LIMITS))
        self.assertTrue(np.all(np.abs(result.joint_velocity) <= QDOT_LIMITS))

    def test_operator_limits_can_be_stricter_than_legacy_limits(self):
        limits = np.array([0.05, 0.05, 0.025, 0.125, 0.125, 0.125])
        core = GirafArmControlCore(theta_limits=WRIST_LIMITS, task_velocity_limits=limits)
        result = core.step(np.ones(6), 0.0, 0.005)
        np.testing.assert_allclose(result.task_velocity, limits)

    def test_nonfinite_command_is_rejected(self):
        command = np.zeros(6)
        command[2] = np.nan
        with self.assertRaises(ValueError):
            self.core.step(command, 0.0, 0.005)

    def test_position_limits_remain_enforced(self):
        for _ in range(100):
            self.core.step(np.full(6, 100.0), 100.0, 0.05)
        q = self.core.coordinates
        self.assertGreaterEqual(q.pitch, 0.0)
        self.assertLessEqual(q.pitch, np.pi / 2)
        self.assertGreaterEqual(q.d3, 0.31)
        self.assertGreaterEqual(q.grip, 0.0)
        self.assertLessEqual(q.grip, 1.0)
        self.assertGreaterEqual(q.th4, WRIST_LIMITS[0][0])
        self.assertLessEqual(q.th4, WRIST_LIMITS[0][1])
        self.assertGreaterEqual(q.th5, WRIST_LIMITS[1][0])
        self.assertLessEqual(q.th5, WRIST_LIMITS[1][1])
        self.assertGreaterEqual(q.th6, WRIST_LIMITS[2][0])
        self.assertLessEqual(q.th6, WRIST_LIMITS[2][1])


if __name__ == "__main__":
    unittest.main()
