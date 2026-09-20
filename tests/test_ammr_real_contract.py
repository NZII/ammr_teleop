import unittest

import numpy as np

from ammr_teleop.common.data_contract import (
    REAL_JOINTS,
    arm_feedback,
    arms_from_sim_contract,
    clamp_command,
    combine_feedback,
    driver_command,
    from_sim_contract,
    to_sim_contract,
)


class AmmrRealContractTest(unittest.TestCase):
    def test_contract_is_fixed_14_dimensional_order(self):
        self.assertEqual(len(REAL_JOINTS), 14)
        self.assertEqual(REAL_JOINTS[0], "left_joint1")
        self.assertEqual(REAL_JOINTS[6], "left_gripper")
        self.assertEqual(REAL_JOINTS[7], "right_joint1")
        self.assertEqual(REAL_JOINTS[13], "right_gripper")

    def test_feedback_is_reordered_and_requires_complete_arm(self):
        names = ["gripper", "joint6", "joint5", "joint4", "joint3", "joint2", "joint1"]
        positions = [0.08, 6, 5, 4, 3, 2, 1]
        np.testing.assert_array_equal(
            arm_feedback(names, positions), [1, 2, 3, 4, 5, 6, 0.08]
        )
        with self.assertRaisesRegex(ValueError, "missing driver joints"):
            arm_feedback(["joint1"], [0.0])

    def test_driver_command_always_contains_all_six_joints_and_gripper(self):
        values = np.linspace(0.0, 0.13, 14)
        left_names, left = driver_command(values, "left")
        right_names, right = driver_command(values, "right")
        self.assertEqual(left_names, ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "gripper"])
        self.assertEqual(right_names, left_names)
        np.testing.assert_allclose(left, values[:7])
        np.testing.assert_allclose(right, values[7:])

    def test_14_21_bridge_preserves_arm_values(self):
        left = np.array([0.1, 0.2, -0.3, 0.4, -0.5, 0.6, 0.08])
        right = np.array([-0.1, 0.3, -0.4, 0.5, 0.6, -0.7, 0.02])
        real = combine_feedback(left, right)
        sim = to_sim_contract(real)
        recovered = arms_from_sim_contract(sim, real)
        np.testing.assert_allclose(recovered, real)
        np.testing.assert_allclose(sim[15:19], [0.04, -0.04, 0.01, -0.01])
        np.testing.assert_allclose(from_sim_contract(sim), real)

    def test_clamp_applies_arm_and_gripper_limits(self):
        values = np.full(14, 100.0)
        result = clamp_command(values)
        self.assertAlmostEqual(result[6], 0.1)
        self.assertAlmostEqual(result[13], 0.1)
        self.assertLess(result[0], 3.0)


if __name__ == "__main__":
    unittest.main()
