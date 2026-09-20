import unittest

import numpy as np

from ammr_teleop.common.contract import (
    FLAT_RECORD_INDEXES,
    POSITION_UPPER,
    ROBOT_JOINTS,
    clamp_sim_positions,
    flatten_record_mapping,
    gripper_pair,
    grouped_record_data,
    jerk_limit_positions,
    ordered_positions,
    rate_limit_positions,
)


class AmmrContractTest(unittest.TestCase):
    def test_jerk_limit_ramps_arm_and_gripper_acceleration(self):
        previous = np.zeros(21)
        desired = np.ones(21)
        command, velocity, acceleration = jerk_limit_positions(
            desired,
            previous,
            np.zeros(21),
            np.zeros(21),
            0.1,
            velocity_scale=0.35,
            arm_acceleration=0.8,
            gripper_acceleration=0.4,
            arm_jerk=4.0,
            gripper_jerk=2.0,
            enforce_position_limits=False,
        )
        self.assertAlmostEqual(acceleration[3], 0.4)
        self.assertAlmostEqual(velocity[3], 0.04)
        self.assertAlmostEqual(command[3], 0.004)
        self.assertAlmostEqual(acceleration[15], 0.2)
        self.assertAlmostEqual(velocity[15], 0.02)
        self.assertAlmostEqual(command[15], 0.002)
        # Chassis remains on its established velocity-only limiter.
        self.assertAlmostEqual(velocity[0], 0.35)
        self.assertAlmostEqual(command[0], 0.035)

    def test_sim_jerk_limit_brakes_before_reversing(self):
        previous = np.zeros(21)
        desired = -np.ones(21)
        prior_velocity = np.zeros(21)
        prior_velocity[3:15] = 0.2
        command, velocity, acceleration = jerk_limit_positions(
            desired,
            previous,
            prior_velocity,
            np.zeros(21),
            0.1,
            velocity_scale=0.35,
            arm_acceleration=0.8,
            gripper_acceleration=0.4,
            arm_jerk=4.0,
            gripper_jerk=2.0,
            enforce_position_limits=False,
        )
        self.assertAlmostEqual(acceleration[3], -0.4)
        self.assertAlmostEqual(velocity[3], 0.16)
        self.assertAlmostEqual(command[3], 0.016)

    def test_sim_contract_accepts_independent_arm_and_gripper_velocity_limits(self):
        command = np.zeros(21)
        velocity = np.zeros(21)
        acceleration = np.zeros(21)
        desired = np.full(21, 100.0)
        for _ in range(200):
            command, velocity, acceleration = jerk_limit_positions(
                desired,
                command,
                velocity,
                acceleration,
                0.01,
                velocity_scale=0.35,
                arm_velocity=0.50,
                gripper_velocity=0.10,
                arm_acceleration=1.6,
                gripper_acceleration=0.4,
                arm_jerk=5.0,
                gripper_jerk=2.0,
                enforce_position_limits=False,
            )
        self.assertAlmostEqual(velocity[3], 0.50, places=3)
        self.assertAlmostEqual(velocity[15], 0.10, places=3)
        self.assertAlmostEqual(velocity[0], 0.35)

    def test_named_state_is_reordered(self):
        values = np.arange(21, dtype=float)
        names = list(reversed(ROBOT_JOINTS))
        positions = list(reversed(values))
        np.testing.assert_array_equal(ordered_positions(names, positions), values)

    def test_gripper_mapping(self):
        self.assertEqual(gripper_pair(0.0), (0.0, -0.0))
        self.assertEqual(gripper_pair(1.0), (0.05, -0.05))
        self.assertEqual(gripper_pair(2.0), (0.05, -0.05))

    def test_rate_limit_and_position_limit(self):
        previous = np.zeros(21)
        desired = np.full(21, 100.0)
        limited = rate_limit_positions(desired, previous, 0.1)
        self.assertTrue(np.all(limited <= POSITION_UPPER))
        self.assertAlmostEqual(limited[0], 0.1)
        self.assertAlmostEqual(limited[3], 0.5)
        self.assertAlmostEqual(limited[19], 0.02)

    def test_rate_limit_does_not_jump_from_wrapped_sim_angle(self):
        previous = np.zeros(21)
        previous[3] = 4.7
        desired = previous.copy()
        desired[3] = 0.0
        limited = rate_limit_positions(desired, previous, 0.1)
        self.assertAlmostEqual(limited[3], 4.2)

    def test_replay_rate_limit_can_preserve_usd_wrapped_target(self):
        previous = np.zeros(21)
        desired = np.zeros(21)
        desired[3] = 4.7
        limited = rate_limit_positions(
            desired,
            previous,
            3.0,
            enforce_position_limits=False,
        )
        self.assertAlmostEqual(limited[3], 4.7)

    def test_sim_clamp_preserves_arm_wrap_but_clamps_gripper(self):
        values = np.zeros(21)
        values[3] = 4.7
        values[15] = 1.0
        result = clamp_sim_positions(values)
        self.assertEqual(result[3], 4.7)
        self.assertEqual(result[15], 0.05)

    def test_grouped_record_round_trip(self):
        command = np.linspace(-0.1, 0.1, 21)
        grouped = grouped_record_data(command)
        recovered = flatten_record_mapping(grouped)
        np.testing.assert_allclose(recovered, command)
        self.assertEqual(len(FLAT_RECORD_INDEXES), 21)


if __name__ == "__main__":
    unittest.main()
