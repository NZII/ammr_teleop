import unittest

import numpy as np

from ammr_teleop.real.safety import (
    ChassisLimits,
    apply_deadband,
    chassis_command,
    feedback_ik_seed,
    feedback_is_fresh,
    jerk_limit_command,
    rate_limit_command,
)


class AmmrRealSafetyTest(unittest.TestCase):
    def test_deadband_is_zero_and_rescaled(self):
        self.assertEqual(apply_deadband(0.05, 0.08), 0.0)
        self.assertAlmostEqual(apply_deadband(1.0, 0.08), 1.0)
        self.assertAlmostEqual(apply_deadband(-1.0, 0.08), -1.0)

    def test_chassis_mapping_is_planar(self):
        result = chassis_command([1.0, -1.0], [-1.0, 0.5], ChassisLimits())
        np.testing.assert_allclose(result, [0.20, -0.15, 0.30])

    def test_rate_limit_uses_separate_gripper_limit(self):
        previous = np.zeros(14)
        desired = np.ones(14)
        result = rate_limit_command(desired, previous, 0.1, 0.3, 0.05)
        self.assertAlmostEqual(result[0], 0.03)
        self.assertAlmostEqual(result[6], 0.005)
        self.assertAlmostEqual(result[13], 0.005)

    def test_feedback_ik_seed_uses_feedback_and_caps_large_gap(self):
        command = np.zeros(14)
        feedback = np.zeros(14)
        feedback[0] = 0.04
        feedback[1] = 0.50
        feedback[7] = -0.03
        result = feedback_ik_seed(command, feedback, {"left", "right"}, 0.12)
        self.assertAlmostEqual(result[0], 0.04)
        self.assertAlmostEqual(result[1], 0.12)
        self.assertAlmostEqual(result[7], -0.03)

    def test_feedback_ik_seed_does_not_change_inactive_arm(self):
        command = np.zeros(14)
        feedback = np.zeros(14)
        feedback[0] = 0.04
        feedback[7] = 0.09
        result = feedback_ik_seed(command, feedback, {"left"}, 0.12)
        self.assertAlmostEqual(result[0], 0.04)
        self.assertEqual(result[7], 0.0)

    def test_jerk_limiter_ramps_acceleration(self):
        previous = np.zeros(14)
        desired = np.ones(14)
        command, velocity, acceleration = jerk_limit_command(
            desired, previous, np.zeros(14), np.zeros(14),
            0.1, 0.3, 0.1, 0.8, 0.4, 4.0, 2.0
        )
        self.assertAlmostEqual(acceleration[0], 0.4)
        self.assertAlmostEqual(velocity[0], 0.04)
        self.assertAlmostEqual(command[0], 0.004)
        self.assertAlmostEqual(acceleration[6], 0.2)
        self.assertAlmostEqual(velocity[6], 0.02)
        self.assertAlmostEqual(command[6], 0.002)

    def test_jerk_limiter_clamps_out_of_range_previous_command(self):
        previous = np.zeros(14)
        previous[6] = 0.1049
        command, _, _ = jerk_limit_command(
            previous, previous, np.zeros(14), np.zeros(14),
            0.1, 0.3, 0.1, 0.8, 0.4, 4.0, 2.0
        )
        self.assertLessEqual(command[6], 0.1)

    def test_jerk_limiter_does_not_integrate_past_position_limit(self):
        previous = np.zeros(14)
        desired = np.zeros(14)
        velocity = np.zeros(14)
        previous[6] = 0.0999
        desired[6] = 0.1
        velocity[6] = 0.05
        command, next_velocity, next_acceleration = jerk_limit_command(
            desired, previous, velocity, np.zeros(14),
            0.01, 0.5, 0.1, 1.6, 0.4, 5.0, 2.0
        )
        self.assertEqual(command[6], 0.1)
        self.assertEqual(next_velocity[6], 0.0)
        self.assertEqual(next_acceleration[6], 0.0)

    def test_acceleration_limiter_does_not_snap_to_infeasible_target(self):
        previous = np.zeros(14)
        desired = np.zeros(14)
        desired[0] = 0.001
        prior_velocity = np.full(14, 0.2)
        prior_velocity[[6, 13]] = 0.1
        command, velocity, acceleration = jerk_limit_command(
            desired, previous, prior_velocity, np.zeros(14),
            0.1, 0.3, 0.1, 0.8, 0.4, 4.0, 2.0
        )
        self.assertGreater(command[0], desired[0])
        self.assertAlmostEqual(velocity[0], 0.16)
        self.assertAlmostEqual(acceleration[0], -0.4)

    def test_acceleration_limiter_brakes_before_reversing(self):
        previous = np.zeros(14)
        desired = -np.ones(14)
        prior_velocity = np.full(14, 0.2)
        prior_velocity[[6, 13]] = 0.1
        command, velocity, acceleration = jerk_limit_command(
            desired, previous, prior_velocity, np.zeros(14),
            0.1, 0.3, 0.1, 0.8, 0.4, 4.0, 2.0
        )
        self.assertAlmostEqual(velocity[0], 0.16)
        self.assertAlmostEqual(command[0], 0.016)
        self.assertAlmostEqual(acceleration[0], -0.4)

    def test_acceleration_limiter_lands_when_arrival_is_feasible(self):
        previous = np.zeros(14)
        desired = np.zeros(14)
        desired[0] = 0.001
        prior_velocity = np.zeros(14)
        prior_velocity[0] = 0.02
        command, velocity, acceleration = jerk_limit_command(
            desired, previous, prior_velocity, np.zeros(14),
            0.1, 0.3, 0.1, 0.8, 0.4, 4.0, 2.0
        )
        self.assertAlmostEqual(command[0], 0.001)
        self.assertAlmostEqual(velocity[0], 0.01)

    def test_acceleration_limiter_respects_limits_during_target_reversals(self):
        command = np.zeros(14)
        velocity = np.zeros(14)
        acceleration = np.zeros(14)
        dt = 1.0 / 30.0
        for cycle in range(120):
            # Exercise reversals on a joint whose +/-1 rad targets remain
            # inside its hard position range. Position-boundary saturation is
            # intentionally allowed to override jerk continuity.
            desired = np.zeros(14)
            desired[0] = 1.0 if (cycle // 10) % 2 == 0 else -1.0
            prior_velocity = velocity.copy()
            prior_acceleration = acceleration.copy()
            command, velocity, acceleration = jerk_limit_command(
                desired, command, velocity, acceleration,
                dt, 0.2, 0.1, 0.8, 0.4, 4.0, 2.0
            )
            self.assertLessEqual(np.max(np.abs(velocity[:6])), 0.2 + 1e-12)
            self.assertLessEqual(
                np.max(np.abs(velocity[:6] - prior_velocity[:6])),
                0.8 * dt + 1e-12,
            )
            self.assertLessEqual(
                np.max(np.abs(acceleration[:6] - prior_acceleration[:6])),
                4.0 * dt + 1e-12,
            )

    def test_feedback_freshness_requires_both_arms(self):
        self.assertTrue(feedback_is_fresh(10.0, [9.9, 9.8], 0.25))
        self.assertFalse(feedback_is_fresh(10.0, [9.9, 9.7], 0.25))
        self.assertFalse(feedback_is_fresh(10.0, [], 0.25))


if __name__ == "__main__":
    unittest.main()
