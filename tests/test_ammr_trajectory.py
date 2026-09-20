import unittest

import numpy as np

from ammr_teleop.common.trajectory import jerk_limited_step


class AmmrTrajectoryTest(unittest.TestCase):
    def test_fixed_target_settles_without_limit_violation(self):
        position = np.zeros(6)
        velocity = np.zeros(6)
        acceleration = np.zeros(6)
        target = np.full(6, 0.1)
        dt = 1.0 / 30.0
        for _ in range(600):
            old_acceleration = acceleration.copy()
            position, velocity, acceleration = jerk_limited_step(
                target,
                position,
                velocity,
                acceleration,
                dt,
                np.full(6, 0.2),
                np.full(6, 0.8),
                np.full(6, 4.0),
            )
            self.assertLessEqual(np.max(np.abs(velocity)), 0.2 + 1e-12)
            self.assertLessEqual(np.max(np.abs(acceleration)), 0.8 + 1e-12)
            self.assertLessEqual(
                np.max(np.abs(acceleration - old_acceleration)),
                4.0 * dt + 1e-12,
            )
        np.testing.assert_allclose(position, target, atol=1e-10)
        np.testing.assert_allclose(velocity, 0.0, atol=1e-10)
        np.testing.assert_allclose(acceleration, 0.0, atol=1e-10)

    def test_random_target_reversals_preserve_all_limits(self):
        rng = np.random.default_rng(20260808)
        position = np.zeros(6)
        velocity = np.zeros(6)
        acceleration = np.zeros(6)
        for _ in range(10_000):
            dt = float(rng.uniform(1.0 / 35.0, 1.0 / 25.0))
            old_acceleration = acceleration.copy()
            position, velocity, acceleration = jerk_limited_step(
                rng.uniform(-1.0, 1.0, 6),
                position,
                velocity,
                acceleration,
                dt,
                np.full(6, 0.2),
                np.full(6, 0.8),
                np.full(6, 4.0),
            )
            self.assertLessEqual(np.max(np.abs(velocity)), 0.2 + 1e-12)
            self.assertLessEqual(np.max(np.abs(acceleration)), 0.8 + 1e-12)
            self.assertLessEqual(
                np.max(np.abs(acceleration - old_acceleration) / dt),
                4.0 + 1e-10,
            )


if __name__ == "__main__":
    unittest.main()
