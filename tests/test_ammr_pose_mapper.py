import math
import unittest

import numpy as np

from ammr_teleop.common.pose_mapper import (
    FixedAbsoluteWristMapper,
    PoseMapperConfig,
    RelativeWristMapper,
    calibrated_translation_scale,
    fixed_xr_neutral_poses,
)
from ammr_teleop.common.kinematics import IKRegularizationConfig, _PassThroughSolutionFilter


def pose(x=0.0, y=0.0, z=0.0, yaw=0.0):
    result = np.eye(4)
    c = math.cos(yaw)
    s = math.sin(yaw)
    result[:3, :3] = [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]
    result[:3, 3] = [x, y, z]
    return result


class RelativeWristMapperTest(unittest.TestCase):
    def test_arm_length_calibration_computes_translation_gain(self):
        self.assertAlmostEqual(calibrated_translation_scale(0.65, 0.13), 0.20)

    def test_direct_translation_gain_overrides_arm_length_calibration(self):
        self.assertAlmostEqual(
            calibrated_translation_scale(0.65, 0.13, override=0.25), 0.25
        )

    def test_arm_length_calibration_rejects_invalid_values(self):
        with self.assertRaises(ValueError):
            calibrated_translation_scale(0.0, 0.13)
        with self.assertRaises(ValueError):
            calibrated_translation_scale(0.65, 0.0)
        with self.assertRaises(ValueError):
            calibrated_translation_scale(0.65, 0.13, override=-1.0)

    def test_ik_regularization_applies_to_all_joints_and_allows_ab_off(self):
        config = IKRegularizationConfig()
        self.assertAlmostEqual(config.motion_weight, 0.20)
        self.assertAlmostEqual(config.posture_weight, 0.01)
        disabled = IKRegularizationConfig(motion_weight=0.0, posture_weight=0.0)
        self.assertEqual(disabled.motion_weight, 0.0)
        with self.assertRaises(ValueError):
            IKRegularizationConfig(motion_weight=-0.1)

    def test_calibration_has_no_jump(self):
        mapper = RelativeWristMapper()
        left_robot = pose(0.3, 0.2, 0.8, yaw=0.3)
        right_robot = pose(0.3, -0.2, 0.8, yaw=-0.3)
        left_xr = pose(-0.1, 0.4, 1.2)
        right_xr = pose(0.1, 0.4, 1.2)
        mapper.calibrate(left_xr, right_xr, left_robot, right_robot)
        mapped_left, mapped_right = mapper.map(left_xr, right_xr)
        np.testing.assert_allclose(mapped_left, left_robot)
        np.testing.assert_allclose(mapped_right, right_robot)

    def test_translation_and_rotation_are_relative(self):
        mapper = RelativeWristMapper(PoseMapperConfig(translation_scale=2.0))
        identity = pose()
        mapper.calibrate(identity, identity, pose(0.3, 0.2, 0.8), pose())
        moved = pose(0.1, -0.05, 0.02, yaw=math.pi / 2)
        mapped, _ = mapper.map(moved, identity)
        np.testing.assert_allclose(mapped[:3, 3], [0.5, 0.1, 0.84])
        np.testing.assert_allclose(mapped[:3, :3], moved[:3, :3], atol=1e-7)

    def test_rotation_scale_reduces_relative_rotation(self):
        mapper = RelativeWristMapper(PoseMapperConfig(rotation_scale=0.5))
        identity = pose()
        mapper.calibrate(identity, identity, identity, identity)
        mapped, _ = mapper.map(pose(yaw=math.pi / 2), identity)
        expected = pose(yaw=math.pi / 4)
        np.testing.assert_allclose(mapped[:3, :3], expected[:3, :3], atol=1e-7)

    def test_translation_is_bounded(self):
        mapper = RelativeWristMapper(PoseMapperConfig(max_translation=0.2))
        identity = pose()
        mapper.calibrate(identity, identity, identity, identity)
        mapped, _ = mapper.map(pose(x=2.0), identity)
        self.assertAlmostEqual(np.linalg.norm(mapped[:3, 3]), 0.2)

    def test_invalid_pose_is_rejected(self):
        mapper = RelativeWristMapper()
        invalid = np.zeros((4, 4))
        with self.assertRaises(ValueError):
            mapper.calibrate(invalid, pose(), pose(), pose())

    def test_filter_holds_each_arm_independently(self):
        mapper = RelativeWristMapper(
            PoseMapperConfig(
                filter_alpha=0.5,
                translation_deadband=0.002,
                rotation_deadband=np.deg2rad(0.5),
            )
        )
        identity = pose()
        mapper.calibrate(identity, identity, identity, identity)
        mapper.map(identity, identity)

        filtered_left, filtered_right = mapper.map(pose(x=0.1), identity)

        self.assertAlmostEqual(float(filtered_left[0, 3]), 0.05, places=6)
        np.testing.assert_allclose(filtered_right, identity)

    def test_ik_pass_through_solution_filter_has_no_history_lag(self):
        filter_ = _PassThroughSolutionFilter(3)
        filter_.add_data([1.0, 2.0, 3.0])
        filter_.add_data([4.0, 5.0, 6.0])
        np.testing.assert_allclose(filter_.filtered_data, [4.0, 5.0, 6.0])

    def test_one_euro_filter_smooths_motion_and_preserves_independent_arms(self):
        mapper = RelativeWristMapper(
            PoseMapperConfig(
                filter_mode="one_euro",
                one_euro_min_cutoff=0.8,
                one_euro_beta=0.0,
            )
        )
        identity = pose()
        mapper.calibrate(identity, identity, identity, identity)
        mapper.map(identity, identity, timestamp=0.0)
        filtered_left, filtered_right = mapper.map(
            pose(x=0.1), identity, timestamp=0.1
        )
        self.assertGreater(float(filtered_left[0, 3]), 0.0)
        self.assertLess(float(filtered_left[0, 3]), 0.1)
        np.testing.assert_allclose(filtered_right, identity)


class FixedAbsoluteWristMapperTest(unittest.TestCase):
    def test_fixed_neutral_pose_maps_to_robot_neutral(self):
        xr_reference = fixed_xr_neutral_poses("controller")
        robot_reference = (pose(0.2, 0.3, 0.9), pose(0.2, -0.3, 0.9))
        mapper = FixedAbsoluteWristMapper(xr_reference, robot_reference)

        mapped_left, mapped_right = mapper.map(*xr_reference)

        np.testing.assert_allclose(mapped_left, robot_reference[0])
        np.testing.assert_allclose(mapped_right, robot_reference[1])

    def test_fixed_reference_does_not_change_after_clear(self):
        xr_reference = fixed_xr_neutral_poses("controller")
        robot_reference = (pose(0.2, 0.3, 0.9), pose(0.2, -0.3, 0.9))
        mapper = FixedAbsoluteWristMapper(
            xr_reference,
            robot_reference,
            PoseMapperConfig(filter_alpha=0.5),
        )
        moved_left = xr_reference[0].copy()
        moved_left[0, 3] += 0.1

        mapper.map(*xr_reference)
        mapper.clear()
        mapped_left, mapped_right = mapper.map(moved_left, xr_reference[1])

        self.assertAlmostEqual(float(mapped_left[0, 3]), 0.3, places=6)
        np.testing.assert_allclose(mapped_right, robot_reference[1])


if __name__ == "__main__":
    unittest.main()
