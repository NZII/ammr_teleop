import unittest

from ammr_teleop.common.control_profiles import available_profiles, load_profile


class AmmrRealControlProfileTest(unittest.TestCase):
    def test_piper_absolute_smooth_v1_has_validated_defaults(self):
        self.assertIn("piper_absolute_smooth_v1", available_profiles())
        profile = load_profile("piper_absolute_smooth_v1")
        parameters = profile["parameters"]
        self.assertEqual(parameters["wrist_mapping"], "absolute")
        self.assertEqual(parameters["xr_frequency"], 50.0)
        self.assertEqual(parameters["command_frequency"], 100.0)
        self.assertGreater(parameters["translation_scale"], 0.0)
        self.assertGreater(parameters["rotation_scale"], 0.0)
        self.assertGreater(parameters["arm_velocity"], 0.0)

    def test_piper_absolute_smooth_v2_matches_its_filename(self):
        self.assertIn("piper_absolute_smooth_v2", available_profiles())
        profile = load_profile("piper_absolute_smooth_v2")
        self.assertEqual(profile["name"], "piper_absolute_smooth_v2")
        self.assertEqual(profile["version"], 2)

    def test_unknown_profile_is_rejected(self):
        with self.assertRaises(ValueError):
            load_profile("does_not_exist")


if __name__ == "__main__":
    unittest.main()
