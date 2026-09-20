import unittest

from ammr_teleop.xr.frontend import (
    controller_open_fraction,
    hand_open_fraction,
)


class AmmrXrInputTest(unittest.TestCase):
    def test_controller_gripper_mapping(self):
        self.assertEqual(controller_open_fraction(10.0), 1.0)
        self.assertEqual(controller_open_fraction(0.0), 0.0)
        self.assertEqual(controller_open_fraction(-1.0), 0.0)

    def test_hand_gripper_mapping(self):
        self.assertEqual(hand_open_fraction(4.0), 0.0)
        self.assertEqual(hand_open_fraction(5.0), 0.0)
        self.assertEqual(hand_open_fraction(6.0), 0.5)
        self.assertEqual(hand_open_fraction(7.0), 1.0)
        self.assertEqual(hand_open_fraction(15.0), 1.0)


if __name__ == "__main__":
    unittest.main()
