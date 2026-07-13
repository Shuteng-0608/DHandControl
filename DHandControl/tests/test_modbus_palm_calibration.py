import sys
import unittest
from pathlib import Path
from unittest.mock import Mock


SCRIPTS_DIR = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS_DIR))

from modbus_dev import (
    DexHandControl,
    PALM_MOTOR_CALIBRATION,
    PALM_MOTOR_SAFE_LIMITS,
)


class PalmMotorCalibrationTest(unittest.TestCase):
    def test_three_point_calibration_matches_actual_motor_ids(self) -> None:
        self.assertEqual(
            PALM_MOTOR_CALIBRATION,
            {
                1: {"outward": 0, "neutral": 247, "inward": 1000},
                2: {"outward": 630, "neutral": 500, "inward": 120},
                3: {"outward": 536, "neutral": 500, "inward": 401},
            },
        )
        self.assertEqual(
            PALM_MOTOR_SAFE_LIMITS,
            {1: (0, 1000), 2: (120, 630), 3: (401, 536)},
        )

    def test_actual_motor_targets_are_validated_and_rounded(self) -> None:
        hand = DexHandControl.__new__(DexHandControl)
        hand.palm_safe_limits = PALM_MOTOR_SAFE_LIMITS.copy()

        self.assertEqual(
            hand.validate_palm_motor_positions([247.2, 500.4, 499.6]),
            {1: 247, 2: 500, 3: 500},
        )

        with self.assertRaises(ValueError):
            hand.validate_palm_motor_positions([247, 631, 500])

    def test_free_functions_use_each_device_open_or_outward_limit(self) -> None:
        hand = DexHandControl.__new__(DexHandControl)
        hand.finger_limit = {motor_id: (20 + motor_id, 1950) for motor_id in range(1, 6)}
        hand.palm_calibration = {
            motor_id: points.copy()
            for motor_id, points in PALM_MOTOR_CALIBRATION.items()
        }
        hand.move_hand = Mock(return_value=True)
        hand.move_palms = Mock(return_value=True)
        hand.move_fingers = Mock(return_value=True)

        self.assertTrue(hand.free_all())
        hand.move_hand.assert_called_once_with(
            finger_ids=[1, 2, 3, 4, 5],
            finger_positions=[21, 22, 23, 24, 25],
            palm_ids=[1, 2, 3],
            palm_positions=[0, 630, 536],
            palm_times=[2000, 2000, 2000],
            wait_status=False,
        )

        self.assertTrue(hand.palm_free([2, 3]))
        hand.move_palms.assert_called_once_with(
            id_list=[2, 3],
            pos_list=[630, 536],
            time_list=[2000, 2000],
        )

        self.assertTrue(hand.finger_free([1, 5]))
        hand.move_fingers.assert_called_once_with(
            id_list=[1, 5],
            pos_list=[21, 25],
        )


if __name__ == "__main__":
    unittest.main()
