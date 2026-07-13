import sys
import unittest
from pathlib import Path


SCRIPTS_DIR = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS_DIR))

from mh6_palm_solution_selector import PalmSolutionSelector


class PalmSolutionSelectorTest(unittest.TestCase):
    def test_first_frame_selects_solution_closest_to_neutral(self) -> None:
        selector = PalmSolutionSelector()

        result = selector.select(
            (0.0, 0.0, 0.0),
            [[1000, 120, 401], [250, 500, 500]],
            timestamp=1.0,
        )

        self.assertEqual(result.selected_motor, [250.0, 500.0, 500.0])
        self.assertEqual(result.status, "SELECTED")

    def test_asymmetric_calibration_is_used_for_branch_distance(self) -> None:
        selector = PalmSolutionSelector(max_normalized_speed_per_sec=100.0)
        selector.select((0.0, 0.0, 0.0), [[247, 500, 500]], timestamp=1.0)

        result = selector.select(
            (0.1, 0.1, 0.1),
            [[347, 500, 536], [247, 550, 500]],
            timestamp=1.1,
        )

        self.assertEqual(result.selected_motor, [247.0, 550.0, 500.0])

    def test_no_solution_holds_previous_valid_motor(self) -> None:
        selector = PalmSolutionSelector()
        selector.select((0.0, 0.0, 0.0), [[247, 500, 500]], timestamp=1.0)

        result = selector.select((0.8, 0.8, 0.8), [], timestamp=1.05)

        self.assertEqual(result.selected_motor, [247.0, 500.0, 500.0])
        self.assertEqual(result.status, "HELD_NO_SOLUTION")
        self.assertTrue(result.held_previous)
        self.assertEqual(selector.previous_valid_input, (0.0, 0.0, 0.0))

    def test_invalid_solution_holds_previous(self) -> None:
        selector = PalmSolutionSelector()
        selector.select((0.0, 0.0, 0.0), [[247, 500, 500]], timestamp=1.0)

        result = selector.select((0.2, 0.2, 0.2), [[247, 700, 500]], timestamp=1.05)

        self.assertEqual(result.status, "HELD_NO_VALID_SOLUTION")
        self.assertEqual(result.selected_motor, [247.0, 500.0, 500.0])

    def test_excessive_jump_is_rejected(self) -> None:
        selector = PalmSolutionSelector(max_normalized_speed_per_sec=2.0)
        selector.select((0.0, 0.0, 0.0), [[247, 500, 500]], timestamp=1.0)

        result = selector.select(
            (1.0, 1.0, 1.0),
            [[1000, 120, 401]],
            timestamp=1.05,
        )

        self.assertEqual(result.status, "HELD_JUMP_REJECTED")
        self.assertEqual(result.selected_motor, [247.0, 500.0, 500.0])
        self.assertEqual(selector.previous_valid_input, (0.0, 0.0, 0.0))


if __name__ == "__main__":
    unittest.main()
