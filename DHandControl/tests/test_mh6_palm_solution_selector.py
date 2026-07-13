import sys
import unittest
from pathlib import Path


SCRIPTS_DIR = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS_DIR))

from mh6_palm_solution_selector import PalmInputSlewLimiter, PalmSolutionSelector


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

    def test_initial_solution_still_obeys_jump_limit(self) -> None:
        selector = PalmSolutionSelector(
            max_normalized_speed_per_sec=2.0,
            nominal_dt=1.0,
            max_dt=0.10,
        )

        result = selector.select(
            (1.0, 1.0, 1.0),
            [[1000, 120, 401]],
            timestamp=1.0,
        )

        self.assertEqual(result.status, "HELD_NEUTRAL_INITIAL_JUMP_REJECTED")
        self.assertEqual(result.selected_motor, [247, 500, 500])

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

    def test_jump_allowance_caps_dt_after_long_gap(self) -> None:
        selector = PalmSolutionSelector(
            max_normalized_speed_per_sec=2.0,
            max_dt=0.10,
        )
        selector.select((0.0, 0.0, 0.0), [[247, 500, 500]], timestamp=1.0)

        result = selector.select(
            (1.0, 1.0, 1.0),
            [[1000, 120, 401]],
            timestamp=10.0,
        )

        self.assertEqual(result.status, "HELD_JUMP_REJECTED")


class PalmInputSlewLimiterTest(unittest.TestCase):
    def test_first_step_caps_nominal_dt(self) -> None:
        limiter = PalmInputSlewLimiter(
            max_speed_per_sec=(2.0, 2.0, 2.0),
            nominal_dt=1.0,
            max_dt=0.10,
        )

        first = limiter.apply((1.0, 1.0, 1.0), timestamp=1.0)

        self.assertEqual(first, (0.2, 0.2, 0.2))

    def test_limits_each_axis_and_caps_dt_after_gap(self) -> None:
        limiter = PalmInputSlewLimiter(
            max_speed_per_sec=(2.0, 1.0, 0.5),
            nominal_dt=0.05,
            max_dt=0.10,
        )

        first = limiter.apply((1.0, -1.0, 1.0), timestamp=1.0)
        after_gap = limiter.apply((1.0, -1.0, 1.0), timestamp=2.0)

        self.assertEqual(first, (0.1, -0.05, 0.025))
        self.assertAlmostEqual(after_gap[0], 0.3)
        self.assertAlmostEqual(after_gap[1], -0.15)
        self.assertAlmostEqual(after_gap[2], 0.075)

    def test_hold_restores_last_feasible_input(self) -> None:
        limiter = PalmInputSlewLimiter()
        limiter.apply((1.0, 1.0, 1.0), timestamp=1.0)

        limiter.hold((0.0, 0.0, 0.0))

        self.assertEqual(limiter.applied, [0.0, 0.0, 0.0])

if __name__ == "__main__":
    unittest.main()
