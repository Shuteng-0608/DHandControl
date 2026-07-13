import sys
import unittest
from pathlib import Path


SCRIPTS_DIR = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS_DIR))

from mh6_palm_solver import MH6PalmSolver
from mh6_palm_solution_selector import PalmSolutionSelector
from mh6_teleop_run import select_palm_motor_preview, solve_palm_motor_preview


class RecordingSolver:
    def __init__(self):
        self.inputs = None

    def solve_motor_from_normalized(self, u1, u2, u3):
        self.inputs = (u1, u2, u3)
        return [[101.0, 202.0, 303.0]]


class PalmSolverIntegrationTest(unittest.TestCase):
    def test_neutral_solution_uses_actual_hardware_motor_id_order(self) -> None:
        solutions = MH6PalmSolver().solve_motor_from_normalized(0.0, 0.0, 0.0)

        self.assertEqual(solutions, [[247.0, 500.0, 500.0]])

    def test_runtime_preview_selects_neutral_solver_solution(self) -> None:
        result = {
            "palm_command": {
                "vertical": 0.0,
                "lateral": 0.0,
                "thumb_rotation_command": 0.0,
            }
        }

        normalized, candidates, selection = select_palm_motor_preview(
            result,
            MH6PalmSolver(),
            PalmSolutionSelector(),
            timestamp=1.0,
        )

        self.assertEqual(tuple(normalized.values()), (0.0, 0.0, 0.0))
        self.assertEqual(candidates, [[247.0, 500.0, 500.0]])
        self.assertEqual(selection.selected_motor, [247.0, 500.0, 500.0])
        self.assertEqual(selection.status, "SELECTED")

    def test_low_dim_values_are_passed_in_expected_order(self) -> None:
        result = {
            "palm_command": {
                "vertical": 0.2,
                "lateral": 0.6,
                "thumb_rotation_command": 0.8,
            }
        }
        solver = RecordingSolver()

        normalized_inputs, motor_solutions = solve_palm_motor_preview(result, solver)

        self.assertEqual(solver.inputs, (0.2, 0.6, 0.8))
        self.assertEqual(
            normalized_inputs,
            {"palm_flexion": 0.2, "palm_cross": 0.6, "thumb_inward": 0.8},
        )
        self.assertEqual(motor_solutions, [[101.0, 202.0, 303.0]])

    def test_real_solver_returns_both_motor_branches(self) -> None:
        result = {
            "palm_command": {
                "vertical": -1.0,
                "lateral": 35.1 / 59.0,
                "thumb_rotation_command": 1.0,
            }
        }

        _, motor_solutions = solve_palm_motor_preview(result, MH6PalmSolver())

        self.assertEqual(len(motor_solutions), 2)
        for solution in motor_solutions:
            self.assertEqual(len(solution), 3)
            self.assertTrue(all(0.0 <= value <= 1000.0 for value in solution))

    def test_real_solver_exposes_no_solution(self) -> None:
        result = {
            "palm_command": {
                "vertical": 29.85 / 90.8,
                "lateral": -60.5 / 180.0,
                "thumb_rotation_command": -7.55 / 23.7,
            }
        }

        _, motor_solutions = solve_palm_motor_preview(result, MH6PalmSolver())

        self.assertEqual(motor_solutions, [])


class SignedPalmSolverMappingTest(unittest.TestCase):
    def setUp(self) -> None:
        self.solver = MH6PalmSolver()

    def test_piecewise_mapping_hits_negative_neutral_and_positive_points(self) -> None:
        self.assertEqual(
            self.solver.map_normalized(-1.0, -1.0, -1.0),
            (-31.1, -180.0, -23.7),
        )
        self.assertEqual(
            self.solver.map_normalized(0.0, 0.0, 0.0),
            (0.0, 0.0, 0.0),
        )
        self.assertEqual(
            self.solver.map_normalized(1.0, 1.0, 1.0),
            (90.8, 59.0, 8.6),
        )

    def test_negative_and_positive_halves_use_independent_slopes(self) -> None:
        negative = self.solver.map_normalized(-0.5, -0.5, -0.5)
        positive = self.solver.map_normalized(0.5, 0.5, 0.5)

        self.assertEqual(negative, (-15.55, -90.0, -11.85))
        self.assertEqual(positive, (45.4, 29.5, 4.3))

    def test_out_of_range_input_is_rejected_or_clipped_explicitly(self) -> None:
        with self.assertRaises(ValueError):
            self.solver.map_normalized(1.01, 0.0, 0.0)

        self.assertEqual(
            self.solver.map_normalized_safe(2.0, -2.0, 0.0, clip=True),
            (90.8, -180.0, 0.0),
        )


if __name__ == "__main__":
    unittest.main()
