import math
import unittest
from unittest.mock import patch

from solve_from_arpha2_arpha3_theta1 import MH6PalmSolver


class MH6PalmSolverTests(unittest.TestCase):
    def setUp(self):
        self.solver = MH6PalmSolver()

    def test_solidworks_reference_has_two_precise_solutions(self):
        solutions = self.solver.solve_remaining(47, -80, -20)
        self.assertEqual(len(solutions), 2)
        expected = [(-54.2312, 75.6617, -4.3990), (-34.1784, 50.0236, -19.0601)]
        for solution, target in zip(solutions, expected):
            for actual, wanted in zip(solution[:3], target):
                self.assertAlmostEqual(actual, wanted, places=3)
            self.assertLess(solution[3], 1e-12)
            self.assertLess(solution[4], 1e-4)

    def test_legacy_motor_range_mapping_is_preserved(self):
        mapped = self.solver.map_normalized(0.641, 0.582, 0.885, mode="motor_range")
        self.assertAlmostEqual(mapped[0], 47.0379, places=4)
        self.assertAlmostEqual(mapped[1], -80.098, places=3)
        self.assertAlmostEqual(mapped[2], -19.9855, places=4)
        motors = self.solver.solve_motor_from_normalized(
            0.641, 0.582, 0.885, mode="motor_range")
        self.assertEqual(motors, [
            [420.6188, 303.1454, 582.0766],
            [480.6452, 303.1454, 582.0766],
        ])

    def test_default_conditional_mapping_stays_inside_margin(self):
        for u1 in (0.0, 0.25, 0.5, 0.75, 1.0):
            for u2 in (0.0, 0.25, 0.5, 0.75, 1.0):
                for u3 in (0.0, 0.25, 0.5, 0.75, 1.0):
                    angles = self.solver.map_normalized(u1, u2, u3)
                    self.assertLessEqual(abs(self.solver.closure_value(*angles)), 0.9990001)
                    solutions = self.solver.solve_remaining(*angles)
                    self.assertEqual(len(solutions), 2)

    def test_workspace_independent_is_high_coverage_but_not_guaranteed(self):
        valid = 0
        total = 0
        for i in range(11):
            for j in range(11):
                for k in range(11):
                    angles = self.solver.map_normalized(
                        i / 10, j / 10, k / 10, mode="workspace_independent")
                    valid += bool(self.solver.solve_remaining(*angles))
                    total += 1
        rate = valid / total
        self.assertGreater(rate, 0.90)
        self.assertLess(rate, 1.0)

    def test_diagnostic_distinguishes_workspace_and_limits(self):
        legacy_corner = self.solver.map_motor_range_normalized(0, 0, 0)
        diagnostic = self.solver.diagnose_input(*legacy_corner)
        self.assertFalse(diagnostic["valid"])
        self.assertEqual(diagnostic["reason"], "rotational_workspace")
        self.assertLess(diagnostic["closure_margin"], 0)

        diagnostic = self.solver.diagnose_input(100, 0, 0)
        self.assertFalse(diagnostic["valid"])
        self.assertEqual(diagnostic["reason"], "input_limit")
        self.assertIn("arpha2", diagnostic["limit_violations"])

    def test_projection_repairs_invalid_input_and_reports_delta(self):
        requested = (-1.51394762, -6.962459705, -0.916723861)
        projection = self.solver.project_angles_to_workspace(*requested)
        self.assertTrue(projection["projected"])
        self.assertGreater(projection["normalized_distance"], 0)
        self.assertAlmostEqual(abs(projection["closure_value_after"]), 0.999, places=8)
        self.assertEqual(len(self.solver.solve_remaining(*projection["angles"])), 2)

    def test_safe_solver_can_fail_explicitly_or_project(self):
        requested = (-1.51394762, -6.962459705, -0.916723861)
        failed = self.solver.solve_motor_safe(*requested, project_invalid=False)
        self.assertFalse(failed["success"])
        self.assertEqual(failed["diagnostic"]["reason"], "rotational_workspace")

        repaired = self.solver.solve_motor_safe(*requested, project_invalid=True)
        self.assertTrue(repaired["success"])
        self.assertTrue(repaired["projected"])
        self.assertEqual(len(repaired["solutions"]), 2)
        self.assertIsNotNone(repaired["selected"])

    def test_safe_solver_does_not_project_by_default(self):
        requested = (-1.51394762, -6.962459705, -0.916723861)
        result = self.solver.solve_motor_safe(*requested)
        self.assertFalse(result["success"])
        self.assertFalse(result["projected"])
        self.assertEqual(result["error"], "rotational_workspace")

    def test_projection_delta_limit_can_reject_large_motion_change(self):
        requested = (-1.51394762, -6.962459705, -0.916723861)
        result = self.solver.solve_motor_safe(
            *requested,
            project_invalid=True,
            max_projection_delta_deg=[0.01, 0.01, 0.01],
        )
        self.assertFalse(result["success"])
        self.assertTrue(result["projected"])
        self.assertFalse(result["projection_within_limits"])
        self.assertEqual(result["error"], "projection_delta_limit_exceeded")

    def test_safe_solver_reports_projection_runtime_failure(self):
        requested = (-1.51394762, -6.962459705, -0.916723861)
        with patch.object(
                self.solver, "project_angles_to_workspace",
                side_effect=RuntimeError("synthetic failure")):
            result = self.solver.solve_motor_safe(*requested, project_invalid=True)
        self.assertFalse(result["success"])
        self.assertIn("workspace_projection_failed", result["error"])

    def test_motor_order_and_continuity_selection(self):
        api = self.solver.solve_motor(47, -80, -20)
        historical = self.solver.reorder_motor_solution(api[0], "timeseries")
        self.assertEqual(historical, [api[0][2], api[0][1], api[0][0]])

        selected, index = self.solver.select_continuous_motor_solution(
            api, previous_motor=api[1])
        self.assertEqual(index, 1)
        self.assertEqual(selected, api[1])

    def test_periodic_input_aliases_use_canonical_motor_calibration(self):
        reference = self.solver.solve_motor(47, -80, -20)
        self.assertEqual(self.solver.solve_motor(407, 280, -20), reference)
        diagnostic = self.solver.diagnose_input(407, 280, -20)
        self.assertTrue(diagnostic["valid"])

    def test_safe_solver_filters_out_of_range_motor_solutions(self):
        with patch.object(
                self.solver, "solve_motor",
                return_value=[[-1.0, 500.0, 500.0], [500.0, 500.0, 500.0]]):
            result = self.solver.solve_motor_safe(47, -80, -20)
        self.assertTrue(result["success"])
        self.assertEqual(result["solutions"], [[500.0, 500.0, 500.0]])
        self.assertEqual(result["rejected_motor_solutions"], [[-1.0, 500.0, 500.0]])

    def test_normalized_input_validation(self):
        for values in ((-0.1, 0, 0), (0, 1.1, 0), (0, 0, math.nan)):
            with self.assertRaises(ValueError):
                self.solver.map_normalized(*values)


if __name__ == "__main__":
    unittest.main()
