"""Regression checks for the palm-only natural-to-inward experiment."""

from pathlib import Path
from contextlib import redirect_stdout
import io
import sys
import unittest
from unittest.mock import patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

from mh6_mapping import MH6HandMapper
from mh6_palm_solution_selector import PalmSolutionSelector
from mh6_palm_solver_adapter import PalmAdapterCalibration, PalmSolverAdapter
from mh6_teleop_run import main, select_palm_motor_preview
from mh6_palm_solver_v2 import MH6PalmSolver


class InwardMappingTest(unittest.TestCase):
    def setUp(self):
        self.mapper = MH6HandMapper()
        self.intent = {"power_grasp": 0., "tripod_precision": 0., "opposition_cross": 0.}

    def test_outward_motion_stays_at_natural_without_half_range_remapping(self):
        command = self.mapper.compute_palm_commands(
            self.intent, -1.,
            signed_curls=dict.fromkeys(("thumb", "index", "middle", "ring", "little"), -1.),
            signed_opposition=dict.fromkeys(("p_I", "p_M", "p_R", "p_L"), -1.),
        )
        for name in ("vertical", "lateral", "thumb_rotation_measured", "thumb_rotation_command"):
            self.assertEqual(command[name], 0.)
        self.assertEqual(self.mapper.compute_palm_commands(self.intent, .5)["thumb_rotation_command"], .5)

    def test_outward_features_do_not_cancel_a_positive_grasp(self):
        command = self.mapper.compute_palm_commands(
            {**self.intent, "power_grasp": .2, "opposition_cross": .1}, -.8,
            signed_curls=dict.fromkeys(("thumb", "index", "middle", "ring", "little"), -1.),
            signed_opposition=dict.fromkeys(("p_I", "p_M", "p_R", "p_L"), -1.),
        )
        self.assertAlmostEqual(command["vertical"], .2)
        self.assertAlmostEqual(command["lateral"], .12)
        self.assertEqual(command["thumb_rotation_command"], 0.)

    def test_existing_nonnegative_grasp_contributions_are_preserved(self):
        old = MH6HandMapper(palm_motion_range="signed")
        for power in (0., .2, .8, 1.):
            for tripod in (0., .5, 1.):
                for thumb in (0., .3, 1.):
                    intent = {"power_grasp": power, "tripod_precision": tripod, "opposition_cross": .4}
                    self.assertEqual(self.mapper.compute_palm_commands(intent, thumb),
                                     old.compute_palm_commands(intent, thumb))

    def test_finger_bending_remains_separate_from_palm_range(self):
        curls = dict(zip(("thumb", "index", "middle", "ring", "little"), (-1., -.5, 0., .5, 1.)))
        self.assertEqual(self.mapper.compute_finger_bending_commands(curls),
                         MH6HandMapper(palm_motion_range="signed").compute_finger_bending_commands(curls))

    def test_invalid_motion_range_is_rejected(self):
        with self.assertRaises(ValueError):
            MH6HandMapper(palm_motion_range="unknown")


class InwardSolverTest(unittest.TestCase):
    def setUp(self):
        self.adapter = PalmSolverAdapter()
        self.core = MH6PalmSolver()

    def test_default_zero_intent_uses_original_solver_zero_input_pose(self):
        actual = self.adapter.solve_motor_from_teleop(0., 0., 0.)
        expected = self.core.solve_motor_safe_from_normalized(
            0., 0., 0., mode="workspace_conditional", motor_order="timeseries", project_invalid=False)
        self.assertEqual(actual["mapping_mode"], "workspace_conditional")
        self.assertEqual(actual["workspace_input"], {"u1": 0., "u2": 0., "u3": 0.})
        self.assertEqual(actual["requested_angles"], expected["requested_angles"])
        self.assertEqual(actual["candidates"], expected["solutions"])
        self.assertNotEqual(actual["requested_angles"], [0., 0., 0.])
        self.assertFalse(actual["projected"])

    def test_normalized_order_and_history_reach_unmodified_solver(self):
        previous = [247., 500., 500.]
        with patch.object(self.adapter.solver, "solve_motor_safe_from_normalized",
                          wraps=self.adapter.solver.solve_motor_safe_from_normalized) as solve:
            actual = self.adapter.solve_motor_from_teleop(.2, .3, .7, previous_motor=previous)
        self.assertEqual(solve.call_args.args, (.2, .7, .3))
        self.assertEqual(solve.call_args.kwargs["mode"], "workspace_conditional")
        self.assertEqual(solve.call_args.kwargs["previous_motor"], tuple(previous))
        self.assertFalse(solve.call_args.kwargs["project_invalid"])
        np.testing.assert_array_equal(actual["requested_angles"],
                                      self.core.map_normalized(.2, .7, .3, mode="workspace_conditional"))

    def test_negative_or_invalid_intent_is_rejected_before_geometry(self):
        for value in (-1., -1e-9, 1.01, float("nan"), float("inf"), True):
            with self.subTest(value=value), patch.object(self.adapter.solver, "map_normalized") as solve:
                with self.assertRaises(ValueError):
                    self.adapter.solve_motor_from_teleop(0., value, 0.)
                solve.assert_not_called()

    def test_entire_command_cube_reuses_original_solver_targets(self):
        for h in np.linspace(0., 1., 11):
            for v in np.linspace(0., 1., 11):
                for r in np.linspace(0., 1., 11):
                    with self.subTest(h=h, v=v, r=r):
                        actual = self.adapter.solve_motor_from_teleop(h, v, r)
                        np.testing.assert_array_equal(actual["requested_angles"],
                                                      self.core.map_normalized(h, r, v, mode="workspace_conditional"))
                        self.assertTrue(actual["candidates"])
                        self.assertFalse(actual["projected"])

    def test_physical_side_of_neutral_is_not_used_to_modify_solver_candidates(self):
        actual = self.adapter.solve_motor_from_teleop(0., 0., 0.)
        self.assertTrue(any(row[0] < 247. and row[2] > 500. for row in actual["solutions"]))
        self.assertEqual(actual["solutions"], actual["candidates"])

    def test_control_rejection_keeps_solver_request_unchanged(self):
        inputs = {"palm_command": {"vertical": 0., "lateral": 0., "thumb_rotation_command": 0.}}
        preview = select_palm_motor_preview(inputs, self.adapter, PalmSolutionSelector())
        self.assertTrue(preview[3].held_previous)
        np.testing.assert_array_equal(preview[3].solver_diagnostics["requested_angles"],
                                      self.core.map_normalized(0., 0., 0., mode="workspace_conditional"))

    def test_old_cli_profiles_are_rejected_before_stream_or_hardware_start(self):
        config = Path(__file__).resolve().parents[1] / "config"
        for options in (("--palm-solver", "legacy"),
                        ("--palm-adapter-config", str(config / "mh6_palm_adapter_calibration.json")),
                        ("--palm-adapter-config", str(config / "mh6_palm_adapter_workspace_conditional.json"))):
            with self.subTest(options=options), redirect_stdout(io.StringIO()), \
                    patch("mh6_teleop_run.VisionProHandStream", side_effect=AssertionError("stream started")), \
                    patch("mh6_teleop_run.HardwareSender", side_effect=AssertionError("hardware started")):
                self.assertEqual(main(list(options)), 2)

    def test_invalid_input_domain_is_rejected(self):
        with self.assertRaises(ValueError):
            PalmAdapterCalibration(input_domain="unknown")


if __name__ == "__main__":
    unittest.main()
