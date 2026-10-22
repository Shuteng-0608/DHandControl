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

    def test_default_zero_intent_is_exact_physical_open_pose(self):
        actual = self.adapter.solve_motor_from_teleop(0., 0., 0.)
        self.assertEqual(actual["mapping_mode"], "workspace_conditional")
        self.assertEqual(actual["workspace_profile"], "closing_only")
        self.assertEqual(actual["workspace_input"], {"u1": 0., "u2": 0., "u3": 0.})
        self.assertEqual(actual["requested_angles"], [0., 0., 0.])
        self.assertEqual(actual["solutions"], [[247., 500., 500.]])
        self.assertEqual(actual["candidate_branch_ids"], ["plus_acos"])
        self.assertFalse(actual["projected"])

    def test_normalized_order_and_history_reach_unmodified_solver(self):
        previous = [247., 500., 500.]
        with patch.object(self.adapter.solver, "solve_motor_safe",
                          wraps=self.adapter.solver.solve_motor_safe) as solve:
            actual = self.adapter.solve_motor_from_teleop(.2, .3, .7, previous_motor=previous)
        self.assertEqual(actual["workspace_input"], {"u1": .2, "u2": .7, "u3": .3})
        self.assertEqual(solve.call_args.args, tuple(actual["requested_angles"]))
        self.assertEqual(solve.call_args.kwargs["previous_motor"], tuple(previous))
        self.assertFalse(solve.call_args.kwargs["project_invalid"])
        self.assertEqual(actual["solver_entrypoint"], "solve_motor_safe")

    def test_negative_or_invalid_intent_is_rejected_before_geometry(self):
        for value in (-1., -1e-9, 1.01, float("nan"), float("inf"), True):
            with self.subTest(value=value), patch.object(self.adapter.solver, "map_normalized") as solve:
                with self.assertRaises(ValueError):
                    self.adapter.solve_motor_from_teleop(0., value, 0.)
                solve.assert_not_called()

    def test_entire_command_cube_stays_on_inward_side_with_closed_geometry(self):
        for h in np.linspace(0., 1., 11):
            for v in np.linspace(0., 1., 11):
                for r in np.linspace(0., 1., 11):
                    with self.subTest(h=h, v=v, r=r):
                        actual = self.adapter.solve_motor_from_teleop(h, v, r)
                        self.assertTrue(actual["solutions"])
                        for motor in actual["solutions"]:
                            self.assertTrue(247 <= motor[0] <= 1000)
                            self.assertTrue(120 <= motor[1] <= 500)
                            self.assertTrue(401 <= motor[2] <= 500)
                        self.assertTrue(self.core.diagnose_input(*actual["used_angles"])["valid"])
                        self.assertFalse(actual["projected"])

    def test_outward_alternate_branch_is_diagnostic_only(self):
        actual = self.adapter.solve_motor_from_teleop(.2, .02, .2)
        self.assertTrue(any(row[2] > 500. for row in actual["candidates"]))
        self.assertTrue(all(row[2] <= 500. for row in actual["solutions"]))
        self.assertEqual(actual["required_branch"], "plus_acos")

    def test_controller_accepts_the_coalesced_open_branch(self):
        inputs = {"palm_command": {"vertical": 0., "lateral": 0., "thumb_rotation_command": 0.}}
        preview = select_palm_motor_preview(inputs, self.adapter, PalmSolutionSelector())
        self.assertFalse(preview[3].held_previous)
        np.testing.assert_array_equal(preview[3].selected_motor, [247.,500.,500.])
        self.assertEqual(preview[3].solver_diagnostics["requested_angles"], [0.,0.,0.])

    def test_near_open_motor_roundoff_does_not_lose_branch_identity(self):
        for h, r in ((.001,0.),(.1,1.),(.5,1.)):
            self.assertTrue(self.adapter.solve_motor_from_teleop(h,1e-8,r)["success"])

    def test_coupled_axes_at_zero_grasp_are_reported_not_forced_outward(self):
        actual = self.adapter.solve_motor_from_teleop(0.,1.,1.)
        self.assertEqual(actual["solutions"], [[247.,500.,500.]])
        self.assertTrue(actual["workspace_diagnostic"]["collapsed_thumb_axis"])
        self.assertTrue(actual["workspace_diagnostic"]["collapsed_lateral_axis"])

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
