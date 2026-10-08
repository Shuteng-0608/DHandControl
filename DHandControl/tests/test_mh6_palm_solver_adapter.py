from contextlib import redirect_stdout
import io
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

import numpy as np


SCRIPTS_DIR = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS_DIR))

from mh6_hand_session import HandSessionRecorder
from mh6_palm_calibration import PALM_MOTOR_SAFE_LIMITS
from mh6_palm_fallback import PalmFallbackController
from mh6_palm_solution_selector import PalmInputSlewLimiter, PalmSolutionSelector
from mh6_palm_solver_adapter import (
    CONDITIONAL_CALIBRATION_PATH,
    DEFAULT_CALIBRATION_PATH,
    MOTOR_RANGE_NEUTRAL,
    PalmAdapterCalibration,
    PalmSolverAdapter,
)
from mh6_palm_solver_v2 import MH6PalmSolver
from mh6_teleop_run import (
    PalmDebugLogger,
    main,
    print_palm_control_preview,
    select_palm_control_preview,
    select_palm_motor_preview,
    solve_palm_motor_preview,
)
from visionpro_stream import VisionProHandFrame


def command(h, v, r):
    return {"palm_command": {
        "vertical": h, "lateral": v, "thumb_rotation_command": r,
    }}


def make_hand_frame(timestamp, amount):
    """A metre-scale hand with both bending and thumb rotation for replay QA."""

    points = np.zeros((27, 3))
    for start, base in (
        (5, (0.025, 0.04, 0)), (10, (0, 0.045, 0)),
        (15, (-0.012, 0.04, 0)), (20, (-0.025, 0.035, 0)),
    ):
        points[start] = base
        for offset in range(1, 5):
            angle = (offset - 1) * amount * 0.8
            points[start + offset] = points[start + offset - 1] + (
                0, 0.018 * np.cos(angle), -0.018 * np.sin(angle)
            )
    points[1] = (0.025, 0.015, 0)
    for offset in range(1, 4):
        angle = 0.3 + amount * 0.85 + (offset - 1) * amount * 0.3
        points[1 + offset] = points[offset] + (
            0.02 * np.cos(angle), 0.02 * np.sin(angle), 0
        )
    transforms = np.repeat(np.eye(4)[None], 27, axis=0)
    transforms[:, :3, 3] = points
    return VisionProHandFrame(points, transforms, timestamp, "right")


class PalmAdapterMappingTest(unittest.TestCase):
    def setUp(self):
        self.adapter = PalmSolverAdapter(PalmAdapterCalibration())

    def test_signed_anchors_use_robot_neutral_instead_of_cube_midpoint(self):
        self.assertEqual(self.adapter.map_teleop_to_workspace(-1, -1, -1), (0, 0, 0))
        self.assertEqual(self.adapter.map_teleop_to_workspace(0, 0, 0), MOTOR_RANGE_NEUTRAL)
        self.assertEqual(self.adapter.map_teleop_to_workspace(1, 1, 1), (1, 1, 1))

    def test_both_directions_map_to_expected_physical_angles(self):
        expected = {
            (-0.5, -0.5, -0.5): (-15.55, 29.5, 4.3),
            (0, 0, 0): (0, 0, 0),
            (0.5, 0.5, 0.5): (45.4, -90, -11.85),
        }
        for signed, target in expected.items():
            with self.subTest(signed=signed):
                result = self.adapter.solve_motor_from_teleop(*signed)
                for actual, wanted in zip(result["requested_angles"], target):
                    self.assertAlmostEqual(actual, wanted, places=10)

    def test_each_semantic_axis_reaches_its_own_solver_angle(self):
        # Unequal and single-axis inputs detect lateral/thumb permutations.
        cases = (
            ((1, 0, 0), (90.8, 0, 0)),
            ((-1, 0, 0), (-31.1, 0, 0)),
            ((0, 1, 0), (0, 0, -23.7)),
            ((0, -1, 0), (0, 0, 8.6)),
            ((0, 0, 1), (0, -180, 0)),
            ((0, 0, -1), (0, 59, 0)),
            ((0.2, 0.3, 0.7), (18.16, -126, -7.11)),
        )
        for signed, expected in cases:
            with self.subTest(signed=signed):
                with patch.object(self.adapter.solver, "solve_motor_safe",
                                  wraps=self.adapter.solver.solve_motor_safe) as solve:
                    result = self.adapter.solve_motor_from_teleop(*signed)
                for actual, wanted in zip(solve.call_args.args, expected):
                    self.assertAlmostEqual(actual, wanted, places=10)
                self.assertEqual(result["semantic_input"], dict(zip(
                    ("vertical", "lateral", "thumb_rotation_command"), signed
                )))

    def test_custom_calibration_anchors_remain_in_solver_angle_order(self):
        adapter = PalmSolverAdapter(PalmAdapterCalibration(neutral=(0.2, 0.3, 0.4)))
        # Calibration indices refer to arpha2/arpha3/theta1, hence h/r/v.
        workspace = adapter.map_teleop_to_workspace(0.25, -0.5, 0.75)
        for actual, expected in zip(workspace, (0.4, 0.825, 0.2)):
            self.assertAlmostEqual(actual, expected)

    def test_default_neutral_remains_existing_hardware_pose(self):
        result = self.adapter.solve_motor_from_teleop(0, 0, 0)
        self.assertTrue(result["success"])
        self.assertEqual(result["solutions"], [[247, 500, 500]])
        self.assertEqual(result["angle_delta_deg"], [0, 0, 0])
        self.assertFalse(result["projected"])

    def test_full_range_endpoint_roundoff_is_not_an_input_limit_failure(self):
        result = self.adapter.solve_motor_from_teleop(1, 0, 0)
        self.assertEqual(result["requested_angles"][0], 90.8)
        self.assertNotEqual(result["diagnostic"]["reason"], "input_limit")
        self.assertTrue(result["success"])

    def test_invalid_intent_is_rejected_before_calling_geometry(self):
        for value in (-1.01, 1.01, math.nan, math.inf, True, None):
            with self.subTest(value=value), patch.object(self.adapter.solver, "map_normalized") as core:
                with self.assertRaises(ValueError):
                    self.adapter.solve_motor_from_teleop(0, value, 0)
                core.assert_not_called()

    def test_all_grid_intents_stay_inside_normalized_contract(self):
        for h in np.linspace(-1, 1, 11):
            for v in np.linspace(-1, 1, 11):
                for r in np.linspace(-1, 1, 11):
                    workspace = self.adapter.map_teleop_to_workspace(h, v, r)
                    self.assertTrue(all(0 <= value <= 1 for value in workspace))

    def test_workspace_modes_require_a_deliberate_neutral_calibration(self):
        with self.assertRaisesRegex(ValueError, "explicit neutral"):
            PalmAdapterCalibration(mapping_mode="workspace_conditional")
        adapter = PalmSolverAdapter(PalmAdapterCalibration(
            mapping_mode="workspace_conditional", neutral=(0.5, 0.5, 0.5),
        ))
        result = adapter.solve_motor_from_teleop(0, 0, 0)
        self.assertEqual(result["workspace_input"], {"u1": 0.5, "u2": 0.5, "u3": 0.5})
        self.assertAlmostEqual(result["requested_angles"][0], 45)
        self.assertAlmostEqual(result["requested_angles"][2], -12.85)

    def test_conditional_profile_calls_normalized_api_with_semantic_order_and_history(self):
        adapter = PalmSolverAdapter(PalmAdapterCalibration.from_file(str(CONDITIONAL_CALIBRATION_PATH)))
        previous = [247, 500, 500]
        expected_u = adapter.map_teleop_to_workspace(0.2, 0.3, 0.7)
        with patch.object(adapter.solver, "solve_motor_safe_from_normalized",
                          wraps=adapter.solver.solve_motor_safe_from_normalized) as solve:
            result = adapter.solve_motor_from_teleop(0.2, 0.3, 0.7, previous_motor=previous)
        self.assertEqual(solve.call_args.args, expected_u)
        self.assertEqual(solve.call_args.kwargs["mode"], "workspace_conditional")
        self.assertEqual(solve.call_args.kwargs["previous_motor"], tuple(previous))
        self.assertFalse(solve.call_args.kwargs["project_invalid"])
        self.assertEqual(result["solver_entrypoint"], "solve_motor_safe_from_normalized")
        self.assertEqual(result["workspace_input"], dict(zip(("u1", "u2", "u3"), expected_u)))
        self.assertFalse(result["projected"])
        self.assertTrue(result["candidates"])

    def test_conditional_profile_reuses_input_coordinates_but_changes_physical_neutral(self):
        conditional = PalmSolverAdapter(PalmAdapterCalibration.from_file(str(CONDITIONAL_CALIBRATION_PATH)))
        for signed in ((0, 0, 0), (-.5, .3, .8), (1, 1, 1)):
            self.assertEqual(conditional.map_teleop_to_workspace(*signed),
                             self.adapter.map_teleop_to_workspace(*signed))
        result = conditional.solve_motor_from_teleop(0, 0, 0)
        np.testing.assert_allclose(result["requested_angles"],
                                   [25.41017227235439, 19.10204580285152, -7.777708978328174])
        self.assertNotEqual(result["requested_angles"], [0, 0, 0])

    def test_conditional_thumb_endpoint_is_feasible_without_projection(self):
        conditional = PalmSolverAdapter(PalmAdapterCalibration.from_file(str(CONDITIONAL_CALIBRATION_PATH)))
        for h, v in ((-1, -1), (-.5, .15), (.3, -.4), (1, 1)):
            result = conditional.solve_motor_from_teleop(h, v, 1)
            self.assertTrue(result["candidates"])
            self.assertFalse(result["projected"])
            self.assertEqual(result["requested_angles"], result["used_angles"])
            self.assertGreater(result["requested_angles"][1], -180)

    def test_bundled_calibration_matches_native_conditional_zero(self):
        adapter = PalmSolverAdapter(PalmAdapterCalibration.from_file(str(DEFAULT_CALIBRATION_PATH)))
        result = adapter.solve_motor_from_teleop(0, 0, 0)
        expected = MH6PalmSolver().solve_motor_safe_from_normalized(
            0, 0, 0, mode="workspace_conditional", motor_order="timeseries")
        self.assertEqual(result["candidates"], expected["solutions"])
        self.assertEqual(result["requested_angles"], expected["requested_angles"])

    def test_malformed_calibration_is_rejected(self):
        for kwargs in (
            {"neutral": (0, 0.5, 0.5)},
            {"inward": (1.1, 1, 1)},
            {"neutral": (0.5, math.nan, 0.5)},
            {"mapping_mode": "typo"},
            {"project_invalid": "false"},
        ):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                PalmAdapterCalibration(**kwargs)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "calibration.json"
            path.write_text('{"neutarl": [0.5, 0.5, 0.5]}', encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "unknown"):
                PalmAdapterCalibration.from_file(str(path))


class PalmAdapterGeometryTest(unittest.TestCase):
    def test_analytic_root_ids_are_stable_when_solver_candidates_are_reordered(self):
        adapter = PalmSolverAdapter(PalmAdapterCalibration())
        baseline = adapter.solve_motor_from_teleop(.5, .5, .1)
        identities = dict(zip(map(tuple, baseline["candidates"]), baseline["candidate_branch_ids"]))
        self.assertEqual(set(identities.values()), {"plus_acos", "minus_acos"})
        solve = adapter.solver.solve_motor_safe
        def reverse(*args, **kwargs):
            result = solve(*args, **kwargs)
            result["solutions"].reverse()
            return result
        with patch.object(adapter.solver, "solve_motor_safe", side_effect=reverse):
            changed = adapter.solve_motor_from_teleop(.5, .5, .1)
        self.assertEqual(dict(zip(map(tuple, changed["candidates"]), changed["candidate_branch_ids"])), identities)
        self.assertEqual(changed["candidates"], list(reversed(baseline["candidates"])))

    def test_no_solution_has_empty_branch_ids(self):
        result = PalmSolverAdapter(PalmAdapterCalibration()).solve_motor_from_teleop(.5, .5, .5)
        self.assertEqual(result["candidate_branch_ids"], [])

    def test_real_geometry_is_reordered_and_filtered_by_actual_motor_limits(self):
        adapter = PalmSolverAdapter(PalmAdapterCalibration())
        result = adapter.solve_motor_from_teleop(0.5, 0.5, 0.1)
        self.assertTrue(result["success"])
        self.assertEqual(len(result["candidates"]), 2)
        self.assertEqual(result["solutions"], [[322.3, 310, 412.7855]])
        self.assertEqual(result["rejected_motor_solutions"][0]["limit_violations"], [3])
        self.assertGreater(result["rejected_motor_solutions"][0]["motors"][2], 536)
        self.assertFalse(result["projected"])

    def test_all_motor_branches_rejected_is_distinct_from_no_geometry(self):
        result = PalmSolverAdapter(PalmAdapterCalibration()).solve_motor_from_teleop(0, 1, 0)
        self.assertFalse(result["success"])
        self.assertEqual(result["status"], "NO_VALID_MOTOR_SOLUTION")
        self.assertTrue(result["candidates"])
        self.assertFalse(result["solutions"])
        self.assertEqual(result["diagnostic"]["reason"], "ok")

    def test_default_does_not_project_an_infeasible_request(self):
        result = PalmSolverAdapter(PalmAdapterCalibration()).solve_motor_from_teleop(0.5, 0.5, 0.5)
        self.assertFalse(result["success"])
        self.assertEqual(result["status"], "NO_SOLUTION")
        self.assertEqual(result["diagnostic"]["reason"], "rotational_workspace")
        self.assertFalse(result["projected"])
        self.assertIsNone(result["used_angles"])

    def test_projection_requires_explicit_permission_and_three_axis_limits(self):
        with self.assertRaisesRegex(ValueError, "requires project_invalid"):
            PalmAdapterCalibration(enforce_margin=True)
        with self.assertRaisesRegex(ValueError, "explicit max_projection"):
            PalmAdapterCalibration(project_invalid=True)

    def test_projection_reports_actual_angle_delta_and_obeys_caps(self):
        # A known infeasible input from the delivered trajectory, expressed
        # using the adapter's inward-positive convention.
        signed = (-1.51394762 / 31.1, 0.916723861 / 23.7, 6.962459705 / 180)
        adapter = PalmSolverAdapter(PalmAdapterCalibration(
            project_invalid=True, max_projection_delta_deg=(10, 30, 5),
        ))
        result = adapter.solve_motor_from_teleop(*signed)
        self.assertTrue(result["success"])
        self.assertTrue(result["projected"])
        self.assertTrue(result["projection_within_limits"])
        self.assertGreater(max(abs(x) for x in result["angle_delta_deg"]), 0)
        for row in result["solutions"]:
            for motor_id, value in enumerate(row, start=1):
                low, high = PALM_MOTOR_SAFE_LIMITS[motor_id]
                self.assertTrue(low <= value <= high)
        restrictive = PalmSolverAdapter(PalmAdapterCalibration(
            project_invalid=True, max_projection_delta_deg=(0.001, 0.001, 0.001),
        )).solve_motor_from_teleop(*signed)
        self.assertFalse(restrictive["success"])
        self.assertEqual(restrictive["error"], "projection_delta_limit_exceeded")
        self.assertFalse(restrictive["projection_within_limits"])
        self.assertEqual(restrictive["candidates"], [])

    def test_delivered_reference_retains_both_branches_without_order_assumption(self):
        core = MH6PalmSolver()
        raw = core.solve_remaining(47, -80, -20)
        self.assertEqual(len(raw), 2)
        self.assertTrue(all(row[3] < 1e-12 and row[4] < 1e-4 for row in raw))
        # The delivered regression fixture has stable values but its branch
        # order depends on rounding of sub-machine-precision rotation errors.
        self.assertCountEqual(core.solve_motor_from_normalized(
            0.641, 0.582, 0.885, mode="motor_range"
        ), [
            [420.6188, 303.1454, 582.0766],
            [480.6452, 303.1454, 582.0766],
        ])


class PalmAdapterRuntimeTest(unittest.TestCase):
    def test_named_adapter_inputs_remain_correct_with_legacy_positional_order(self):
        adapter = PalmSolverAdapter(PalmAdapterCalibration())
        with patch.object(adapter, "solve_motor_from_teleop", wraps=adapter.solve_motor_from_teleop) as solve:
            inputs, candidates = solve_palm_motor_preview(command(0.5, 0.5, 0.1), adapter)
        solve.assert_called_once_with(
            vertical=0.5, lateral=0.5, thumb_rotation_command=0.1,
            previous_motor=None,
        )
        self.assertEqual(inputs, {
            "palm_flexion": 0.5, "palm_cross": 0.5, "thumb_inward": 0.1,
        })
        self.assertIn([322.3, 310, 412.7855], candidates)

    def test_runtime_uses_named_teleop_entry_and_passes_accepted_previous_motor(self):
        adapter = PalmSolverAdapter(PalmAdapterCalibration())
        selector = PalmSolutionSelector()
        select_palm_motor_preview(command(0, 0, 0), adapter, selector, timestamp=1)
        with patch.object(adapter, "solve_motor_from_teleop", wraps=adapter.solve_motor_from_teleop) as solve:
            preview = select_palm_motor_preview(command(0.01, 0, 0), adapter, selector, timestamp=1.05)
        solve.assert_called_once_with(
            vertical=0.01, lateral=0, thumb_rotation_command=0,
            previous_motor=[247, 500, 500],
        )
        self.assertEqual(preview[3].status, "SELECTED")
        self.assertEqual(preview[3].solver_diagnostics["semantic_input"]["vertical"], 0.01)

    def test_invalid_motor_candidate_is_held_without_clipping(self):
        selector = PalmSolutionSelector()
        limiter = PalmInputSlewLimiter(max_speed_per_sec=(20, 20, 20))
        preview = select_palm_motor_preview(
            command(0, 1, 0), PalmSolverAdapter(PalmAdapterCalibration()), selector,
            input_limiter=limiter, timestamp=1,
        )
        self.assertEqual(preview[3].status, "HELD_NEUTRAL_NO_VALID_SOLUTION")
        self.assertEqual(preview[3].selected_motor, [247, 500, 500])
        self.assertEqual(limiter.applied, [0, 0, 0])
        self.assertGreater(preview[2][0][2], 536)

    def test_console_and_json_log_show_all_parameter_stages(self):
        preview = select_palm_control_preview(
            command(0, 0, 0), PalmSolverAdapter(PalmAdapterCalibration()), PalmSolutionSelector(),
            PalmFallbackController(), timestamp=1,
        )
        output = io.StringIO()
        with redirect_stdout(output):
            print_palm_control_preview(preview)
        self.assertIn("palm adapter preview", output.getvalue())
        self.assertIn("requestedAngles=", output.getvalue())
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "debug.jsonl"
            logger = PalmDebugLogger(str(path))
            logger.write(1, preview)
            logger.close()
            record = json.loads(path.read_text(encoding="utf-8"))
        adapter = record["solver"]["adapter"]
        self.assertEqual(adapter["mapping_mode"], "motor_range")
        self.assertEqual(adapter["angle_order"], ["arpha2", "arpha3", "theta1"])
        self.assertEqual(adapter["motor_order"], [1, 2, 3])
        self.assertEqual(adapter["solutions"], [[247, 500, 500]])
        self.assertEqual(record["frame_index"], 0)
        self.assertIn("candidate_scores", record["solver"]["branch_selection"])
        self.assertEqual(len(adapter["candidate_branch_ids"]), len(adapter["candidates"]))

    def test_hardware_lock_is_kept_before_opening_any_device(self):
        with patch("mh6_teleop_run.HardwareSender") as sender, redirect_stdout(io.StringIO()):
            status = main(["--enable-hardware"])
        self.assertEqual(status, 2)
        sender.assert_not_called()

    def test_invalid_branch_options_exit_before_replay_or_hardware(self):
        for options in (["--palm-branch-switch-penalty", "nan"],
                        ["--palm-branch-switch-penalty", "-1"],
                        ["--solver-only", "--palm-no-jump-guard"],
                        ["--solver-only", "--palm-fixed-branch", "plus_acos"],
                        ["--palm-solver", "legacy", "--palm-fixed-branch", "minus_acos"]):
            with self.subTest(options=options), patch("mh6_teleop_run.HardwareSender") as sender, redirect_stdout(io.StringIO()):
                self.assertEqual(main(options), 2)
                sender.assert_not_called()

    def test_full_replay_uses_default_adapter_and_logs_each_teleop_frame(self):
        with tempfile.TemporaryDirectory() as directory:
            session = Path(directory) / "hand.npz"
            log = Path(directory) / "debug.jsonl"
            recorder = HandSessionRecorder(str(session))
            phases = {
                "neutral": [0.2] * 8,
                "range": list(np.linspace(0, 1, 20)) + list(np.linspace(1, 0, 20)),
                "teleop": [0.2, 0.25, 0.3, 0.4, 0.6, 0.8, 1, 0.8, 0.6, 0.4, 0.2, 0.1, 0],
            }
            index = 0
            for phase, amounts in phases.items():
                for amount in amounts:
                    recorder.record(make_hand_frame(index * 0.05, amount), phase)
                    index += 1
            recorder.save()
            with redirect_stdout(io.StringIO()):
                status = main([
                    "--replay-session", str(session), "--replay-no-wait",
                    "--debug-log", str(log),
                ])
            records = [json.loads(line) for line in log.read_text(encoding="utf-8").splitlines()]
        self.assertEqual(status, 0)
        self.assertEqual(len(records), len(phases["teleop"]))
        self.assertEqual(records[0]["solver"]["selected_motor"], [247, 500, 500])
        for record in records:
            stage = record["solver"]["adapter"]
            self.assertEqual(stage["mapping_mode"], "workspace_conditional")
            self.assertEqual(stage["semantic_input"], {
                "vertical": record["solver"]["applied"]["palm_flexion"],
                "lateral": record["solver"]["applied"]["palm_cross"],
                "thumb_rotation_command": record["solver"]["applied"]["thumb_inward"],
            })
            self.assertTrue(all(0 <= q <= 1 for q in stage["workspace_input"].values()))
            for motor_id, value in enumerate(record["solver"]["selected_motor"], start=1):
                low, high = PALM_MOTOR_SAFE_LIMITS[motor_id]
                self.assertTrue(low <= value <= high)


if __name__ == "__main__":
    unittest.main()
