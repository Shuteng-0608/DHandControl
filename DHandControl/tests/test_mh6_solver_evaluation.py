from contextlib import redirect_stdout
from dataclasses import asdict
import io
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import Mock, patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

from mh6_hand_session import HandSessionRecorder
from mh6_palm_solver import MH6PalmSolver as LegacySolver
from mh6_palm_solver_adapter import PalmAdapterCalibration, PalmSolverAdapter
from mh6_solver_evaluation import PalmSolverTestRecorder, evaluate_palm_solver
from mh6_teleop_run import main
from visionpro_stream import VisionProHandFrame


def command(h, v, r):
    return {"palm_command": {
        "vertical": h, "lateral": v, "thumb_rotation_command": r,
    }}


def frame(timestamp):
    points = np.zeros((27, 3))
    points[9] = (0, 0.15, 0)
    transforms = np.repeat(np.eye(4)[None], 27, axis=0)
    transforms[:, :3, 3] = points
    return VisionProHandFrame(points, transforms, timestamp, "right")


class SolverObservationTest(unittest.TestCase):
    def test_named_inputs_log_and_solve_the_correct_unequal_angle_triplet(self):
        observation = evaluate_palm_solver(command(0.2, 0.3, 0.7), PalmSolverAdapter())
        self.assertEqual(observation["input"], {
            "palm_flexion": 0.2, "palm_cross": 0.3, "thumb_inward": 0.7,
        })
        np.testing.assert_allclose(observation["requested_angles"], [18.16, -126, -7.11],
                                   rtol=0, atol=1e-10)
        self.assertEqual(observation["adapter"]["angle_input_order"],
                         ["vertical", "thumb_rotation_command", "lateral"])

    def test_keeps_all_geometry_branches_and_identifies_valid_motor_branch(self):
        observation = evaluate_palm_solver(command(0.5, 0.5, 0.1), PalmSolverAdapter())
        self.assertTrue(observation["has_solution"])
        self.assertTrue(observation["has_valid_motor_solution"])
        self.assertEqual(observation["candidate_count"], 2)
        self.assertEqual(observation["valid_motor_candidates"], [[322.3, 310, 412.7855]])

    def test_motor_limit_failure_still_counts_as_geometric_solution(self):
        observation = evaluate_palm_solver(command(0, 1, 0), PalmSolverAdapter())
        self.assertTrue(observation["has_solution"])
        self.assertFalse(observation["has_valid_motor_solution"])
        self.assertEqual(observation["status"], "NO_VALID_MOTOR_SOLUTION")

    def test_infeasible_pose_does_not_return_a_held_neutral_motor(self):
        observation = evaluate_palm_solver(command(0.5, 0.5, 0.5), PalmSolverAdapter())
        self.assertFalse(observation["has_solution"])
        self.assertEqual(observation["candidates"], [])
        self.assertNotIn("selected_motor", observation)

    def test_no_motor_history_is_passed_after_a_solved_pose(self):
        adapter = PalmSolverAdapter()
        with patch.object(adapter, "solve_motor_from_teleop", wraps=adapter.solve_motor_from_teleop) as solve:
            evaluate_palm_solver(command(0, 0, 0), adapter)
            evaluate_palm_solver(command(0.5, 0.5, 0.1), adapter)
        self.assertTrue(all(call.kwargs["previous_motor"] is None for call in solve.call_args_list))

    def test_legacy_solver_keeps_its_positional_contract(self):
        observation = evaluate_palm_solver(command(0.2, 0.3, -0.4), LegacySolver())
        self.assertEqual(observation["requested_angles"], list(LegacySolver().map_normalized(0.2, -0.4, 0.3)))
        self.assertEqual(observation["candidates"], LegacySolver().solve_motor_from_normalized(0.2, -0.4, 0.3))


class SolverTestRuntimeTest(unittest.TestCase):
    def replay(self, folder, *, filtered=False, speed=1, default_log=False):
        session = Path(folder) / "hand.npz"
        log = Path(folder) / "solver.jsonl"
        recorder = HandSessionRecorder(str(session))
        # Calibration frames must never enter the rate's denominator.
        for index, phase in enumerate(["neutral", "range", "teleop", "teleop", "teleop"]):
            recorder.record(frame(index * 0.05), phase)
        recorder.save()
        with (
            patch("mh6_teleop_run.MH6HandMapper.step", side_effect=[
                command(0, 1, 0), command(0.5, 0.5, 0.1), command(0.5, 0.5, 0.5),
            ]),
            patch("mh6_teleop_run.print_mapping_line", side_effect=AssertionError("console mapping entered")),
            patch("mh6_teleop_run.__file__", str(Path(folder) / "DHandControl/scripts/mh6_teleop_run.py")),
            patch("mh6_teleop_run.PalmSolutionSelector", side_effect=AssertionError("selector entered")),
            patch("mh6_teleop_run.PalmInputSlewLimiter", side_effect=AssertionError("slew entered")),
            patch("mh6_teleop_run.PalmFallbackController", side_effect=AssertionError("fallback entered")),
            patch("mh6_teleop_run.HardwareSender", side_effect=AssertionError("hardware entered")),
            redirect_stdout(io.StringIO()) as output,
        ):
            status = main([
                "--solver-only", "--replay-session", str(session), "--use-default-calibration",
                "--replay-no-wait", "--replay-speed", str(speed),
                *([] if default_log else ["--debug-log", str(log)]),
                "--solver-input", "filtered" if filtered else "raw",
                "--print-every-frame", "--print-filtered",
            ])
        if default_log:
            logs = list((Path(folder) / "results/solver_only").glob("*.jsonl"))
            self.assertEqual(len(logs), 1)
            log = logs[0]
        rows = [json.loads(line) for line in log.read_text().splitlines()]
        summary = json.loads(log.with_suffix(".summary.json").read_text())
        return status, rows, summary, output.getvalue()

    def test_raw_replay_measures_solver_returns_without_any_control_guard(self):
        with tempfile.TemporaryDirectory() as folder, patch(
            "mh6_teleop_run.CommandLowPassFilter", side_effect=AssertionError("filter entered")
        ):
            status, rows, summary, output = self.replay(folder)
        self.assertEqual(status, 0)
        self.assertEqual(summary["termination"], "completed")
        self.assertEqual(summary["total_frames"], 3)
        self.assertEqual(summary["geometry_solved_frames"], 2)
        self.assertEqual(summary["valid_motor_frames"], 1)
        self.assertAlmostEqual(summary["geometry_solution_rate"], 2 / 3)
        self.assertAlmostEqual(summary["valid_motor_solution_rate"], 1 / 3)
        self.assertEqual(summary["metadata"]["filter_tau"], 0)
        self.assertEqual([row["frame_index"] for row in rows], [0, 1, 2])
        self.assertAlmostEqual(rows[-1]["timestamp"], 0.1)
        self.assertEqual(rows[0]["solver"]["input"]["palm_cross"], 1)
        self.assertTrue(all("control" not in row for row in rows))
        self.assertEqual(output, "")

    def test_filtered_mode_records_exact_filtered_input_and_raw_mapping(self):
        with tempfile.TemporaryDirectory() as folder:
            status, rows, summary, output = self.replay(folder, filtered=True)
        self.assertEqual(status, 0)
        self.assertEqual(output, "")
        self.assertEqual(summary["metadata"]["input_source"], "filtered")
        self.assertEqual(summary["metadata"]["filter_tau"], 0.24)
        self.assertEqual(rows[0]["raw_mapping"]["palm_command"]["lateral"], 1)
        self.assertGreater(rows[0]["solver"]["input"]["palm_cross"], 0)
        self.assertLess(rows[0]["solver"]["input"]["palm_cross"], 1)

    def test_omitting_log_path_still_saves_every_frame_without_console_output(self):
        with tempfile.TemporaryDirectory() as folder:
            status, rows, summary, output = self.replay(folder, default_log=True)
        self.assertEqual(status, 0)
        self.assertEqual(output, "")
        self.assertEqual(len(rows), 3)
        self.assertEqual(summary["total_frames"], 3)
        self.assertTrue(all("thumb_rotation_command" in row["raw_mapping"]["palm_command"] for row in rows))

    def test_raw_results_are_independent_of_playback_speed(self):
        with tempfile.TemporaryDirectory() as folder:
            _, normal, normal_summary, _ = self.replay(folder)
            _, slow, slow_summary, _ = self.replay(folder, speed=0.5)
        self.assertEqual(normal_summary["geometry_solution_rate"], slow_summary["geometry_solution_rate"])
        self.assertEqual(
            [row["solver"]["candidates"] for row in normal],
            [row["solver"]["candidates"] for row in slow],
        )

    def test_ctrl_c_saves_observations_and_excludes_missing_frames(self):
        stream = Mock(no_wait=True, is_replay=False, speed=1)
        stream.get_latest_frame.side_effect = [frame(100), None, frame(100.1), KeyboardInterrupt]
        with tempfile.TemporaryDirectory() as folder:
            log = Path(folder) / "solver.jsonl"
            with (
                patch("mh6_teleop_run.VisionProHandStream", return_value=stream),
                patch("mh6_teleop_run.MH6HandMapper.step", return_value=command(0, 0, 0)),
                patch("mh6_teleop_run.print_mapping_line"),
                redirect_stdout(io.StringIO()),
            ):
                status = main(["--solver-only", "--use-default-calibration", "--debug-log", str(log)])
            summary = json.loads(log.with_suffix(".summary.json").read_text())
            self.assertEqual(len(log.read_text().splitlines()), 2)
        self.assertEqual(status, 0)
        self.assertEqual(summary["termination"], "interrupted")
        self.assertEqual(summary["total_frames"], 2)
        self.assertEqual(summary["geometry_solution_rate"], 1)
        stream.stop.assert_called_once()

    def test_empty_run_has_undefined_rates(self):
        with tempfile.TemporaryDirectory() as folder:
            recorder = PalmSolverTestRecorder(None, str(Path(folder) / "summary.json"))
            with redirect_stdout(io.StringIO()):
                recorder.close("interrupted")
            summary = json.loads(recorder.summary_path.read_text())
        self.assertEqual(summary["total_frames"], 0)
        self.assertIsNone(summary["geometry_solution_rate"])
        self.assertIsNone(summary["valid_motor_solution_rate"])

    def test_summary_write_failure_still_saves_raw_recording_and_closes_stream(self):
        stream = Mock(no_wait=True, is_replay=False, speed=1)
        stream.get_latest_frame.side_effect = [frame(100), KeyboardInterrupt]
        with tempfile.TemporaryDirectory() as folder:
            session = Path(folder) / "hand.npz"
            with (
                patch("mh6_teleop_run.VisionProHandStream", return_value=stream),
                patch("mh6_teleop_run.MH6HandMapper.step", return_value=command(0, 0, 0)),
                patch("mh6_teleop_run.print_mapping_line"),
                patch("mh6_solver_evaluation.Path.write_text", side_effect=OSError("disk full")),
                redirect_stdout(io.StringIO()),
                self.assertRaises(OSError),
            ):
                main([
                    "--solver-only", "--use-default-calibration", "--record-session", str(session),
                    "--solver-summary", str(Path(folder) / "summary.json"),
                ])
            with np.load(session, allow_pickle=False) as archive:
                self.assertEqual(len(archive["timestamps"]), 1)
        stream.stop.assert_called_once()

    def test_projection_is_rejected_before_opening_a_stream(self):
        calibration = PalmAdapterCalibration(project_invalid=True, max_projection_delta_deg=(10, 30, 5))
        with tempfile.TemporaryDirectory() as folder:
            config = Path(folder) / "adapter.json"
            config.write_text(json.dumps(asdict(calibration)))
            with patch("mh6_teleop_run.VisionProHandStream") as stream, redirect_stdout(io.StringIO()):
                status = main(["--solver-only", "--palm-adapter-config", str(config)])
            stream.assert_not_called()
        self.assertEqual(status, 2)


if __name__ == "__main__":
    unittest.main()
